#include "naca_control.h"
#include "naca_debug/naca_loop_rate.h"

uint8_t _rcs_buf[25]{};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS]{};
uint16_t _raw_rc_count;
bool naca_imu_ready = false;
static bool rc_link_ok = false; // true từ khi nhận frame, false sau khi đã dừng vì mất sóng
static unsigned long last_rc_frame_ms = 0;
static int servo_left_gripper_state = 0;  // 0: closed, 1: open
static int servo_right_gripper_state = 0; // 0: closed, 1: open, 2: special close
static int servo_mid_gripper_state = 0;   // 0: closed, 1: open, 2: special close
// Sau bài tự động nút xanh: shooter/blower/servo xoay bóng và cuốn bóng tiếp tục chạy ở điều khiển tay
// (như chương trình OhStem gốc), kể cả ở SAFETY_MODE, cho đến khi gạt công tắc tương ứng hoặc mất sóng.
static bool keep_shooting = false;
static bool keep_scrolling = false;
static int keep_sw_shooter = 0; // vị trí công tắc lúc bắt đầu giữ, đổi vị trí thì thôi giữ
static int keep_sw_scroll = 0;
static unsigned long keep_start_ms = 0;
// CH16 ở điều khiển tay: thời điểm gạt lên, để chạy cùng trình tự bắn như sau bài tự động
static int prev_sw_shooter = 0;
static unsigned long manual_shoot_start_ms = 0;
// Hạ cơ cấu nâng có thời hạn (bước "hạ tay gắp" của bài nút xanh), chạy trong điều khiển tay
static bool lift_lowering = false;
static unsigned long lift_lower_start_ms = 0;
static unsigned long lift_lower_ms = 0;
void naca_control_init()
{
    Serial.begin(115200);
    naca_led_init(); // LED đỏ trong suốt lúc khởi động
    naca_motor_init();
    naca_timer_init();
    naca_motor_dump_registers(); // TODO: xóa sau khi debug pin xong
    // Hiệu chuẩn gyro mất khoảng 7 giây, robot phải đứng yên
    naca_imu_ready = naca_imu_init();
    if (!naca_imu_ready)
    {
        Serial.println("IMU not found -> headless mode and autonomous disabled");
    }
    naca_odom_reset();
    naca_auto_init();
    Serial.println("Starting CRSF Receiver...");
    ORC_SERIAL.begin(420000, SERIAL_8N1, ORC_SERIAL_RX_PIN, ORC_SERIAL_TX_PIN);
}

// Đọc Serial2, trả về true khi vừa giải mã xong 1 frame RC vào _raw_rc_values
bool naca_control_read_rc()
{
    int serial2Available = Serial2.available();
    if (serial2Available <= 0)
    {
        return false;
    }
    size_t bytesToRead = (size_t)min(serial2Available, SBUS_BUFFER_SIZE);
    size_t numBytesRead = Serial2.readBytes(_rcs_buf, bytesToRead);
    if (numBytesRead > 0 &&
        crsf_parse(&_rcs_buf[0], numBytesRead, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS))
    {
        last_rc_frame_ms = millis();
        rc_link_ok = true;
        return true;
    }
    return false;
}

// Dừng tất cả motor; cơ cấu nâng giữ ở LIFT_HOLD_SPEED để không rơi
void naca_control_stop_all()
{
    drive_omni(0, 0, 0);
    lift_motor(LIFT_HOLD_SPEED);
    scroll_motor(0);
    shooter_motor(0);
    blower_motor(0);
    servo_xoay_bong.spin(0);
    keep_shooting = false;
    keep_scrolling = false;
    lift_lowering = false;
}

// Gọi khi bài tự động nút xanh xong (bước "Bắn bóng" của chương trình gốc): điều khiển tay bật shooter,
// sau AUTO_SHOOT_STAGGER_MS bật blower, sau 2 lần AUTO_SHOOT_STAGGER_MS bắt đầu lặp servo xoay bóng; cuốn bóng chạy tiếp.
void naca_control_keep_shooting()
{
    if (!rc_link_ok)
    {
        return; // không có sóng thì không có cách nào tắt lại
    }
    keep_shooting = true;
    keep_scrolling = true;
    keep_sw_shooter = _raw_rc_values[BLOWER_SHOOTER_RC_CHANNEL - 1] > 1700 ? 1 : 0;
    keep_sw_scroll = _raw_rc_values[SCROLL_RC_CHANNEL - 1] > 1700 ? 1 : 0;
    keep_start_ms = millis();
    shooter_motor(AUTO_SHOOTER_SPEED);
    scroll_motor(AUTO_GREEN_SCROLL_SPEED);
}

// Trình tự bắn tính từ start_ms: shooter ngay, blower sau AUTO_SHOOT_STAGGER_MS, sau 2 lần AUTO_SHOOT_STAGGER_MS
// servo xoay bóng lặp quay đẩy AUTO_FEED_FORWARD_MS rồi quay ngược AUTO_FEED_REVERSE_MS
static void run_shoot_sequence(unsigned long start_ms, int shooter_speed, int blower_speed)
{
    unsigned long elapsed = millis() - start_ms;
    shooter_motor(shooter_speed);
    blower_motor(elapsed >= AUTO_SHOOT_STAGGER_MS ? blower_speed : 0);
    if (elapsed >= 2 * AUTO_SHOOT_STAGGER_MS)
    {
        unsigned long t = (elapsed - 2 * AUTO_SHOOT_STAGGER_MS) % (AUTO_FEED_FORWARD_MS + AUTO_FEED_REVERSE_MS);
        servo_xoay_bong.spin(t < AUTO_FEED_FORWARD_MS ? AUTO_SHOT_SPEED_FORWARD : AUTO_SHOT_SPEED_REVERSE);
    }
    else
    {
        servo_xoay_bong.spin(0);
    }
}

// Bước "hạ tay gắp" của bài nút xanh: điều khiển tay chạy cơ cấu nâng xuống trong ms mili giây, sau đó trả về cần
void naca_control_lower_lift(unsigned long ms)
{
    if (!rc_link_ok)
    {
        return;
    }
    lift_lowering = true;
    lift_lower_start_ms = millis();
    lift_lower_ms = ms;
    lift_motor(-100);
}

// 0: closed, 1: open, 2: special close (chỉ kẹp phải và kẹp giữa)
void naca_control_set_gripper_state(int left, int right, int mid)
{
    servo_left_gripper_state = left;
    servo_right_gripper_state = right;
    servo_mid_gripper_state = mid;
}

void naca_manual_handle_loop()
{
    static unsigned long last_print = 0;
    static NacaLoopRate rate_manual = NACA_LOOP_RATE_INIT("manual");
    naca_loop_rate_begin(rate_manual);
    if (naca_imu_ready)
    {
        naca_imu_update(0, false);
    }
    naca_odom_update();
    naca_led_update();
    if (timer_flag)
    {
        timer_flag = false;
        naca_loop_rate_timer_hit(rate_manual);
        // So khoảng thời gian thay vì % 1000: loop có thể chậm hơn 1ms và trượt mất tick chia hết cho 1000
        if (cnt_timer - last_print >= 1000)
        {
            last_print += 1000;
            Serial.print(cnt_timer);
            float battery_voltage = read_battery_voltage();
            naca_led_set_battery_low(battery_voltage < NACA_LED_BATTERY_LOW_V);
            Serial.print(" - battery voltage: " + String(battery_voltage) + "V");
            // for (int i = 10; i < RC_INPUT_MAX_CHANNELS; i++)
            // {
            //     Serial.print(" - ch" + String(i) + ": " + String(_raw_rc_values[i]));
            // }
            Serial.printf(" - enc_left: %d tick, enc_right: %d tick - odom x: %.0fmm y: %.0fmm yaw: %.1f", naca_odom_pose.left_ticks, naca_odom_pose.right_ticks, naca_odom_pose.x_mm, naca_odom_pose.y_mm, naca_odom_pose.yaw_deg);
            Serial.println();
        }
    }
    static int vx_cmd, vy_cmd, vr_cmd, v_hold_cmd;
    static int lift_state = 0;
    static int user_pilot = -1;
    static int speed_mode = SPEED_MODE_0; // 0: low speed, 1: high speed
    // naca_control_read_rc trả về true khi đã giải mã xong 1 frame RC -> chỉ lúc đó mới gửi lệnh motor qua I2C
    if (!naca_control_read_rc())
    {
        // Mất sóng: quá RC_TIMEOUT_MS không có frame thì dừng hết (chỉ gửi lệnh dừng 1 lần)
        if (rc_link_ok && millis() - last_rc_frame_ms > RC_TIMEOUT_MS)
        {
            rc_link_ok = false;
            naca_control_stop_all();
            Serial.println("RC link lost -> stop all");
        }
        if (!rc_link_ok)
        {
            naca_led_set_state(NACA_LED_NO_LINK);
        }
    }
    else
    {
        static NacaLoopRate rate_rc_frame = NACA_LOOP_RATE_INIT("rc_frame");
        naca_loop_rate_begin(rate_rc_frame);
        speed_mode = _raw_rc_values[SPEED_MODE_RC_CHANNEL - 1] > 1700 ? SPEED_MODE_1 : SPEED_MODE_0;
        if (_raw_rc_values[PILOT_MODE_RC_CHANNEL - 1] > 1700)
        {
            user_pilot = TUAT_VUONG_MODE;
        }
        else if (_raw_rc_values[PILOT_MODE_RC_CHANNEL - 1] > 1400)
        {
            user_pilot = TU_VUONG_MODE;
        }
        else
        {
            user_pilot = SAFETY_MODE;
        }
        // Chỉ cập nhật khi công tắc đổi vị trí -> công tắc gạt sau cùng thắng.
        // ch11 điều khiển cả 2 kẹp, ch4/ch7 ghi đè riêng kẹp trái/phải, ch8 kẹp giữa.
        // prev = -1 để frame đầu tiên áp dụng tất cả (ch11 trước, ch4/ch7 đè sau).
        static int prev_sw_both = -1, prev_sw_left = -1, prev_sw_right = -1, prev_sw_mid = -1;
        static bool auto_servo_state = 0;
        int sw_both = _raw_rc_values[BOTH_GRIPPER_RC_CHANNEL - 1] > 1700 ? 1 : 0;
        int sw_left = _raw_rc_values[LEFT_GRIPPER_RC_CHANNEL - 1] > 1700 ? 1 : 0;
        int sw_right = _raw_rc_values[RIGHT_GRIPPER_RC_CHANNEL - 1] > 1700 ? 1 : 0;
        int sw_mid = _raw_rc_values[MID_GRIPPER_RC_CHANNEL - 1] > 1700 ? 1 : 0;
        int sw_special = _raw_rc_values[SPECIAL_GRIPPER_RC_CHANNEL - 1] > 1700 ? 1 : 0;
        int sw_single_pickup = _raw_rc_values[SINGLE_SERVO_PICKUP_BALL_RC_CHANNEL - 1] > 1700 ? 1 : 0;
        if (user_pilot != SAFETY_MODE)
        {
            auto_servo_state = 0;
            if (user_pilot == TUAT_VUONG_MODE)
            {
                // 0: down, 1: middle, 2: up
                lift_state = _raw_rc_values[PITCH_RC_CHANNEL - 1] > 1600 ? 2 : (_raw_rc_values[PITCH_RC_CHANNEL - 1] < 1400 ? 0 : 1);
                v_hold_cmd = mapRC(_raw_rc_values[PITCH_RC_CHANNEL - 1], 100);        //
                vx_cmd = mapRC(_raw_rc_values[THROTTLE_RC_CHANNEL - 1], -speed_mode); // Channel 3: Throttle (Forward/Backward)
                vy_cmd = mapRC(_raw_rc_values[YAW_RC_CHANNEL - 1], speed_mode);       // Channel 4: Yaw (Rotation)
                vr_cmd = mapRC(_raw_rc_values[ROLL_RC_CHANNEL - 1], speed_mode);      // Channel 1: Roll (Strafe Left/Right)
            }
            else if (user_pilot == TU_VUONG_MODE)
            {
                // 0: down, 1: middle, 2: up
                lift_state = _raw_rc_values[THROTTLE_RC_CHANNEL - 1] > 1600 ? 2 : (_raw_rc_values[THROTTLE_RC_CHANNEL - 1] < 1400 ? 0 : 1);
                v_hold_cmd = mapRC(_raw_rc_values[THROTTLE_RC_CHANNEL - 1], 100); //
                // lift_state = _raw_rc_values[2] > 1700 ? (_raw_rc_values[2] > 1400 ? 2 : 1) : 0;
                vx_cmd = mapRC(_raw_rc_values[PITCH_RC_CHANNEL - 1], -speed_mode); // Channel 2: Pitch (Forward/Backward)
                vy_cmd = mapRC(_raw_rc_values[ROLL_RC_CHANNEL - 1], speed_mode);   // Channel 1: Roll (Strafe Left/Right)
                vr_cmd = mapRC(_raw_rc_values[YAW_RC_CHANNEL - 1], speed_mode);    // Channel 4: Yaw (Rotation)
            }
            if (lift_lowering && millis() - lift_lower_start_ms >= lift_lower_ms)
            {
                lift_lowering = false;
            }
            // 0: down, 1: middle, 2: up
            if (lift_lowering)
            {
                lift_motor(-100);
            }
            else if (lift_state == 0 /*|| _raw_rc_values[6] > 1700*/)
            {
                // Serial.println("lift_state == 0");
                lift_motor(v_hold_cmd + 20);
            }
            else if (lift_state == 1 /*|| _raw_rc_values[6] > 1400*/)
            {
                // Serial.println("lift_state == 1");
                lift_motor(LIFT_HOLD_SPEED);
                // motor_nang.stop();
            }
            // else if (lift_state == 0 /* || _raw_rc_values[6] < 1400*/)
            else
            {
                // Serial.println("lift_state == 2");
                lift_motor(v_hold_cmd);
            }
            //
            int sw_scroll = _raw_rc_values[SCROLL_RC_CHANNEL - 1] > 1700 ? 1 : 0;
            if (keep_scrolling && sw_scroll != keep_sw_scroll)
            {
                keep_scrolling = false; // gạt công tắc cuốn bóng -> trả về điều khiển bằng công tắc
            }
            if (keep_scrolling)
            {
                scroll_motor(AUTO_GREEN_SCROLL_SPEED);
            }
            else if (sw_scroll)
            {
                scroll_motor(100);
            }
            else
            {
                scroll_motor(0);
            }
            if (sw_mid != prev_sw_mid)
            {
                servo_mid_gripper_state = sw_mid;
                prev_sw_mid = sw_mid;
            }
            if (sw_both != prev_sw_both)
            {
                servo_left_gripper_state = sw_both;
                servo_right_gripper_state = sw_both;
                prev_sw_both = sw_both;
            }
            if (sw_left != prev_sw_left)
            {
                servo_left_gripper_state = sw_left;
                prev_sw_left = sw_left;
            }
            if (sw_right != prev_sw_right)
            {
                servo_right_gripper_state = sw_right;
                prev_sw_right = sw_right;
            }
            // // handle shooter motor + blower motor
            int sw_shooter = _raw_rc_values[BLOWER_SHOOTER_RC_CHANNEL - 1] > 1700 ? 1 : 0;
            if (keep_shooting && sw_shooter != keep_sw_shooter)
            {
                keep_shooting = false; // gạt công tắc shooter -> trả về điều khiển bằng công tắc
            }
            if (sw_shooter && !prev_sw_shooter)
            {
                manual_shoot_start_ms = millis(); // vừa gạt lên -> bắt đầu trình tự bắn
            }
            prev_sw_shooter = sw_shooter;
            if (keep_shooting)
            {
                run_shoot_sequence(keep_start_ms, AUTO_SHOOTER_SPEED, AUTO_BLOWER_SPEED);
            }
            else if (sw_shooter)
            {
                run_shoot_sequence(manual_shoot_start_ms, AUTO_SHOOTER_SPEED, AUTO_BLOWER_SPEED);
            }
            else
            {
                shooter_motor(0);
                blower_motor(0);
                servo_xoay_bong.spin(0);
            }
        }
        // AUTO_MODE: không arm -> tự động chạy trình tự bắn bóng, cuốn bóng, hạ tay gắp (nếu có) ở SAFETY_MODE
        else
        {
            auto_servo_state = 1;
            vx_cmd = 0;
            vy_cmd = 0;
            vr_cmd = 0;
            // Tự động chạy khi không arm -> sau bài xanh vẫn giữ hạ tay gắp / cuốn bóng / bắn bóng ở SAFETY_MODE,
            // dừng khi hết thời gian hạ tay, gạt công tắc tương ứng hoặc mất sóng.
            if (lift_lowering && millis() - lift_lower_start_ms >= lift_lower_ms)
            {
                lift_lowering = false;
            }
            // lift_motor(LIFT_HOLD_SPEED); // giữ để cơ cấu nâng không rơi
            lift_motor(lift_lowering ? -100 : 0); // không hạ thì dừng hẳn để motor không nóng khi SAFETY_MODE
            int sw_scroll = _raw_rc_values[SCROLL_RC_CHANNEL - 1] > 1700 ? 1 : 0;
            if (keep_scrolling && sw_scroll != keep_sw_scroll)
            {
                keep_scrolling = false;
            }
            scroll_motor(keep_scrolling ? AUTO_GREEN_SCROLL_SPEED : 0);
            int sw_shooter = _raw_rc_values[BLOWER_SHOOTER_RC_CHANNEL - 1] > 1700 ? 1 : 0;
            if (keep_shooting && sw_shooter != keep_sw_shooter)
            {
                keep_shooting = false;
            }
            if (keep_shooting)
            {
                run_shoot_sequence(keep_start_ms, AUTO_SHOOTER_SPEED, AUTO_BLOWER_SPEED);
            }
            else
            {
                shooter_motor(0);
                blower_motor(0);
                servo_xoay_bong.spin(0);
            }
            prev_sw_shooter = 0; // kích hoạt lại mà CH16 đang bật thì chạy lại trình tự bắn từ đầu
            // Công tắc gạt trong lúc SAFETY không được tính là sườn khi kích hoạt lại.
            // Chưa kích hoạt lần nào (prev = -1) thì giữ nguyên để frame kích hoạt đầu tiên áp dụng tất cả.f
            if (prev_sw_both != -1)
            {
                prev_sw_both = sw_both;
                prev_sw_left = sw_left;
                prev_sw_right = sw_right;
                prev_sw_mid = sw_mid;
            }
        }
        // AUTO_MODE: không arm -> tự động chạy trình tự bắn bóng, cuốn bóng, hạ tay gắp (nếu có) ở SAFETY_MODE

        // Mode không đầu: vx/vy tính theo hướng robot tại lúc bật công tắc, không theo đầu robot
        static bool prev_headless = false;
        bool headless = naca_imu_ready && _raw_rc_values[HEADLESS_RC_CHANNEL - 1] > 1700;
        if (headless && !prev_headless)
        {
            naca_imu_reset_yaw();
        }
        prev_headless = headless;
        if (headless)
        {
            float yaw_rad = HEADLESS_YAW_SIGN * naca_imu_get_snapshot().yaw * DEG_TO_RAD;
            int vx_field = vx_cmd;
            int vy_field = vy_cmd;
            vx_cmd = lroundf(vx_field * cosf(yaw_rad) - vy_field * sinf(yaw_rad));
            vy_cmd = lroundf(vx_field * sinf(yaw_rad) + vy_field * cosf(yaw_rad));
        }
        naca_led_set_state(user_pilot == SAFETY_MODE ? NACA_LED_SAFETY : (headless ? NACA_LED_HEADLESS : NACA_LED_ARMED));

        // handle omni drive
        // Nhả cần Vx hoặc Vr về 0 (tất cả cần về giữa) -> phanh VX_BRAKE_MS / VR_BRAKE_MS rồi mới thả.
        // Nếu nhả cả Vx và Vr cùng lúc thì lấy thời gian phanh dài hơn.
        static int prev_vx_cmd = 0;
        static int prev_vr_cmd = 0;
        static unsigned long brake_start = 0;
        static unsigned long brake_ms = 0;
        static bool braking = false;
        bool idle = (vx_cmd == 0 && vy_cmd == 0 && vr_cmd == 0);
        if (idle && (prev_vx_cmd != 0 || prev_vr_cmd != 0))
        {
            unsigned long vx_ms = (prev_vx_cmd != 0) ? VX_BRAKE_MS : 0;
            unsigned long vr_ms = (prev_vr_cmd != 0) ? VR_BRAKE_MS : 0;
            braking = true;
            brake_start = millis();
            brake_ms = (vx_ms > vr_ms) ? vx_ms : vr_ms;
        }
        if (!idle || millis() - brake_start >= brake_ms)
        {
            braking = false;
        }
        prev_vx_cmd = vx_cmd;
        prev_vr_cmd = vr_cmd;
        if (braking)
        {
            brake_omni();
        }
        else
        {
            drive_omni(vx_cmd, vy_cmd, vr_cmd);
        }

        // handle servo gripper
        if (auto_servo_state == 1)
        {
            servo_left_gripper.spin(left_servo_initial_ms);
            servo_right_gripper.spin(right_servo_initial_ms);
            servo_mid_gripper.spin(mid_servo_initial_ms);
        }
        else if (auto_servo_state == 0)
        {
            if (sw_single_pickup)
            {
                // Xét riêng từng bên: state chỉ có 0/1 nên không dùng chung 1 chuỗi else-if được
                bool left_on = (servo_left_gripper_state == 1);
                bool right_on = (servo_right_gripper_state == 1);
                servo_left_gripper.spin(left_on ? single_left_servo_pickup_ball_close_ms : single_left_servo_pickup_ball_open_ms);
                servo_right_gripper.spin(right_on ? single_right_servo_pickup_ball_close_ms : single_right_servo_pickup_ball_open_ms);
                // Kẹp giữa nghiêng về bên đang kẹp; cả 2 bên cùng bật (hoặc cùng tắt) thì kẹp giữa mở
                if (left_on && !right_on)
                {
                    servo_mid_gripper.spin(single_mid_left_servo_pickup_ball_close_ms);
                }
                else if (right_on && !left_on)
                {
                    servo_mid_gripper.spin(single_mid_right_servo_pickup_ball_close_ms);
                }
                else
                {
                    servo_mid_gripper.spin(mid_servo_open_ms);
                }
            }
            else
            {
                switch (sw_special)
                {
                case 1:
                {
                    if (servo_left_gripper_state == 1)
                    {
                        servo_left_gripper.spin(left_servo_close_ms);
                    }
                    else if (servo_left_gripper_state == 0)
                    {
                        servo_left_gripper.spin(left_servo_dual_pickup_ball_close_ms);
                    }
                    if (servo_right_gripper_state == 1)
                    {
                        servo_right_gripper.spin(right_servo_close_ms);
                    }
                    else if (servo_right_gripper_state == 0)
                    {
                        servo_right_gripper.spin(right_servo_dual_pickup_ball_close_ms);
                    }
                    break;
                }
                case 0:
                {
                    if (servo_left_gripper_state == 1)
                    {
                        servo_left_gripper.spin(left_servo_open_ms);
                    }
                    else if (servo_left_gripper_state == 0)
                    {
                        servo_left_gripper.spin(left_servo_close_ms);
                    }
                    if (servo_right_gripper_state == 1)
                    {
                        servo_right_gripper.spin(right_servo_open_ms);
                    }
                    else if (servo_right_gripper_state == 0)
                    {
                        servo_right_gripper.spin(right_servo_close_ms);
                    }
                    break;
                }
                }

                if (servo_mid_gripper_state == 0)
                {
                    servo_mid_gripper.spin(mid_servo_open_ms);
                }
                else if (servo_mid_gripper_state == 1)
                {
                    servo_mid_gripper.spin(mid_servo_close_ms);
                }
            }
        }
        naca_loop_rate_end(rate_rc_frame);
    }
    naca_loop_rate_end(rate_manual);
}
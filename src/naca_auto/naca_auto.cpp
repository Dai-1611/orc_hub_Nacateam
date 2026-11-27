#include "naca_auto.h"
#include "naca_control/naca_control.h"
#include "naca_debug/naca_loop_rate.h"

struct AutoButton
{
    uint8_t pin;
    bool pressed;
    bool last_raw;
    unsigned long changed_ms;
};
static AutoButton btn_green = {AUTO_BTN_GREEN_PIN, false, false, 0};
static AutoButton btn_red = {AUTO_BTN_RED_PIN, false, false, 0};
static bool auto_aborted = false;
// Debug tần số: auto_service chạy liên tục trong lúc auto, auto_ctl là bước điều khiển mỗi AUTO_CONTROL_PERIOD_MS
static NacaLoopRate rate_auto_service = NACA_LOOP_RATE_INIT("auto_svc");
static NacaLoopRate rate_auto_ctl = NACA_LOOP_RATE_INIT("auto_ctl");

// CH5 > 1400 là arm (điều khiển tay). Tự động chỉ chạy khi không arm.
static bool auto_rc_is_armed()
{
    return _raw_rc_values[PILOT_MODE_RC_CHANNEL - 1] > 1400;
}

void naca_auto_init()
{
    pinMode(AUTO_BTN_GREEN_PIN, INPUT_PULLUP);
    pinMode(AUTO_BTN_RED_PIN, INPUT_PULLUP);
}

// Nút nối xuống GND (nhấn = LOW). Trả về 1 khi vừa nhấn, -1 khi vừa nhả, 0 nếu không đổi.
static int button_edge(AutoButton &btn)
{
    bool raw = digitalRead(btn.pin) == LOW;
    if (raw != btn.last_raw)
    {
        btn.last_raw = raw;
        btn.changed_ms = millis();
    }
    if (raw != btn.pressed && millis() - btn.changed_ms >= AUTO_BTN_DEBOUNCE_MS)
    {
        btn.pressed = raw;
        return raw ? 1 : -1;
    }
    return 0;
}

// Gọi liên tục trong lúc chạy tự động: cập nhật IMU (giữ hướng), odometry và kiểm tra lệnh dừng.
// Trả về false nếu bài tự động đã bị hủy.
static bool auto_service()
{
    if (auto_aborted)
    {
        return false;
    }
    naca_loop_rate_begin(rate_auto_service);
    naca_imu_update(0, true);
    naca_odom_update();
    naca_led_update();
    if (naca_control_read_rc() && auto_rc_is_armed())
    {
        auto_aborted = true; // gạt arm -> hủy tự động, về điều khiển tay
    }
    int green_edge = button_edge(btn_green);
    int red_edge = button_edge(btn_red);
    if (green_edge == 1 || red_edge == 1)
    {
        auto_aborted = true;
    }
    naca_loop_rate_end(rate_auto_service);
    return !auto_aborted;
}

static bool auto_wait(unsigned long ms)
{
    unsigned long start = millis();
    while (millis() - start < ms)
    {
        if (!auto_service())
        {
            return false;
        }
    }
    return true;
}

// Lệnh xoay để giữ yaw_target, lấy từ PID trong naca_imu_update
static int auto_hold_vr()
{
    return constrain(AUTO_YAW_PID_SIGN * vr_final, -AUTO_VR_MAX, AUTO_VR_MAX);
}

// Chạy (forward, right) có giữ hướng cho đến khi bánh RR lăn đủ distance_mm (giống chương trình gốc đo bằng E2).
// Khởi động mềm: tốc độ tăng tuyến tính từ AUTO_MOVE_MIN_SPEED lên tốc độ đặt trong AUTO_SOFT_START_MS.
// Không tự dừng motor khi xong: nơi gọi chọn phanh hay thả.
static bool auto_move(int forward, int right, float distance_mm)
{
    const int base_speed = max(abs(forward), abs(right));
    const float min_scale = base_speed > 0 ? (float)min(AUTO_MOVE_MIN_SPEED, base_speed) / base_speed : 1.0f;
    int32_t rl_ticks, rr_start, rr_ticks;
    naca_motor_read_encoders(&rl_ticks, &rr_start);
    unsigned long start = millis();
    unsigned long last_step = 0;
    while (true)
    {
        if (!auto_service())
        {
            return false;
        }
        if (millis() - last_step < AUTO_CONTROL_PERIOD_MS)
        {
            continue;
        }
        last_step = millis();
        naca_loop_rate_begin(rate_auto_ctl);
        naca_motor_read_encoders(&rl_ticks, &rr_ticks);
        if (fabsf((float)(rr_ticks - rr_start)) * NACA_ODOM_MM_PER_TICK >= distance_mm)
        {
            return true;
        }
        if (millis() - start > AUTO_STEP_TIMEOUT_MS)
        {
            Serial.println("Auto: move timeout");
            auto_aborted = true;
            return false;
        }
        unsigned long elapsed = millis() - start;
        float scale = elapsed < AUTO_SOFT_START_MS ? min_scale + (1.0f - min_scale) * elapsed / AUTO_SOFT_START_MS : 1.0f;
        drive_omni(lroundf(AUTO_FORWARD_SIGN * forward * scale), lroundf(AUTO_RIGHT_SIGN * right * scale), auto_hold_vr());
        naca_loop_rate_end(rate_auto_ctl);
    }
}

// Giống auto_move nhưng đo quãng đường bằng odometry, theo hướng (forward, right) tính từ điểm bắt đầu.
// Khởi động mềm: trong AUTO_SOFT_START_MS đầu, tốc độ tối đa tăng tuyến tính từ AUTO_MOVE_MIN_SPEED lên tốc độ đặt.
// Còn cách đích dưới AUTO_MOVE_SLOW_MM thì hạ tốc dần (không thấp hơn AUTO_MOVE_MIN_SPEED);
// nếu lố thì chạy ngược lại ở tốc độ thấp. Xong khi cách đích dưới AUTO_MOVE_TOL_MM.
// Cần hiệu chỉnh odometry trước khi dùng. Không tự dừng motor khi xong: nơi gọi chọn phanh hay thả.
static bool naca_auto_move(int forward, int right, float distance_mm, float auto_move_slow_mm, int _auto_move_min_speed)
{
    const int base_speed = max(abs(forward), abs(right));
    if (base_speed == 0)
    {
        return true;
    }
    const float x_start = naca_odom_pose.x_mm;
    const float y_start = naca_odom_pose.y_mm;
    // Hướng cần đi trong hệ tọa độ odometry (cùng công thức xoay với naca_odom_update).
    // Yaw được giữ cố định khi chạy nên lấy yaw lúc bắt đầu.
    const float yaw_rad = NACA_ODOM_YAW_SIGN * naca_odom_pose.yaw_deg * DEG_TO_RAD;
    const float f = (float)forward / base_speed;
    const float r = (float)right / base_speed;
    float ux = f * cosf(yaw_rad) + r * sinf(yaw_rad);
    float uy = -f * sinf(yaw_rad) + r * cosf(yaw_rad);
    const float u_len = sqrtf(ux * ux + uy * uy);
    ux /= u_len;
    uy /= u_len;
    unsigned long start = millis();
    unsigned long last_step = 0;
    while (true)
    {
        if (!auto_service())
        {
            return false;
        }
        if (millis() - last_step < AUTO_CONTROL_PERIOD_MS)
        {
            continue;
        }
        last_step = millis();
        naca_loop_rate_begin(rate_auto_ctl);
        // Quãng đã đi theo hướng cần đi (âm nếu đi ngược), và phần còn lại (âm nếu đã lố)
        float progress = (naca_odom_pose.x_mm - x_start) * ux + (naca_odom_pose.y_mm - y_start) * uy;
        float remaining = distance_mm - progress;
        if (fabsf(remaining) < AUTO_MOVE_TOL_MM)
        {
            return true;
        }
        if (millis() - start > AUTO_STEP_TIMEOUT_MS)
        {
            Serial.println("Auto: odom move timeout");
            auto_aborted = true;
            return false;
        }
        // Tỉ lệ tốc độ: 1 khi còn xa, giảm tuyến tính trong vùng AUTO_MOVE_SLOW_MM, âm khi lố
        float scale = constrain(remaining / auto_move_slow_mm, -1.0f, 1.0f);
        float min_scale = (float)min(_auto_move_min_speed, base_speed) / base_speed;
        // Khởi động mềm: giới hạn tốc độ tăng dần từ min_scale lên 1
        unsigned long elapsed = millis() - start;
        if (elapsed < AUTO_SOFT_START_MS)
        {
            float ramp = min_scale + (1.0f - min_scale) * elapsed / AUTO_SOFT_START_MS;
            scale = constrain(scale, -ramp, ramp);
        }
        if (fabsf(scale) < min_scale)
        {
            scale = remaining > 0 ? min_scale : -min_scale;
        }
        drive_omni(lroundf(AUTO_FORWARD_SIGN * forward * scale), lroundf(AUTO_RIGHT_SIGN * right * scale), auto_hold_vr());
        naca_loop_rate_end(rate_auto_ctl);
    }
}

// Xoay tại chỗ về yaw_target (hướng lúc bắt đầu bài tự động)
static bool auto_hold_heading()
{
    unsigned long start = millis();
    unsigned long last_step = 0;
    while (millis() - start < AUTO_TURN_TIMEOUT_MS)
    {
        if (!auto_service())
        {
            return false;
        }
        if (millis() - last_step < AUTO_CONTROL_PERIOD_MS)
        {
            continue;
        }
        last_step = millis();
        float err = yaw_target - naca_imu_get_snapshot().yaw;
        if (err > 180.0f)
            err -= 360.0f;
        if (err < -180.0f)
            err += 360.0f;
        if (fabsf(err) < AUTO_HEADING_TOL_DEG)
        {
            break;
        }
        drive_omni(0, 0, auto_hold_vr());
    }
    drive_omni(0, 0, 0);
    return true;
}

// "Hạ tay gắp": đưa 3 kẹp về góc gắp rồi chạy cơ cấu nâng xuống
static bool auto_lower_gripper()
{
    servo_mid_gripper.spin(AUTO_LOWER_MID_SERVO);
    servo_right_gripper.spin(AUTO_LOWER_RIGHT_SERVO);
    servo_left_gripper.spin(AUTO_LOWER_LEFT_SERVO);
    if (!auto_wait(500))
        return false;
    lift_motor(-100);
    bool ok = auto_wait(AUTO_LIFT_DOWN_MS);
    lift_motor(LIFT_HOLD_SPEED);
    return ok;
}

// Nút xanh: lùi 83 cm, bật cuốn bóng, lùi 9 bước 8 cm để cuốn bóng, bắn bóng, hạ tay gắp
static bool auto_green()
{
    // step 0: đợi 500ms
    if (!auto_wait(500))
        return false;

    // step 1: đi lùi {AUTO_GREEN_BACK_MM} tới nơi lấy bóng với tốc độ {AUTO_GREEN_BACK_SPEED}
    if (!naca_auto_move(-AUTO_GREEN_BACK_SPEED, 0, AUTO_GREEN_BACK_MM, 400, 25))
        return false;
    brake_omni();

    if (!auto_wait(200))
        return false;

    if (!naca_auto_move(0, -AUTO_GREEN_BACK_SPEED, 40, 10, 30))
        return false;
    brake_omni();
    // step 2: bật cuốn bóng
    if (!auto_wait(500))
        return false;
    scroll_motor(AUTO_GREEN_SCROLL_SPEED);

    // step 3: đi lùi 9 bước, mỗi bước {AUTO_GREEN_STEP_MM} với tốc độ {AUTO_GREEN_STEP_SPEED}
    for (int i = 0; i < AUTO_GREEN_STEP_COUNT; i++)
    {
        if (!naca_auto_move(-AUTO_GREEN_STEP_SPEED, 0, AUTO_GREEN_STEP_MM, 50, 40))
            return false;
        // drive_omni(0, 0, 0);
        brake_omni();
        if (!auto_wait(500))
            return false;
        // if (i == 2 && !auto_hold_heading())
        //     return false;
    }
    drive_omni(0, 0, 0);
    
    if (!auto_wait(500))
        return false;

    if (!naca_auto_move(-AUTO_GREEN_STEP_SPEED, 0, 40, 10, 40))
        return false;
        // drive_omni(0, 0, 0);
    brake_omni();


    if (!auto_wait(500))
        return false;

    // step 4: đi tiến {AUTO_GREEN_FORWARD_MM} với tốc độ {AUTO_GREEN_FORWARD_SPEED}
    if (!naca_auto_move(AUTO_GREEN_FORWARD_SPEED, 0, AUTO_GREEN_FORWARD_MM, 150, AUTO_GREEN_FORWARD_SPEED - 10))
        return false;
    brake_omni();
    /*
    // step 5: đi sang trái {AUTO_GREEN_LEFT_MM} với tốc độ {AUTO_GREEN_STEP_SPEED}
    if (!naca_auto_move(0, -AUTO_GREEN_LEFT_SPEED, AUTO_GREEN_LEFT_MM))
        return false;
    brake_omni();
    // step 6 đợi 600ms, phanh omni
    if (!auto_wait(600))
        return false;
    brake_omni();
    */
    // step 7: đợi 100ms
    if (!auto_wait(100))
        return false;

    // step 8, 9: bắn bóng và hạ tay gắp do điều khiển tay xử lý sau khi bài tự động kết thúc
    // (chương trình gốc gán auto = 0 trước 2 bước này), xem naca_auto_handle_loop
    return true;
}

// Nút đỏ: đi ngang sang trái 74 cm, tiến 25 cm, hạ tay gắp
static bool auto_red()
{
    if (!auto_wait(500))
        return false;
    if (!auto_move(0, -AUTO_RED_LEFT_SPEED, AUTO_RED_LEFT_MM))
        return false;
    brake_omni();
    if (!auto_wait(500))
        return false;
    if (!auto_move(AUTO_RED_FORWARD_SPEED, 0, AUTO_RED_FORWARD_MM))
        return false;
    brake_omni();
    return auto_lower_gripper();
}

bool naca_auto_handle_loop()
{
    int green_edge = button_edge(btn_green);
    int red_edge = button_edge(btn_red);
    bool run_green = green_edge == 1;
    bool run_red = red_edge == -1;
    if (!run_green && !run_red)
    {
        return false;
    }
    if (auto_rc_is_armed())
    {
        Serial.println("Auto: armed -> manual mode, skipped");
        return false;
    }
    if (!naca_imu_ready)
    {
        Serial.println("Auto: no IMU, cannot hold heading -> skipped");
        return false;
    }
    Serial.println(run_green ? "Auto green: ball shooting" : "Auto red: block picking");
    naca_led_set_state(run_green ? NACA_LED_AUTO_GREEN : NACA_LED_AUTO_RED);
    auto_aborted = false;
    naca_control_stop_all();
    yaw_target = naca_imu_get_snapshot().yaw; // giữ hướng hiện tại trong suốt bài tự động

    bool ok = run_green ? auto_green() : auto_red();

    naca_control_stop_all();
    if (ok)
    {
        // Sau khi hạ tay gắp: kẹp trái/phải mở, kẹp giữa đóng (như chương trình gốc)
        // naca_control_set_gripper_state(1, 1, 0);
        if (run_green)
        {
            // Như chương trình gốc: gán biến rồi để điều khiển tay bắn bóng và hạ tay gắp
            naca_control_keep_shooting();
            // naca_control_lower_lift(AUTO_LIFT_DOWN_MS);
        }
    }
    Serial.println(ok ? "Auto: done" : "Auto: aborted");
    naca_led_set_state(NACA_LED_NO_LINK); // loop điều khiển tay sẽ đặt lại theo frame RC kế tiếp
    return true;
}

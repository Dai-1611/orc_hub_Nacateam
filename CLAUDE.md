# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Tổng quan

Firmware cho robot thi đấu của Nacateam, gồm hệ di chuyển 4 bánh mecanum, cơ cấu nâng, cơ cấu cuốn bóng, shooter, blower và ba servo kẹp. Firmware chạy trên OhStem Yolo Uno (ESP32-S3, Arduino framework) và được điều khiển từ xa qua sóng RC giao thức CRSF. Hai nút D7/D8 chạy hai bài tự động (bắn bóng, gắp khối).

Thư mục này là git repository, nhưng mới chỉ có một commit ("initial commit"), nên lịch sử gần như không giúp so sánh hoặc khôi phục.

## Lệnh

PlatformIO chỉ có một environment là `yolo_uno`. Nếu `pio` không có trong PATH, hãy dùng `~/.platformio/penv/Scripts/pio.exe`.

```sh
pio run                 # build
pio run -t upload       # nạp firmware (board kết nối qua native USB)
pio device monitor      # xem log serial ở 115200 qua USB CDC
pio run -t clean
```

Dự án không có unit test, thư mục `test/` hay cấu hình linter. `pio run` là bước kiểm tra tự động duy nhất; hành vi thực tế chỉ có thể xác nhận trên robot.

## Kiến trúc

`src/main.cpp` chỉ làm nhiệm vụ điều phối: `setup()` gọi `naca_control_init()`, còn `loop()` gọi `naca_auto_handle_loop()` (chạy bài tự động khi có nút D7/D8, chặn đến khi xong) và nếu không có thì gọi `naca_manual_handle_loop()`.

Luồng điều khiển đi qua các lớp sau:

1. **`lib/crsf`**: bộ parser CRSF được port từ PX4. `crsf_parse()` giữ trạng thái parser giữa các lần gọi và chỉ trả về true khi đã giải mã xong một frame RC hoàn chỉnh.
2. **`src/naca_control`**: toàn bộ logic teleop nằm trong `naca_manual_handle_loop()`. Hàm này đọc `Serial2`, rồi ánh xạ các kênh thành lệnh di chuyển và trạng thái cơ cấu bằng các biến `static` cục bộ trong hàm.
3. **`src/naca_motor`**: quản lý các đối tượng phần cứng và gán chân, cùng với `mapRC`, động học mecanum (hàm vẫn tên `omni_kinematics`) và các hàm điều khiển motor.
4. **`lib/OhStemRobotics`** (`ohstem::robotics`): bản C++ port từ runtime MicroPython của OhStem. Firmware hiện tại chỉ dùng `MotorDriverV2`, `DCMotor` và `ServoMotor`; `DriveBase`, `Gamepad`, các lớp IMU và cảm biến line không được dùng.

`src/naca_timer` chạy ISR hardware timer 0 chu kỳ 1 ms để bật `timer_flag` và tăng `cnt_timer`. Timer này chỉ dùng để định nhịp dòng log trạng thái mỗi giây, không tham gia logic điều khiển.

`src/naca_debug/naca_loop_rate.h` đo tần số và thời gian xử lý của vòng lặp manual, frame RC và auto, rồi in ra Serial mỗi `NACA_LOOP_RATE_PRINT_MS`. Đặt `NACA_LOOP_RATE_DEBUG` về 0 để tắt.

`src/naca_imu` đọc LSM6DSR qua SPI 3 dây (CS 1, SDA 3, SCLK 2), tích phân yaw theo tick 1 ms của hardware timer 1 và có PID giữ hướng. `naca_imu_init()` hiệu chuẩn gyro khoảng 7 giây lúc khởi động (robot phải đứng yên) và trả về false nếu không thấy chip; khi đó `naca_imu_ready` là false, mode không đầu và chế độ tự động bị tắt.

`src/naca_odom` fuse 2 encoder bánh sau (E1 = RL, E2 = RR) với yaw IMU thành `naca_odom_pose` (x, y theo mm, in trong dòng log mỗi giây). Các hằng số encoder, dấu và hệ số xoay trong `naca_odom.h` chưa được hiệu chỉnh trên robot.

`src/naca_led` báo trạng thái máy bằng LED RGB WS2812B (GPIO 45) và LED D13 (GPIO 48): đỏ khi khởi động, xanh dương khi chưa có hoặc mất sóng, vàng ở `SAFETY_MODE`, xanh lá khi kích hoạt, xanh ngọc khi bật mode không đầu, trắng/tím khi chạy bài tự động nút xanh/đỏ, nháy đỏ xen kẽ khi pin dưới `NACA_LED_BATTERY_LOW_V`. LED D13 nháy đều khi điều khiển tay và sáng liên tục khi chạy tự động.

`src/naca_auto` là bản port hai bài tự động từ chương trình OhStem Blockly (XanhNgoc2): nút D7 chạy khi nhấn (bắn bóng), nút D8 chạy khi nhả (gắp khối). Bài tự động chỉ bắt đầu khi CH5 ở `SAFETY_MODE` (`<=1400`, không kích hoạt) và có IMU. Bài chạy kiểu chặn, giữ hướng lúc bắt đầu bằng PID của IMU, và bị hủy khi:

- CH5 được gạt sang kích hoạt (`>1400`);
- nhấn lại một trong hai nút;
- một bước di chuyển quá `AUTO_STEP_TIMEOUT_MS`.

Mọi thông số nằm trong `naca_auto.h`.

- **Bài nút xanh:** lùi `AUTO_GREEN_BACK_MM`, đi ngang 40 mm, bật cuốn bóng, lùi `AUTO_GREEN_STEP_COUNT` bước `AUTO_GREEN_STEP_MM`, lùi thêm 40 mm rồi tiến `AUTO_GREEN_FORWARD_MM`. Bước đi ngang sang trái `AUTO_GREEN_LEFT_MM` đang bị comment.
- **Kết thúc bài xanh:** như chương trình gốc (gán `auto = 0` trước "Bắn bóng"), bài kết thúc ngay sau phần di chuyển rồi gọi `naca_control_keep_shooting()`. Shooter, blower, servo xoay bóng (lặp quay tới/lui) và cuốn bóng tiếp tục chạy trong điều khiển tay, kể cả ở `SAFETY_MODE`. Chúng chỉ dừng khi gạt công tắc tương ứng (đổi vị trí so với lúc bài kết thúc) hoặc khi mất sóng.
- **Hạ tay gắp sau bài xanh:** lệnh gọi `naca_control_lower_lift(AUTO_LIFT_DOWN_MS)` đang bị comment, nên bài xanh không hạ cơ cấu nâng.
- **Bài nút đỏ:** đi ngang sang trái `AUTO_RED_LEFT_MM`, tiến `AUTO_RED_FORWARD_MM`, rồi `auto_lower_gripper()`: đặt 3 servo kẹp, hạ cơ cấu nâng trong `AUTO_LIFT_DOWN_MS` và giữ ở `LIFT_HOLD_SPEED`.

Lệnh motor chỉ được gửi qua I2C khi giải mã xong một frame CRSF, nên lưu lượng I2C này đi theo tốc độ frame RC chứ không theo tốc độ của `loop()`.

### Sơ đồ phần cứng

| Chức năng           | Kết nối                                                                                                           |
| ------------------- | ----------------------------------------------------------------------------------------------------------------- |
| Bộ thu CRSF         | `ORC_SERIAL` = `Serial2`, 420000 baud, RX 8 (D5), TX 9 (D6), define trong `nacateam_crsf_define.h`                |
| Motor driver        | `MotorDriverV2`, I2C `0x54`, SDA 11, SCL 12, 400 kHz                                                              |
| Bánh mecanum        | FL `M1` (đảo dấu trong `drive_omni`: `run(-FL)`), FR `M4`, RL `E1`, RR `E2` (ba bánh này đảo chiều qua constructor) |
| Encoder             | RL `E1`, RR `E2`, `setEncoder(600, 11, 34)`                                                                       |
| Nâng / cuốn bóng    | `M2` / `M3`                                                                                                       |
| Chân H-bridge local | M3 ở GPIO 5, 4; M4 ở GPIO 14, 13                                                                                  |
| Shooter / blower    | Shooter GPIO 7 (D3), kênh LEDC 2; blower GPIO 6 (D4), kênh LEDC 3; 1 kHz, 8-bit (comment "20kHz" trong code đã lỗi thời) |
| Servo               | kẹp giữa `S1`, kẹp phải `S2`, kẹp trái `S3`, xoay bóng `S4` (servo 360°, lặp quay tới/lui khi bắn)                |
| IMU                 | LSM6DSR, SPI 3 dây, CS 1, SDA 3, SCLK 2                                                                           |
| Nút tự động         | D7 = GPIO 0 (`AUTO_BTN_GREEN_PIN`, đang thử; bản gốc GPIO 10 bị comment), D8 = GPIO 17, `INPUT_PULLUP`, nhấn = LOW |
| LED                 | WS2812B GPIO 45 (`NACA_LED_RGB_PIN`), D13 GPIO 48                                                                 |

`MotorDriverV2::setMotors()` gửi lệnh `M3` và `M4` qua I2C, đồng thời điều khiển luôn các chân GPIO local của chúng.

### Kênh RC

Mỗi kênh là một define `*_RC_CHANNEL` trong `naca_control.h`, đánh số từ 1 giống mix monitor của tay điều khiển (CH1, CH2, ...), và được đọc bằng `_raw_rc_values[X_RC_CHANNEL - 1]`. Muốn đổi kênh thì sửa define, không sửa chỉ số trong code. Công tắc được coi là bật khi `>1700`.

| Kênh | Define                                | Chức năng                                                                                                |
| ---- | ------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| CH1  | `ROLL_RC_CHANNEL`                     | `TU_VUONG_MODE`: đi ngang (`vy`). `TUAT_VUONG_MODE`: xoay (`vr`)                                         |
| CH2  | `PITCH_RC_CHANNEL`                    | `TU_VUONG_MODE`: tiến/lùi (`vx`). `TUAT_VUONG_MODE`: nâng                                                |
| CH3  | `THROTTLE_RC_CHANNEL`                 | `TU_VUONG_MODE`: nâng. `TUAT_VUONG_MODE`: tiến/lùi (`vx`)                                                |
| CH4  | `YAW_RC_CHANNEL`                      | `TU_VUONG_MODE`: xoay (`vr`). `TUAT_VUONG_MODE`: đi ngang (`vy`)                                         |
| CH5  | `PILOT_MODE_RC_CHANNEL`               | chọn kiểu cần và kích hoạt: `>1700` `TUAT_VUONG_MODE`, `>1400` `TU_VUONG_MODE`, còn lại `SAFETY_MODE`    |
| CH6  | `BOTH_GRIPPER_RC_CHANNEL`             | cả kẹp trái và kẹp phải                                                                                  |
| CH7  | `MID_GRIPPER_RC_CHANNEL`              | kẹp giữa                                                                                                 |
| CH8  | `LEFT_GRIPPER_RC_CHANNEL`             | kẹp trái                                                                                                 |
| CH9  | `RIGHT_GRIPPER_RC_CHANNEL`            | kẹp phải                                                                                                 |
| CH10 | `SINGLE_SERVO_PICKUP_BALL_RC_CHANNEL` | kẹp bóng đơn: kẹp trái/phải dùng vị trí `single_*`, kẹp giữa nghiêng về bên đang kẹp                     |
| CH11 | `HEADLESS_RC_CHANNEL`                 | mode không đầu, lấy hướng robot lúc bật công tắc làm hướng tiến (cần IMU)                                |
| CH13 | `SPECIAL_GRIPPER_RC_CHANNEL`          | kẹp trái/phải dùng vị trí kẹp 2 bóng: đóng → `*_dual_pickup_ball_close_ms`, mở → `*_close_ms`            |
| CH14 | `SPEED_MODE_RC_CHANNEL`               | tốc độ: `>1700` `SPEED_MODE_1` (150), còn lại `SPEED_MODE_0` (50)                                        |
| CH15 | `SCROLL_RC_CHANNEL`                   | cuốn bóng (100)                                                                                          |
| CH16 | `BLOWER_SHOOTER_RC_CHANNEL`           | trình tự bắn: shooter ngay, blower sau `AUTO_SHOOT_STAGGER_MS`, servo xoay bóng lặp tới/lui sau 2 lần đó |

CH12 hiện không dùng.

**Cơ cấu nâng** dùng cần nâng của mode đang chọn (CH3 ở `TU_VUONG_MODE`, CH2 ở `TUAT_VUONG_MODE`):

- `>1600`: chạy lên với `mapRC(cần, 100)`, tỉ lệ theo độ đẩy cần.
- `<1400`: chạy xuống với `mapRC(cần, 100) + 20`.
- Ở giữa: giữ ở `LIFT_HOLD_SPEED` (20).

**Kẹp:**

- Công tắc kẹp hoạt động theo sườn thay đổi, nên công tắc gạt sau cùng sẽ thắng.
- Trạng thái kẹp chỉ cập nhật khi đang kích hoạt.
- Công tắc gạt trong lúc `SAFETY_MODE` không được tính là sườn khi kích hoạt lại.

**Ở `SAFETY_MODE`:**

- `vx`, `vy`, `vr` = 0.
- Ba servo kẹp về vị trí `*_servo_initial_ms`.
- Cơ cấu nâng nhận 0 (dừng hẳn để motor không nóng), hoặc -100 nếu đang trong `naca_control_lower_lift()`.
- Cuốn bóng và trình tự bắn chỉ chạy nếu đang được giữ sau bài tự động nút xanh.

**Phanh khi nhả cần:**

- Khi cả `vx`, `vy`, `vr` về 0 mà ngay trước đó `vx` hoặc `vr` khác 0, `brake_omni()` được gọi để short-brake 4 bánh trong `VX_BRAKE_MS` / `VR_BRAKE_MS`. Nếu cả hai cùng khác 0 thì lấy thời gian dài hơn.
- Đẩy lại cần bất kỳ thì phanh ngắt ngay.
- Nhả riêng cần đi ngang (`vy`) thì không phanh.

## Hành vi cần biết trước khi sửa code

- **Timeout mất sóng:** quá `RC_TIMEOUT_MS` (300 ms) không giải mã được frame thì `naca_manual_handle_loop()` gọi `naca_control_stop_all()` một lần (dừng motor, cơ cấu nâng giữ ở `LIFT_HOLD_SPEED`). Trước khi nhận frame đầu tiên thì không có lệnh nào được gửi.
- **Treo khi khởi động:** `naca_motor_init()` lặp vô hạn nếu `MotorDriverV2::begin()` không qua được bước kiểm tra WHO_AM_I, nên robot sẽ không phản hồi RC nếu thiếu board driver.
- **Dải tốc độ:** `mapRC` map cần trên dải 988–2012 và có thể trả về tới ±200 (`SPEED_MODE_1` + 50); `omni_kinematics` chuẩn hóa cả 4 bánh về ±100 (giới hạn của `DCMotor::run`), nên lệnh lớn bị giảm tỉ lệ đồng đều.
- **Giá trị `*_servo_*_ms` không phải mili giây:** các define này trong `naca_motor.h` là tốc độ truyền vào `ServoMotor::spin()`. `spin()` đổi chúng thành góc theo công thức `90 - speed/100*90`, nên trên các servo 180° này chúng đóng vai trò điểm đặt vị trí.
- **Debug tạm khi khởi động:** `naca_motor_dump_registers()` chạy trong lúc init và có TODO đánh dấu cần xóa.
- **API của Arduino core 2.x:** code dùng `ledcSetup`, `ledcAttachPin` và `timerBegin(0, 80, true)`, những API đã bị loại bỏ trong Arduino-ESP32 3.x. Hiện code build với core 2.0.17 (`espressif32` 6.12.0), nhưng `platformio.ini` không cố định phiên bản platform.

## File lỗi thời hoặc không dùng

- **`include/AppConfig.h`** mô tả bản demo PS4 gốc của OhStem. Firmware hiện tại có include file này nhưng không dùng hằng số nào trong đó, nên các feature flag, cờ đảo chiều motor và thông số hình học ở đây không có tác dụng. Hãy sửa giá trị trong `src/naca_motor` và `src/nacateam/nacateam_crsf_define.h`.
- **`README.md`** mô tả firmware Nacateam, có sơ đồ kiến trúc Mermaid chuyển từ archify. Khi đổi kênh RC, chân hoặc thông số hay chỉnh thì cập nhật cả README.
- **`.archify/` và `floorplan/`** là sơ đồ kiến trúc HTML sinh ngày 2026-11-25. Đây là ảnh chụp tại thời điểm đó và không tự cập nhật. Sơ đồ archify ghi shooter/blower là "GPIO 6 / 7", trong khi code là shooter GPIO 7, blower GPIO 6.
- **`src/main.cpptxt`** là bản demo PS4 đó, còn **`src/test_crsf.cppok`** là chương trình độc lập để in các kênh CRSF. Đuôi file khiến PlatformIO không biên dịch chúng. Muốn dùng file nào, hãy đổi sang đuôi `.cpp` và đưa `src/main.cpp` ra khỏi bản build, vì mỗi file đều định nghĩa `setup()` và `loop()`.
- **`lib/OhStemRobotics/src/`** là phần code thư viện được biên dịch. `lib/OhStemRobotics/DCMotor.{cpp,h}` ở thư mục gốc của thư viện là bản cũ và không được build.
- **`Arduino IDE Library/`** là bản sao riêng cho Arduino IDE. Bản này đã khác với `lib/` và không thuộc bản build này.
- **Code chưa hiện thực:** Các hàm `naca_control_process_*` và `naca_control_send_telemetry_battery` được khai báo nhưng chưa định nghĩa, và các extern `*Mapped` trong `nacateam_crsf_define.h` cũng chưa có định nghĩa.

## Quy ước

- Include của project tính từ `src/`, ví dụ `"naca_control/naca_control.h"`. PlatformIO tự động biên dịch các file `.cpp` mới trong `src/`.
- Các module dùng tiền tố `naca_`, với hàm tự do kiểu C và biến global `extern`, kèm ghi chú nơi định nghĩa trong file `.cpp`. Các giá trị cần tinh chỉnh là `#define` trong header của module.
- Comment trong code dùng lẫn tiếng Việt và tiếng Anh.

ehavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## Rules for Claude Code

### 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:

- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

### 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

### 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:

- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:

- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

### 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:

- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:

```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

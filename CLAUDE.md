# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Tổng quan

Firmware cho robot thi đấu của Nacateam, gồm hệ di chuyển 4 bánh mecanum, cơ cấu nâng, cơ cấu cuốn bóng, shooter, blower và ba servo kẹp. Firmware chạy trên OhStem Yolo Uno (ESP32-S3, Arduino framework) và được điều khiển từ xa qua sóng RC giao thức CRSF. Hai nút D7/D8 chạy hai bài tự động (bắn bóng, gắp khối).

Thư mục này không phải git repository, nên không có lịch sử để so sánh hoặc khôi phục.

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

`src/naca_imu` đọc LSM6DSR qua SPI 3 dây (CS 1, SDA 3, SCLK 2), tích phân yaw theo tick 1 ms của hardware timer 1 và có PID giữ hướng. `naca_imu_init()` hiệu chuẩn gyro khoảng 7 giây lúc khởi động (robot phải đứng yên) và trả về false nếu không thấy chip; khi đó `naca_imu_ready` là false, mode không đầu và chế độ tự động bị tắt.

`src/naca_odom` fuse 2 encoder bánh sau (E1 = RL, E2 = RR) với yaw IMU thành `naca_odom_pose` (x, y theo mm, in trong dòng log mỗi giây). Các hằng số encoder, dấu và hệ số xoay trong `naca_odom.h` chưa được hiệu chỉnh trên robot.

`src/naca_led` báo trạng thái máy bằng LED RGB WS2812B (GPIO 45) và LED D13 (GPIO 48): đỏ khi khởi động, xanh dương khi chưa có hoặc mất sóng, vàng ở `SAFETY_MODE`, xanh lá khi kích hoạt, xanh ngọc khi bật mode không đầu, trắng/tím khi chạy bài tự động nút xanh/đỏ, nháy đỏ xen kẽ khi pin dưới `NACA_LED_BATTERY_LOW_V`. LED D13 nháy đều khi điều khiển tay và sáng liên tục khi chạy tự động.

`src/naca_auto` là bản port hai bài tự động từ chương trình OhStem Blockly (XanhNgoc2): nút D7 chạy khi nhấn (bắn bóng), nút D8 chạy khi nhả (gắp khối). Bài tự động chạy kiểu chặn và hủy khi nhấn lại nút, khi ch5 chuyển về `SAFETY_MODE`, hoặc khi một bước di chuyển quá `AUTO_STEP_TIMEOUT_MS`. Mọi thông số nằm trong `naca_auto.h`. Như chương trình gốc (gán `auto = 0` trước "Bắn bóng" và "hạ tay gắp"), bài nút xanh kết thúc ngay sau phần di chuyển rồi gọi `naca_control_keep_shooting()` và `naca_control_lower_lift()`: điều khiển tay hạ cơ cấu nâng trong `AUTO_LIFT_DOWN_MS`, đồng thời shooter, blower, servo xoay bóng (lặp quay tới/lui) và cuốn bóng tiếp tục chạy như chương trình gốc, cho đến khi gạt công tắc tương ứng (đổi vị trí so với lúc bài tự động kết thúc), về `SAFETY_MODE` hoặc mất sóng.

Lệnh motor chỉ được gửi qua I2C khi giải mã xong một frame CRSF, nên lưu lượng I2C này đi theo tốc độ frame RC chứ không theo tốc độ của `loop()`.

### Sơ đồ phần cứng

| Chức năng           | Kết nối                                                                                                           |
| ------------------- | ----------------------------------------------------------------------------------------------------------------- |
| Bộ thu CRSF         | `Serial2`, 420000 baud, RX 8 (D5), TX 9 (D6)                                                                              |
| Motor driver        | `MotorDriverV2`, I2C `0x54`, SDA 11, SCL 12, 400 kHz                                                              |
| Bánh mecanum        | FL `M1`, FR `M4` (đảo chiều), RL `E1` (đảo chiều), RR `E2` (đảo chiều)                                            |
| Nâng / cuốn bóng    | `M2` / `M3`                                                                                                       |
| Chân H-bridge local | M3 ở GPIO 5, 4; M4 ở GPIO 14, 13                                                                                  |
| Shooter / blower    | GPIO 6 / 7, kênh LEDC 2 / 3, 1 kHz, 8-bit (comment "20kHz" trong code đã lỗi thời)                                |
| Servo               | kẹp giữa `S1`, kẹp phải `S2`, kẹp trái `S3`, xoay bóng `S4` (servo 360°, lặp quay tới/lui khi bắn) |

`MotorDriverV2::setMotors()` gửi lệnh `M3` và `M4` qua I2C, đồng thời điều khiển luôn các chân GPIO local của chúng.

### Kênh RC

Mỗi kênh là một define `*_RC_CHANNEL` trong `naca_control.h`, đánh số từ 1 giống mix monitor của tay điều khiển (CH1, CH2, ...), và được đọc bằng `_raw_rc_values[X_RC_CHANNEL - 1]`. Bảng dưới dùng chỉ số mảng (từ 0), tức CH = chỉ số + 1. Muốn đổi kênh thì sửa define, không sửa chỉ số trong code.

| Chỉ số    | Chức năng                                                                                             |
| --------- | ----------------------------------------------------------------------------------------------------- |
| 0         | `TU_VUONG_MODE`: đi ngang (`vy`). `TUAT_VUONG_MODE`: xoay (`vr`)                                      |
| 1         | `TU_VUONG_MODE`: tiến/lùi (`vx`). `TUAT_VUONG_MODE`: nâng                                             |
| 2         | `TU_VUONG_MODE`: nâng. `TUAT_VUONG_MODE`: tiến/lùi (`vx`)                                             |
| 3         | `TU_VUONG_MODE`: xoay (`vr`). `TUAT_VUONG_MODE`: đi ngang (`vy`)                                      |
| 4 / 7 / 9 | kẹp trái / phải / giữa                                                                                |
| 5         | chọn kiểu cần và kích hoạt: `>1700` `TUAT_VUONG_MODE`, `>1400` `TU_VUONG_MODE`, còn lại `SAFETY_MODE` |
| 6         | tốc độ: `>1700` `SPEED_MODE_1` (150), còn lại `SPEED_MODE_0` (50)                                     |
| 8         | cuốn bóng                                                                                             |
| 10        | chuyển kẹp phải và kẹp giữa qua lại giữa special-close và close, nếu cả hai đều không mở              |
| 11        | cả kẹp trái và kẹp phải                                                                               |
| 12        | mode không đầu (`>1700`), lấy hướng robot lúc bật công tắc làm hướng tiến                             |
| 14 / 15   | shooter / blower                                                                                      |

Cơ cấu nâng có ba vị trí cần: `<1100` chạy xuống ở -100, `>1900` chạy lên ở 100, còn ở giữa thì giữ ở 10.

Công tắc kẹp hoạt động theo sườn thay đổi, nên công tắc gạt sau cùng sẽ thắng. Trạng thái kẹp chỉ cập nhật khi đang kích hoạt; ở `SAFETY_MODE`, kẹp giữ nguyên vị trí cuối, các motor nhận lệnh dừng, riêng cơ cấu nâng giữ ở `LIFT_HOLD_SPEED` (10) để không rơi. Công tắc gạt trong lúc `SAFETY_MODE` không được tính là sườn khi kích hoạt lại.

## Hành vi cần biết trước khi sửa code

- **Timeout mất sóng:** quá `RC_TIMEOUT_MS` (300 ms) không giải mã được frame thì `naca_manual_handle_loop()` gọi `naca_control_stop_all()` một lần (dừng motor, cơ cấu nâng giữ ở `LIFT_HOLD_SPEED`). Trước khi nhận frame đầu tiên thì không có lệnh nào được gửi.
- **Treo khi khởi động:** `naca_motor_init()` lặp vô hạn nếu `MotorDriverV2::begin()` không qua được bước kiểm tra WHO_AM_I, nên robot sẽ không phản hồi RC nếu thiếu board driver.
- **Dải tốc độ:** `mapRC` map cần trên dải 988–2012 và có thể trả về tới ±200 (`SPEED_MODE_1` + 50); `omni_kinematics` chuẩn hóa cả 4 bánh về ±100 (giới hạn của `DCMotor::run`), nên lệnh lớn bị giảm tỉ lệ đồng đều.
- **Giá trị `*_servo_*_ms` không phải mili giây:** các define này trong `naca_motor.h` là tốc độ truyền vào `ServoMotor::spin()`. `spin()` đổi chúng thành góc theo công thức `90 - speed/100*90`, nên trên các servo 180° này chúng đóng vai trò điểm đặt vị trí.
- **Debug tạm khi khởi động:** `naca_motor_dump_registers()` chạy trong lúc init và có TODO đánh dấu cần xóa.
- **API của Arduino core 2.x:** code dùng `ledcSetup`, `ledcAttachPin` và `timerBegin(0, 80, true)`, những API đã bị loại bỏ trong Arduino-ESP32 3.x. Hiện code build với core 2.0.17 (`espressif32` 6.12.0), nhưng `platformio.ini` không cố định phiên bản platform.

## File lỗi thời hoặc không dùng

- **`README.md` và `include/AppConfig.h`** mô tả bản demo PS4 gốc của OhStem. Firmware hiện tại có include `AppConfig.h` nhưng không dùng hằng số nào trong đó, nên các feature flag, cờ đảo chiều motor và thông số hình học ở đây không có tác dụng. Hãy sửa giá trị trong `src/naca_motor` và `src/nacateam/nacateam_crsf_define.h`.
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

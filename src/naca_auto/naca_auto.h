#ifndef NACA_AUTO_H
#define NACA_AUTO_H

#include <Arduino.h>

// Chế độ tự động, port từ chương trình OhStem XanhNgoc2:
// nút D7 (xanh, chạy khi nhấn) tự động bắn bóng, nút D8 (đỏ, chạy khi nhả) tự động gắp khối.
// Dừng giữa chừng: nhấn lại 1 trong 2 nút, hoặc gạt ch5 về SAFETY_MODE.

// #define AUTO_BTN_GREEN_PIN 10 // D7
// #define AUTO_BTN_RED_PIN 17   // D8

#define AUTO_BTN_GREEN_PIN 0 // D7 // thử
#define AUTO_BTN_RED_PIN 17  // D8
#define AUTO_BTN_DEBOUNCE_MS 30

// Dấu lệnh của drive_omni, đổi nếu robot chạy ngược hướng mong muốn
#define AUTO_FORWARD_SIGN -1 // vx của drive_omni để đi tiến (teleop: đẩy cần lên cho vx âm)
#define AUTO_RIGHT_SIGN 1    // vy của drive_omni để đi sang phải
#define AUTO_YAW_PID_SIGN 1  // đổi thành -1 nếu giữ hướng làm robot xoay càng lúc càng lệch
#define AUTO_VR_MAX 40       // giới hạn lệnh xoay khi giữ hướng

#define AUTO_CONTROL_PERIOD_MS 10
#define AUTO_STEP_TIMEOUT_MS 6000 // 1 bước di chuyển quá lâu (encoder không đếm, kẹt) thì hủy
#define AUTO_TURN_TIMEOUT_MS 2000
#define AUTO_HEADING_TOL_DEG 2.0f
#define AUTO_SOFT_START_MS 300 // auto_move, naca_auto_move: thời gian tăng tốc từ AUTO_MOVE_MIN_SPEED lên tốc độ đặt

// naca_auto_move (đo bằng odometry): hạ tốc khi gần đích, lố thì tự chạy ngược lại
#define AUTO_MOVE_MIN_SPEED 40   // tốc độ thấp nhất khi hạ tốc và khi chỉnh lố (phải đủ để robot còn lăn)
#define AUTO_MOVE_TOL_MM 10.0f   // sai số chấp nhận: cách đích dưới mức này thì coi là tới

// Nút xanh: tự động bắn bóng
// STEP1
#define AUTO_GREEN_BACK_MM 730.0f // đi lùi đến nơi lấy bóng
#define AUTO_GREEN_BACK_SPEED 60  // tốc độ lùi
// STEP2
#define AUTO_GREEN_STEP_MM 80.0f // quãng tiến mỗi bước
#define AUTO_GREEN_STEP_SPEED 50
#define AUTO_GREEN_STEP_COUNT 9
#define AUTO_GREEN_SCROLL_SPEED 100 // tốc độ cuốn bóng
// STEP3
#define AUTO_GREEN_FORWARD_MM 60.0f // đi tiến đến nơi bắn bóng
#define AUTO_GREEN_FORWARD_SPEED 60  // tốc độ tiến
// STEP4
#define AUTO_GREEN_LEFT_MM 200.0f // đi ngang sang trái để bắn bóng
#define AUTO_GREEN_LEFT_SPEED 40  // tốc độ sang trái
// STEP5
#define AUTO_SHOOTER_SPEED 100 // chân D3
#define AUTO_BLOWER_SPEED 100 // chân D4
// STEP6
#define AUTO_SHOOT_STAGGER_MS 100 // bật lần lượt shooter -> blower -> servo xoay bóng, cách nhau bấy nhiêu ms
#define AUTO_FEED_FORWARD_MS 1700 // servo xoay bóng quay đẩy bóng
#define AUTO_FEED_REVERSE_MS 300  // rồi quay ngược lại
#define AUTO_SHOT_SPEED_FORWARD 20          // tốc độ cuốn bóng khi bắn
#define AUTO_SHOT_SPEED_REVERSE -20          // tốc độ cuốn bóng khi bắn

// Nút đỏ: tự động gắp khối
#define AUTO_RED_LEFT_MM 740.0f // 5000 tick encoder E2 trong chương trình gốc
#define AUTO_RED_LEFT_SPEED 80
#define AUTO_RED_FORWARD_MM 250.0f
#define AUTO_RED_FORWARD_SPEED 80

// Hạ tay gắp
#define AUTO_LOWER_LEFT_SERVO 45
#define AUTO_LOWER_RIGHT_SERVO -47
#define AUTO_LOWER_MID_SERVO -10
#define AUTO_LIFT_DOWN_MS 1500 // thời gian chạy cơ cấu nâng xuống

void naca_auto_init();
bool naca_auto_handle_loop(); // true nếu vừa chạy xong 1 bài tự động

#endif // NACA_AUTO_H

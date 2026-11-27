#ifndef NACA_CONTROL_H
#define NACA_CONTROL_H

#include <Arduino.h>
#include "stdint.h"
#include "naca_motor/naca_motor.h"
#include "naca_timer/naca_timer.h"
#include "nacateam/nacateam_crsf_define.h"
#include "naca_imu/naca_imu.h"
#include "naca_odom/naca_odom.h"
#include "naca_auto/naca_auto.h"
#include "naca_led/naca_led.h"

#define RC_TIMEOUT_MS 300      // Quá thời gian này không có frame CRSF thì coi là mất sóng
// Số kênh RC theo mix monitor của tay điều khiển (CH1, CH2, ...), đếm từ 1.
// Dùng: _raw_rc_values[X_RC_CHANNEL - 1] vì mảng _raw_rc_values đếm từ 0.
#define ROLL_RC_CHANNEL             1            // cần phải, ngang
#define PITCH_RC_CHANNEL            2           // cần phải, dọc
#define THROTTLE_RC_CHANNEL         3        // cần trái, dọc
#define YAW_RC_CHANNEL              4             // cần trái, ngang
#define PILOT_MODE_RC_CHANNEL       5      // >1700 TUAT_VUONG_MODE, >1400 TU_VUONG_MODE, còn lại SAFETY_MODE
#define LEFT_GRIPPER_RC_CHANNEL     8    // kẹp trái
#define RIGHT_GRIPPER_RC_CHANNEL    9   // kẹp phải
#define MID_GRIPPER_RC_CHANNEL      7    // kẹp giữa
#define SPECIAL_GRIPPER_RC_CHANNEL  13 // đổi kẹp phải + giữa giữa special-close và close
#define BOTH_GRIPPER_RC_CHANNEL     6   // cả kẹp trái và kẹp phải
#define HEADLESS_RC_CHANNEL         11       // mode không đầu (>1700)
#define SPEED_MODE_RC_CHANNEL       14      // >1700 SPEED_MODE_1, còn lại SPEED_MODE_0
#define SCROLL_RC_CHANNEL           15          // cuốn bóng
#define BLOWER_SHOOTER_RC_CHANNEL           16         // // shooter blower
#define SINGLE_SERVO_PICKUP_BALL_RC_CHANNEL 10 // kẹp bóng đơn 1 bên trái hoặc phải, tùy chế độ gạt công tắc
#define HEADLESS_YAW_SIGN -1    // Đổi thành -1 nếu mode không đầu bù ngược chiều xoay

extern uint16_t _raw_rc_count;
extern bool naca_imu_ready; // Định nghĩa trong naca_control.cpp
void naca_control_init();
bool naca_control_read_rc();
void naca_control_stop_all();
void naca_control_set_gripper_state(int left, int right, int mid);
void naca_control_keep_shooting();
void naca_control_lower_lift(unsigned long ms);
void naca_control_process_rc_input(int16_t *rc_values, uint16_t rc_count);
void naca_control_process_telemetry();
void naca_control_send_telemetry_battery(uint16_t voltage, uint16_t current, int fuel, uint8_t remaining);
void naca_manual_handle_loop();

#endif // NACA_CONTROL_H
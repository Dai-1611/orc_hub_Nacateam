#ifndef NACA_MOTOR_H
#define NACA_MOTOR_H

#include <Arduino.h>
#include <OhStemRobotics.h>
#include "AppConfig.h"


#define left_servo_initial_ms -80
#define left_servo_open_ms 100
#define left_servo_close_ms 2 //Kẹp servo trái, càng nhỏ kẹp càng chặt 
#define left_servo_middle_close_ms 8 //Kẹp servo trái khi servo giữa ở vị trí mở, càng nhỏ kẹp càng chặt 
#define left_servo_dual_pickup_ball_close_ms -40


#define right_servo_initial_ms 80
#define right_servo_open_ms -100
#define right_servo_close_ms -5 //Kẹp servo phải, càng lớn kẹp càng chặt
#define right_servo_middle_close_ms -18 //Kẹp servo phải khi servo giữa ở vị trí mở, càng lớn kẹp càng chặt 
#define right_servo_dual_pickup_ball_close_ms 50 //kẹp 1 lần 2 banh

#define mid_servo_initial_ms -90
#define mid_servo_close_ms -90
#define mid_servo_open_ms 10


//Kẹp bóng đơn 1 bên trái
#define single_right_servo_pickup_ball_open_ms right_servo_close_ms
#define single_left_servo_pickup_ball_close_ms -25
#define single_mid_left_servo_pickup_ball_close_ms (mid_servo_open_ms + 45)


//kẹp bóng đơn 1 bên phải
#define single_left_servo_pickup_ball_open_ms left_servo_close_ms
#define single_right_servo_pickup_ball_close_ms 25
#define single_mid_right_servo_pickup_ball_close_ms (mid_servo_open_ms - 45)


#define VX_BRAKE_MS 300 // Thời gian phanh (ms) khi nhả cần Vx về 0
#define VR_BRAKE_MS 300 // Thời gian phanh (ms) khi nhả cần Vr (xoay) về 0
#define LIFT_HOLD_SPEED 20 // Lực giữ cơ cấu nâng khi cần ở giữa, SAFETY_MODE và mất sóng
using namespace ohstem::robotics;

// Mỗi lệnh motor/servo là 1 lần ghi I2C (~200 us). Loop gửi lại tất cả mỗi frame RC (~330 Hz) làm CPU bận ~70%.
// Các lớp dưới chỉ gửi khi lệnh đổi, hoặc đã quá NACA_MOTOR_REFRESH_MS kể từ lần gửi trước
// (gửi lại định kỳ phòng khi driver reset / mất 1 lần ghi I2C). Cách gọi run/brake/spin giữ nguyên.
#define NACA_MOTOR_REFRESH_MS 100

struct NacaCmdCache
{
    static constexpr int kNone = INT32_MIN;      // chưa gửi lần nào -> luôn gửi
    static constexpr int kBrake = INT32_MIN + 1; // lệnh phanh
    static constexpr int kStop = INT32_MIN + 2;  // lệnh thả (stop)
    int last = kNone;
    uint32_t sent_ms = 0;
    // true nếu cần gửi lệnh cmd (và ghi nhận là đã gửi)
    bool need_send(int cmd)
    {
        uint32_t now = millis();
        if (cmd == last && now - sent_ms < NACA_MOTOR_REFRESH_MS)
        {
            return false;
        }
        last = cmd;
        sent_ms = now;
        return true;
    }
    void invalidate() { last = kNone; }
};

class CachedDCMotor : public DCMotor
{
public:
    using DCMotor::DCMotor;
    void run(float speed) override;
    void brake() override;
    void stop() override;
    void invalidate() { cache_.invalidate(); } // buộc lần gọi kế tiếp gửi I2C

private:
    NacaCmdCache cache_;
};

class CachedServo : public ServoMotor
{
public:
    using ServoMotor::ServoMotor;
    // Che (không override, vì ServoMotor không có virtual) spin/setAngle của lớp gốc
    void spin(int speed);
    void setAngle(int angle);
    void invalidate() { cache_.invalidate(); }

private:
    NacaCmdCache cache_;
};

extern MotorDriverV2 mdV2;              // Định nghĩa trong naca_motor.cpp
extern CachedDCMotor motor_fl;          // Định nghĩa trong naca_motor.cpp
extern CachedDCMotor motor_fr;          // Định nghĩa trong naca_motor.cpp
extern CachedDCMotor motor_rr;          // Định nghĩa trong naca_motor.cpp
extern CachedDCMotor motor_rl;          // Định nghĩa trong naca_motor.cpp
extern CachedDCMotor motor_cuon_banh;   // Định nghĩa trong naca_motor.cpp
extern CachedDCMotor motor_nang;        // Định nghĩa trong naca_motor.cpp
extern CachedServo servo_xoay_bong;     // Định nghĩa trong naca_motor.cpp
extern CachedServo servo_left_gripper;  // Định nghĩa trong naca_motor.cpp
extern CachedServo servo_right_gripper; // Định nghĩa trong naca_motor.cpp
extern CachedServo servo_mid_gripper;   // Định nghĩa trong naca_motor.cpp

void naca_motor_init(); // Gọi 1 lần trong setup()
int mapRC(uint16_t raw_value, int speed);
void omni_kinematics(int vx, int vy, int vr, int *FL, int *FR, int *RR, int *RL);
void scroll_motor(int speed);
void lift_motor(int speed);
void shooter_motor(int speed);
void blower_motor(int speed);
void drive_omni(int vx, int vy, int vr);
void brake_omni(); // Phanh 4 bánh omni
void naca_motor_read_encoders(int32_t *rl_ticks, int32_t *rr_ticks);
float read_battery_voltage();
void naca_motor_dump_registers(); // Debug thanh ghi pin
#endif                            // NACA_MOTOR_H
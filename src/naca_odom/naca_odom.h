#ifndef NACA_ODOM_H
#define NACA_ODOM_H

#include <Arduino.h>

// Odometry: fuse 2 encoder bánh sau (E1 = RL, E2 = RR) với yaw của IMU.
// Hệ tọa độ: gốc và hướng lấy tại lúc naca_odom_reset(), X = phía trước, Y = bên phải.

// Thông số encoder và bánh lấy từ chương trình OhStem (ppr 11, tỉ số truyền 46.8, bánh 97 mm)
#define NACA_ODOM_TICKS_PER_REV (11 * 4 * 46.8f)
#define NACA_ODOM_WHEEL_DIAMETER_MM 97.0f
#define NACA_ODOM_MM_PER_TICK (PI * NACA_ODOM_WHEEL_DIAMETER_MM / NACA_ODOM_TICKS_PER_REV)
// Đổi thành -1 nếu đẩy robot tiến mà tick của bánh đó giảm
#define NACA_ODOM_RL_SIGN -1
#define NACA_ODOM_RR_SIGN -1
// Dấu của quãng đi ngang (RL - RR)/2. drive_omni dùng vx âm là tiến (AUTO_FORWARD_SIGN -1), nên khi đã chỉnh
// RL/RR_SIGN cho tiến = tick tăng thì (RL - RR)/2 dương lúc robot sang TRÁI -> cần -1 để Y dương là bên phải.
// Kiểm tra: đẩy robot sang phải bằng tay, odom y phải tăng.
#define NACA_ODOM_STRAFE_SIGN -1
// Quãng đường (mm) bánh sau lăn khi robot xoay tại chỗ 1 độ, dùng để tách xoay khỏi đi ngang.
// Bánh mecanum: = (Lx + Ly) * PI / 180, Lx = nửa khoảng cách trục trước-sau, Ly = nửa khoảng cách bánh trái-phải.
// Trục trước-sau cách nhau 350 mm, bánh trái-phải cách nhau 425 mm -> Lx = 175, Ly = 212.5 -> 6.76.
// Dấu chưa kiểm: xoay robot tại chỗ, nếu x, y trôi nhiều thì đổi dấu.
#define NACA_ODOM_ROT_MM_PER_DEG 6.76f
// Đổi thành -1 nếu yaw của IMU tăng khi robot quay phải
#define NACA_ODOM_YAW_SIGN 1
#define NACA_ODOM_PERIOD_MS 20 // Chu kỳ đọc encoder qua I2C

struct NacaOdomPose
{
    int32_t left_ticks, right_ticks; // tick encoder bánh sau (RL, RR) tại lúc naca_odom_reset()
    float x_mm, y_mm, yaw_deg;
};
extern NacaOdomPose naca_odom_pose; // Định nghĩa trong naca_odom.cpp

void naca_odom_reset();  // Đặt vị trí hiện tại làm gốc (yaw vẫn theo IMU)
void naca_odom_update(); // Gọi liên tục trong loop

#endif // NACA_ODOM_H

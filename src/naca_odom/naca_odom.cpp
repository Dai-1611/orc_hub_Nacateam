#include "naca_odom.h"
#include "naca_motor/naca_motor.h"
#include "naca_imu/naca_imu.h"

NacaOdomPose naca_odom_pose = {};

static int32_t prev_rl_ticks = 0;
static int32_t prev_rr_ticks = 0;
static float prev_yaw_deg = 0.0f;
static unsigned long last_update_ms = 0;

void naca_odom_reset()
{
    naca_motor_read_encoders(&prev_rl_ticks, &prev_rr_ticks);
    prev_yaw_deg = naca_imu_get_snapshot().yaw;
    naca_odom_pose.left_ticks = 0;
    naca_odom_pose.right_ticks = 0;
    naca_odom_pose.x_mm = 0.0f;
    naca_odom_pose.y_mm = 0.0f;
    naca_odom_pose.yaw_deg = prev_yaw_deg;
}

void naca_odom_update()
{
    if (millis() - last_update_ms < NACA_ODOM_PERIOD_MS)
    {
        return;
    }
    last_update_ms = millis();

    int32_t rl_ticks, rr_ticks;
    naca_motor_read_encoders(&rl_ticks, &rr_ticks);
    naca_odom_pose.left_ticks = NACA_ODOM_RL_SIGN * rl_ticks;
    naca_odom_pose.right_ticks = NACA_ODOM_RR_SIGN * rr_ticks;
    float yaw_deg = naca_imu_get_snapshot().yaw;

    float d_rl = NACA_ODOM_RL_SIGN * (rl_ticks - prev_rl_ticks) * NACA_ODOM_MM_PER_TICK;
    float d_rr = NACA_ODOM_RR_SIGN * (rr_ticks - prev_rr_ticks) * NACA_ODOM_MM_PER_TICK;
    float d_yaw = yaw_deg - prev_yaw_deg;
    if (d_yaw > 180.0f)
        d_yaw -= 360.0f;
    if (d_yaw < -180.0f)
        d_yaw += 360.0f;

    // Theo omni_kinematics (động học mecanum): RL = vx + vy - vr, RR = vx - vy + vr
    // -> tổng 2 bánh cho quãng tiến; hiệu 2 bánh cho quãng đi ngang lẫn với xoay, phần xoay lấy từ IMU.
    float d_forward = (d_rl + d_rr) / 2.0f;
    float d_right = NACA_ODOM_STRAFE_SIGN * (d_rl - d_rr) / 2.0f + NACA_ODOM_ROT_MM_PER_DEG * d_yaw;

    // Đổi sang hệ tọa độ sân bằng yaw ở giữa bước
    float yaw_rad = NACA_ODOM_YAW_SIGN * (prev_yaw_deg + d_yaw / 2.0f) * DEG_TO_RAD;
    naca_odom_pose.x_mm += d_forward * cosf(yaw_rad) + d_right * sinf(yaw_rad);
    naca_odom_pose.y_mm += -d_forward * sinf(yaw_rad) + d_right * cosf(yaw_rad);
    naca_odom_pose.yaw_deg = yaw_deg;

    prev_rl_ticks = rl_ticks;
    prev_rr_ticks = rr_ticks;
    prev_yaw_deg = yaw_deg;
}

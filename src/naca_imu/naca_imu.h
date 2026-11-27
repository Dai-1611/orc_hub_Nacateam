#ifndef NACA_IMU_H
#define NACA_IMU_H
#include <Arduino.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"

// ─── Pin config ──────────────────────────────────────────────────────────────
#define IMU_CS_PIN 1
#define IMU_MOSI_PIN 3
//#define IMU_MISO_PIN 6
#define IMU_SCLK_PIN 2


// ─── SPI mode ────────────────────────────────────────────────────────────────
// Bỏ comment để dùng SPI 3 dây: SDA/SDI của IMU nối IMU_MOSI_PIN, không dùng IMU_MISO_PIN
#define LSM6DSR_3WIRE

// ─── LSM6DSR Register map ────────────────────────────────────────────────────
#define WHO_AM_I 0x0F
#define CTRL1_XL 0x10 // CTRL1_XL (10h)
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define OUTX_L_G 0x22
#define OUTX_L_XL 0x28

// LSM6DSR SPI protocol: bit7=1 → read, bit7=0 → write
#define SPI_READ_FLAG 0x80

// CTRL3_C bit3 SIM: 0 = SPI 4 dây, 1 = SPI 3 dây
#ifdef LSM6DSR_3WIRE
#define CTRL3_C_SPI_MODE 0x08
#else
#define CTRL3_C_SPI_MODE 0x00
#endif

// ─── Tuning constants ────────────────────────────────────────────────────────
// #define ACCEL_SENSITIVITY 0.000598f // g/LSB  (±2g, 16-bit)
// #define GYRO_SENSITIVITY 0.00875f   // °/s/LSB (250°/s, 16-bit)
#define ACCEL_SENSITIVITY 0.004786f // m/s²/LSB (±16g: 0.488 mg/LSB)
#define GYRO_SENSITIVITY 0.14f      // °/s/LSB (±4000°/s: 140 mdps/LSB)
//ACCEL_SENSITIVITY = (mg/LSB) / 1000 × 9.80665
//GYRO_SENSITIVITY = (mdps/LSB) / 1000
#define TIMER_HZ 1000
#define DT (1.0f / TIMER_HZ)
#define TILT_ACCEL_LPF_ALPHA 0.005f // lọc gia tốc, τ ≈ DT / alpha = 0.2s
#define TILT_MIN_COS2_PITCH 0.117f  // cos²(70°): nghiêng hơn thì bỏ bù nghiêng

// ─── Yaw mode ────────────────────────────────────────────────────────────────
enum YawMode : uint8_t
{
    YAW_MODE_FLAT = 0,     // tích phân gz (góc quay trong mặt phẳng robot)
    YAW_MODE_TILT_COMP = 1 // bù nghiêng theo roll/pitch (hướng thật so với mặt đất)
};

// ─── Shared state ────────────────────────────────────────────────────────────
struct IMUSnapshot
{
    float yaw, gz, ax, ay, az;
};
// Yaw PID variables
extern volatile float yaw_angle;
extern volatile float yaw_target;
extern volatile float pid_integral;
extern volatile float pid_prev_err;
extern volatile unsigned long last_imu_time;
extern volatile int vr_final;
// extern volatile bool yaw_hold_enabled;
//  PID gains - chỉnh sau khi test
extern volatile float KP;
extern volatile float KI;
extern volatile float KD;
extern volatile IMUSnapshot g_imu;
extern portMUX_TYPE g_imu_mux;
extern volatile float gy_offset;
extern volatile float gz_offset;
extern volatile float g_deadzone; // fallback, sẽ được ghi đè sau calibrate

// Timer ISR tick counter
extern volatile uint32_t imu_tick_pending;

// ─── Timer ISR ───────────────────────────────────────────────────────────────
extern hw_timer_t *imu_timer;
IMUSnapshot naca_imu_get_snapshot();
void naca_imu_reset_yaw();
bool naca_imu_init(); // false nếu không tìm thấy LSM6DSR
uint32_t naca_imu_update(int vr_cmd, bool yaw_hold_enabled, YawMode mode = YAW_MODE_FLAT);
bool naca_lsm6dsr_init();
void IRAM_ATTR naca_imu_on_timer();
uint8_t naca_lsm6dsr_read_reg(uint8_t reg);
void naca_lsm6dsr_write_reg(uint8_t reg, uint8_t val);
void naca_lsm6dsr_read_burst(uint8_t reg, uint8_t *buf, uint8_t len);
void naca_lsm6dsr_calibrate_gyro(int samples);
float naca_lsm6dsr_measure_noise(int samples);
int naca_imu_compute_yaw_pid(float target, float current);
int naca_imu_compute_yaw_pid();
#endif // NACA_IMU_H
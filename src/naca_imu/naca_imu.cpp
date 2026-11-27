#include "naca_imu.h"

#ifdef LSM6DSR_3WIRE
#include "driver/spi_master.h"
static spi_device_handle_t imu_spi = NULL;
#else
static SPISettings imu_spi_cfg(8000000, MSBFIRST, SPI_MODE3);
#endif
hw_timer_t *imu_timer = NULL;

// Shared IMU state (single instance)
volatile float yaw_angle = 0.0f;
volatile float yaw_target = 0.0f;
volatile float pid_integral = 0.0f;
volatile float pid_prev_err = 0.0f;
volatile unsigned long last_imu_time = 0;
volatile int vr_final = 0;
volatile float KP = 4.0f;
volatile float KI = 0.1f;
volatile float KD = 20.0f;

// volatile float KP = 3.0f;
// volatile float KI = 0.08f;
// volatile float KD = 0.001f;

volatile IMUSnapshot g_imu = {};
portMUX_TYPE g_imu_mux = portMUX_INITIALIZER_UNLOCKED;
volatile float gy_offset = 0.0f;
volatile float gz_offset = 0.0f;
volatile float g_deadzone = 0.2f; // fallback, will be overwritten after calibrate
volatile uint32_t imu_tick_pending = 0;

// Gia tốc đã lọc, dùng để tính roll/pitch khi bù nghiêng
static float ax_lpf = 0.0f;
static float ay_lpf = 0.0f;
static float az_lpf = 0.0f;
static bool accel_lpf_ready = false;

void IRAM_ATTR naca_imu_on_timer()
{
    portENTER_CRITICAL_ISR(&g_imu_mux);
    imu_tick_pending++;
    portEXIT_CRITICAL_ISR(&g_imu_mux);
}

// ─── Low-level SPI ───────────────────────────────────────────────────────────
#ifdef LSM6DSR_3WIRE

/**
 * SPI 3 dây: thư viện SPI của Arduino không hỗ trợ nên dùng spi_master của ESP-IDF.
 * Driver tự điều khiển CS để SCLK đã ở mức idle của mode 3 trước khi CS xuống thấp.
 */
static void imu_spi_bus_init()
{
    spi_bus_config_t bus = {};
    bus.mosi_io_num = IMU_MOSI_PIN;
    bus.miso_io_num = -1;
    bus.sclk_io_num = IMU_SCLK_PIN;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_DISABLED));

    spi_device_interface_config_t dev = {};
    dev.command_bits = 8; // byte địa chỉ register
    dev.mode = 3;
    dev.clock_speed_hz = 8000000;
    dev.spics_io_num = IMU_CS_PIN;
    dev.queue_size = 1;
    dev.flags = SPI_DEVICE_3WIRE | SPI_DEVICE_HALFDUPLEX;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &dev, &imu_spi));
}

void naca_lsm6dsr_read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    spi_transaction_t t = {};
    t.cmd = reg | SPI_READ_FLAG;
    t.rxlength = len * 8;
    t.rx_buffer = buf;
    spi_device_polling_transmit(imu_spi, &t);
}

uint8_t naca_lsm6dsr_read_reg(uint8_t reg)
{
    uint8_t val = 0;
    naca_lsm6dsr_read_burst(reg, &val, 1);
    return val;
}

void naca_lsm6dsr_write_reg(uint8_t reg, uint8_t val)
{
    spi_transaction_t t = {};
    t.cmd = reg & ~SPI_READ_FLAG; // đảm bảo bit7=0
    t.length = 8;
    t.tx_buffer = &val;
    spi_device_polling_transmit(imu_spi, &t);
}

#else

/**
 * Đọc 1 register.
 * LSM6DSR: gửi (reg | 0x80) → dummy byte → nhận kết quả.
 */
uint8_t naca_lsm6dsr_read_reg(uint8_t reg)
{
    SPI.beginTransaction(imu_spi_cfg);
    digitalWrite(IMU_CS_PIN, LOW);
    SPI.transfer(reg | SPI_READ_FLAG);
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(IMU_CS_PIN, HIGH);
    SPI.endTransaction();
    return val;
}

/**
 * Ghi 1 register.
 * LSM6DSR: bit7=0 (write) → tự động khi reg < 0x80.
 */
void naca_lsm6dsr_write_reg(uint8_t reg, uint8_t val)
{
    SPI.beginTransaction(imu_spi_cfg);
    digitalWrite(IMU_CS_PIN, LOW);
    SPI.transfer(reg & ~SPI_READ_FLAG); // đảm bảo bit7=0
    SPI.transfer(val);
    digitalWrite(IMU_CS_PIN, HIGH);
    SPI.endTransaction();
}

/**
 * Burst read: CS giữ thấp suốt quá trình, IF_INC=1 (đã set trong CTRL3_C)
 * tự động tăng địa chỉ register sau mỗi byte.
 */
void naca_lsm6dsr_read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    SPI.beginTransaction(imu_spi_cfg);
    digitalWrite(IMU_CS_PIN, LOW);
    SPI.transfer(reg | SPI_READ_FLAG);
    for (uint8_t i = 0; i < len; i++)
        buf[i] = SPI.transfer(0x00);
    digitalWrite(IMU_CS_PIN, HIGH);
    SPI.endTransaction();
}

#endif // LSM6DSR_3WIRE

/**
 * Tốc độ đổi hướng so với mặt đất: yaw_rate = (gy*sin(roll) + gz*cos(roll)) / cos(pitch).
 * Roll/pitch lấy từ gia tốc đã lọc nên không cần gọi hàm lượng giác.
 * Nhân dấu của az để chiều yaw giống YAW_MODE_FLAT, kể cả khi chip gắn úp.
 */
static float compute_tilt_yaw_rate(float gy_c, float gz_c)
{
    float h2 = ay_lpf * ay_lpf + az_lpf * az_lpf;
    float norm2 = ax_lpf * ax_lpf + h2;
    if (h2 <= TILT_MIN_COS2_PITCH * norm2)
        return gz_c;

    float sign = (az_lpf >= 0.0f) ? 1.0f : -1.0f;
    return sign * (gy_c * ay_lpf + gz_c * az_lpf) * sqrtf(norm2) / h2;
}

// ─── IMU Update (called from loop) ─────────────────────────────────────────-
uint32_t naca_imu_update(int vr_cmd, bool yaw_hold_enabled, YawMode mode)
{
    uint8_t buf[12];
    uint32_t ticks_to_process = 0;
    uint32_t ticks_processed = 0;

    portENTER_CRITICAL(&g_imu_mux);
    // Lấy hết tick đang chờ để không bị dồn khi loop chậm; nếu nhiều hơn 10 tick thì
    // vẫn chỉ đọc 10 mẫu nhưng mỗi mẫu tích phân với dt dài hơn, nên yaw không bị trễ.
    ticks_processed = imu_tick_pending;
    imu_tick_pending = 0;
    bool hold_enabled = yaw_hold_enabled;
    float target_yaw = yaw_target;
    // Vừa bật yaw hold: xóa trạng thái PID cũ để không giật
    static bool prev_hold_enabled = false;
    if (hold_enabled && !prev_hold_enabled)
    {
        pid_integral = 0.0f;
        pid_prev_err = 0.0f;
    }
    prev_hold_enabled = hold_enabled;
    portEXIT_CRITICAL(&g_imu_mux);

    if (ticks_processed == 0)
        return 0;
    ticks_to_process = ticks_processed > 10 ? 10 : ticks_processed;
    const float dt = DT * ticks_processed / ticks_to_process;

    while (ticks_to_process > 0)
    {
        ticks_to_process--;

        // Đọc 12 byte: OUTX_L_G(0x22) → OUTZ_H_XL(0x2D)
        // Gyro: buf[0..5], Accel: buf[6..11]
        naca_lsm6dsr_read_burst(OUTX_L_G, buf, 12);

        int16_t raw_gy = (int16_t)(buf[3] << 8 | buf[2]);
        int16_t raw_gz = (int16_t)(buf[5] << 8 | buf[4]);

        int16_t raw_ax = (int16_t)(buf[7] << 8 | buf[6]);
        int16_t raw_ay = (int16_t)(buf[9] << 8 | buf[8]);
        int16_t raw_az = (int16_t)(buf[11] << 8 | buf[10]);

        float gz = raw_gz * GYRO_SENSITIVITY;
        float gz_c = gz - gz_offset;
        float gy_c = raw_gy * GYRO_SENSITIVITY - gy_offset;

        // Lọc gia tốc ở cả 2 mode để khi chuyển mode có sẵn roll/pitch đúng
        float ax = raw_ax * ACCEL_SENSITIVITY;
        float ay = raw_ay * ACCEL_SENSITIVITY;
        float az = raw_az * ACCEL_SENSITIVITY;
        if (!accel_lpf_ready)
        {
            ax_lpf = ax;
            ay_lpf = ay;
            az_lpf = az;
            accel_lpf_ready = true;
        }
        ax_lpf += TILT_ACCEL_LPF_ALPHA * (ax - ax_lpf);
        ay_lpf += TILT_ACCEL_LPF_ALPHA * (ay - ay_lpf);
        az_lpf += TILT_ACCEL_LPF_ALPHA * (az - az_lpf);

        float yaw_rate = (mode == YAW_MODE_TILT_COMP) ? compute_tilt_yaw_rate(gy_c, gz_c) : gz_c;

        float new_yaw;
        portENTER_CRITICAL(&g_imu_mux);
        new_yaw = g_imu.yaw;
        portEXIT_CRITICAL(&g_imu_mux);

        // Tích phân chỉ khi vượt dead-zone
        if (fabsf(yaw_rate) > g_deadzone)
        {
            new_yaw += yaw_rate * dt;
            if (new_yaw > 180.0f)
                new_yaw -= 360.0f;
            if (new_yaw < -180.0f)
                new_yaw += 360.0f;
        }

        portENTER_CRITICAL(&g_imu_mux);
        g_imu.yaw = new_yaw;
        g_imu.gz = gz_c;
        g_imu.ax = ax;
        g_imu.ay = ay;
        g_imu.az = az;
        yaw_angle = new_yaw;
        portEXIT_CRITICAL(&g_imu_mux);

        int vr_out = hold_enabled ? naca_imu_compute_yaw_pid(target_yaw, new_yaw) : vr_cmd;

        portENTER_CRITICAL(&g_imu_mux);
        vr_final = vr_out;
        portEXIT_CRITICAL(&g_imu_mux);
    }

    return ticks_processed;
}

// ─── Public API ──────────────────────────────────────────────────────────────
IMUSnapshot naca_imu_get_snapshot()
{
    IMUSnapshot s;
    portENTER_CRITICAL(&g_imu_mux);
    s.yaw = g_imu.yaw;
    s.gz = g_imu.gz;
    s.ax = g_imu.ax;
    s.ay = g_imu.ay;
    s.az = g_imu.az;
    portEXIT_CRITICAL(&g_imu_mux);
    return s;
}

void naca_imu_reset_yaw()
{
    portENTER_CRITICAL(&g_imu_mux);
    g_imu.yaw = 0.0f;
    yaw_angle = 0.0f;
    yaw_target = 0.0f;
    pid_integral = 0.0f;
    pid_prev_err = 0.0f;
    portEXIT_CRITICAL(&g_imu_mux);
}

// ─── Init ────────────────────────────────────────────────────────────────────
bool naca_lsm6dsr_init()
{
    // Ghi được ở cả SPI 3 dây và 4 dây, nên reset trước khi đọc: chip vẫn giữ SIM
    // cũ nếu nạp lại firmware mà không tắt nguồn, và SW reset đưa SIM về 4 dây.
    naca_lsm6dsr_write_reg(CTRL3_C, 0x01); // SW reset
    delay(10);
    naca_lsm6dsr_write_reg(CTRL3_C, 0x04 | CTRL3_C_SPI_MODE); // chọn SPI 3/4 dây trước khi đọc

    uint8_t whoami = naca_lsm6dsr_read_reg(WHO_AM_I);
    if (whoami != 0x6B)
    {
        Serial.printf("LSM6DSR not found! WHO_AM_I=0x%02X\n", whoami);
        return false;
    }
    // Khởi tạo các thanh ghi của LSM6DSR
    // naca_lsm6dsr_write_reg(CTRL1_XL, 0x60); // 416Hz, ±2g
    naca_lsm6dsr_write_reg(CTRL1_XL, 0x84); // 1.66kHz, ±16g // 1 0 0 0 0 1 0 0
    // naca_lsm6dsr_write_reg(CTRL2_G, 0x60);                    // 416Hz, 250°/s
    naca_lsm6dsr_write_reg(CTRL2_G, 0x81);                    // 1.66kHz, 4000°/s, 4000dps  // 1 0 0 0 0 0 0 1
    naca_lsm6dsr_write_reg(CTRL3_C, 0x44 | CTRL3_C_SPI_MODE); // BDU=1, IF_INC=1 (quan trọng cho burst SPI)
    delay(10);
    Serial.println("LSM6DSR OK (SPI)");
    return true;
}
bool naca_imu_init()
{
    Serial.begin(115200);
    // Khởi SPI bus với pin custom
#ifdef LSM6DSR_3WIRE
    imu_spi_bus_init();
#else
    SPI.begin(IMU_SCLK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, IMU_CS_PIN);
    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWrite(IMU_CS_PIN, HIGH); // CS idle HIGH
#endif

    if (!naca_lsm6dsr_init())
    {
        return false;
    }

    naca_lsm6dsr_calibrate_gyro(2000);
    g_deadzone = naca_lsm6dsr_measure_noise(500);
    Serial.printf("Dead-zone set to: %.4f deg/s\n", g_deadzone);

    imu_timer = timerBegin(1, 80, true); // 1µs/tick. Timer 0 là của naca_timer
    timerAttachInterrupt(imu_timer, &naca_imu_on_timer, true);
    timerAlarmWrite(imu_timer, 1000, true); // 1ms
    timerAlarmEnable(imu_timer);
    return true;
}
// ─── Calibration ─────────────────────────────────────────────────────────────
void naca_lsm6dsr_calibrate_gyro(int samples)
{
    Serial.println("Warming up sensor...");
    delay(2000);

    Serial.println("Calibrating, keep still...");
    double sum = 0, sum_gy = 0;
    uint8_t buf[6];
    for (int i = 0; i < samples; i++)
    {
        naca_lsm6dsr_read_burst(OUTX_L_G, buf, 6);
        int16_t raw_gy = (int16_t)(buf[3] << 8 | buf[2]);
        int16_t raw_gz = (int16_t)(buf[5] << 8 | buf[4]);
        sum_gy += raw_gy * GYRO_SENSITIVITY;
        sum += raw_gz * GYRO_SENSITIVITY;
        delay(2);
    }
    gy_offset = (float)(sum_gy / samples);
    gz_offset = (float)(sum / samples);
    Serial.printf("gy_offset = %.5f, gz_offset = %.5f deg/s\n", gy_offset, gz_offset);

    if (fabsf(gz_offset) > 5.0f || fabsf(gy_offset) > 5.0f)
    {
        Serial.println("WARNING: offset too large, re-calibrating...");
        sum = 0;
        sum_gy = 0;
        for (int i = 0; i < samples; i++)
        {
            naca_lsm6dsr_read_burst(OUTX_L_G, buf, 6);
            int16_t raw_gy = (int16_t)(buf[3] << 8 | buf[2]);
            int16_t raw_gz = (int16_t)(buf[5] << 8 | buf[4]);
            sum_gy += raw_gy * GYRO_SENSITIVITY;
            sum += raw_gz * GYRO_SENSITIVITY;
            delay(2);
        }
        gy_offset = (float)(sum_gy / samples);
        gz_offset = (float)(sum / samples);
        Serial.printf("gy_offset retry = %.5f, gz_offset retry = %.5f deg/s\n", gy_offset, gz_offset);
    }
}

float naca_lsm6dsr_measure_noise(int samples)
{
    double sum = 0, sum_sq = 0;
    uint8_t buf[6];
    for (int i = 0; i < samples; i++)
    {
        naca_lsm6dsr_read_burst(OUTX_L_G, buf, 6);
        int16_t raw_gz = (int16_t)(buf[5] << 8 | buf[4]);
        float gz = raw_gz * GYRO_SENSITIVITY - gz_offset;
        sum += gz;
        sum_sq += gz * gz;
        delay(2);
    }
    float mean = sum / samples;
    float stddev = sqrtf(sum_sq / samples - mean * mean);
    Serial.printf("gz noise: mean=%.4f stddev=%.4f deg/s\n", mean, stddev);
    return stddev * 3.0f; // dead-zone = 3σ
}
int naca_imu_compute_yaw_pid(float target, float current)
{

    float err = target - current;
    while (err > 180.0f)
        err -= 360.0f;
    while (err < -180.0f)
        err += 360.0f;

    pid_integral += err;
    pid_integral = constrain(pid_integral, -50.0f, 50.0f);
    float pid_d = (err - pid_prev_err);
    pid_prev_err = err;

    float out = KP * err + KI * pid_integral + KD * pid_d;
    return (int)constrain(-out, -250.0f, 250.0f);
}

int naca_imu_compute_yaw_pid()
{
    return naca_imu_compute_yaw_pid(yaw_target, yaw_angle);
}
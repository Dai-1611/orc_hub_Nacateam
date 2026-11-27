#include "naca_motor.h"
constexpr int M3_IN1_PIN = 5;  // D2;
constexpr int M3_IN2_PIN = 4;  // A3;
constexpr int M4_IN1_PIN = 14; // A7;//Dung
constexpr int M4_IN2_PIN = 13; // A6 //Dung
MotorDriverV2 mdV2(
    Wire,
    MotorDriverV2::kDefaultAddress,
    M3_IN1_PIN,
    M3_IN2_PIN,
    M4_IN1_PIN,
    M4_IN2_PIN);
CachedDCMotor motor_fl(mdV2, M1);
CachedDCMotor motor_fr(mdV2, M4, true);
CachedDCMotor motor_rl(mdV2, E1, true);
CachedDCMotor motor_rr(mdV2, E2, true);
CachedDCMotor motor_cuon_banh(mdV2, M3);
CachedDCMotor motor_nang(mdV2, M2);
#define motor_shooter 7 // D3
#define ledc_channel_shooter 2
#define motor_blower 6 // D4
#define ledc_channel_blower 3
CachedServo servo_xoay_bong(mdV2, S4, 360);
CachedServo servo_left_gripper(mdV2, S3, 180);
CachedServo servo_right_gripper(mdV2, S2, 180);
CachedServo servo_mid_gripper(mdV2, S1, 180);

void CachedDCMotor::run(float speed)
{
    // Cùng cách làm tròn với DCMotor::run để 2 lệnh ra cùng giá trị thì coi là trùng
    int s = (int)constrain(speed, -100.0f, 100.0f);
    if (cache_.need_send(s))
    {
        DCMotor::run(s);
    }
}

void CachedDCMotor::brake()
{
    if (cache_.need_send(NacaCmdCache::kBrake))
    {
        DCMotor::brake();
    }
}

void CachedDCMotor::stop()
{
    if (cache_.need_send(NacaCmdCache::kStop))
    {
        DCMotor::stop();
    }
}

void CachedServo::spin(int speed)
{
    // Cùng công thức với ServoMotor::spin
    speed = constrain(speed, -200, 200);
    setAngle((int)(90.0f - (speed / 100.0f) * 90.0f));
}

void CachedServo::setAngle(int angle)
{
    if (cache_.need_send(angle))
    {
        ServoMotor::setAngle(angle);
    }
}
void naca_motor_init()
{
    Wire.begin();
    Wire.setClock(400000); // 400kHz
    pinMode(motor_shooter, OUTPUT);
    pinMode(motor_blower, OUTPUT);
    ledcSetup(ledc_channel_shooter, 1000, 8); // 20kHz, 8-bit resolution
    ledcSetup(ledc_channel_blower, 1000, 8);  // 20kHz, 8-bit resolution
    ledcAttachPin(motor_shooter, ledc_channel_shooter);
    ledcAttachPin(motor_blower, ledc_channel_blower);
    ledcWrite(ledc_channel_shooter, 0); // Set initial duty cycle to 0
    ledcWrite(ledc_channel_blower, 0);  // Set initial duty cycle to 0
    if (!mdV2.begin())
    {
        Serial.println("MotorDriverV2 not found");
        while (true)
        {
            delay(100);
        }
    }
    servo_xoay_bong.begin();
    servo_left_gripper.begin();
    servo_right_gripper.begin();
    servo_mid_gripper.begin();
    motor_rl.setEncoder(600, 11, 34);
    motor_rr.setEncoder(600, 11, 34);
}
int mapRC(uint16_t raw_value, int speed)
{
    const int DEAD_ZONE = 8;
    const int MIN_RC = 900;
    const int MAX_RC = 2100;

    // Check for valid RC signal (if out of range, return 0 - no signal)
    if (raw_value < MIN_RC || raw_value > MAX_RC)
    {
        return 0;
    }

    // Cần CRSF chỉ ra 988-2012, nên map trên dải này để đẩy hết cần đạt đủ speed
    const int STICK_MIN = 988;
    const int STICK_MAX = 2012;
    int mapped = map(constrain((int)raw_value, STICK_MIN, STICK_MAX), STICK_MIN, STICK_MAX, -speed, speed);

    // Apply dead zone
    if (abs(mapped) < DEAD_ZONE)
    {
        mapped = 0;
    }

    return mapped;
}
void omni_kinematics(int vx, int vy, int vr, int *FL, int *FR, int *RR, int *RL)
{
    // Calculate the speed for each wheel based on the desired velocities
    *FL = vx - vy - vr;
    *FR = vx + vy + vr;
    *RR = vx - vy + vr;
    *RL = vx + vy - vr;

    // Chuẩn hóa về ±100 (giới hạn của DCMotor::run) để giữ đúng tỉ lệ giữa các bánh
    int maxSpeed = max(max(abs(*FL), abs(*FR)), max(abs(*RR), abs(*RL)));
    if (maxSpeed > 100)
    {
        float scale = 100.0 / maxSpeed;
        *FL *= scale;
        *FR *= scale;
        *RR *= scale;
        *RL *= scale;
    }
}
void drive_omni(int vx, int vy, int vr)
{
    int FL_speed, FR_speed, RR_speed, RL_speed;
    omni_kinematics(vx, vy, vr, &FL_speed, &FR_speed, &RR_speed, &RL_speed);
    motor_fl.run(-FL_speed);
    motor_fr.run(FR_speed);
    motor_rr.run(RR_speed);
    motor_rl.run(RL_speed);

    // Set motor speeds
}
// Phanh chủ động (short-brake) cả 4 bánh, khác run(0) là thả trôi
void brake_omni()
{
    motor_fl.brake();
    motor_fr.brake();
    motor_rr.brake();
    motor_rl.brake();
}
// Đọc tick thô của 2 encoder bánh sau (E1 = RL, E2 = RR)
void naca_motor_read_encoders(int32_t *rl_ticks, int32_t *rr_ticks)
{
    *rl_ticks = motor_rl.encoderTicks();
    *rr_ticks = motor_rr.encoderTicks();
}
void scroll_motor(int speed)
{
    motor_cuon_banh.run(speed);
}
void lift_motor(int speed)
{
    motor_nang.run(speed);
}
void shooter_motor(int speed)
{
    // Set the PWM duty cycle for the shooter motor
    ledcWrite(ledc_channel_shooter, map(speed, 0, 100, 0, 255)); // Map speed from -100 to 100 to duty cycle 0-255
}
void blower_motor(int speed)
{
    // Set the PWM duty cycle for the blower motor
    ledcWrite(ledc_channel_blower, map(speed, 0, 100, 0, 255)); // Map speed from -100 to 100 to duty cycle 0-255
}
// Debug: đọc thanh ghi pin (43) theo 2 cách để so sánh
static uint8_t debug_read_reg(uint8_t reg, bool send_stop)
{
    Wire.beginTransmission(MotorDriverV2::kDefaultAddress);
    Wire.write(reg);
    Wire.endTransmission(send_stop);
    uint8_t n = Wire.requestFrom((int)MotorDriverV2::kDefaultAddress, 1);
    return n ? Wire.read() : 0;
}

void naca_motor_dump_registers()
{
    Serial.print("I2C devices:");
    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
            Serial.printf(" 0x%02X", addr);
    }
    Serial.println();
    Serial.print("FW version: ");
    Serial.println(mdV2.firmwareVersion());
    for (int i = 0; i < 5; i++)
    {
        Serial.printf("reg 43 - repeated start: %u | STOP (giong Python): %u\r\n",
                      debug_read_reg(43, false), debug_read_reg(43, true));
        delay(200);
    }
}

float read_battery_voltage()
{
    // Read the battery voltage from the motor driver
    float voltage = mdV2.battery();
    return voltage;
}
#ifndef NACA_LED_H
#define NACA_LED_H

#include <Arduino.h>

// LED báo trạng thái máy: LED RGB WS2812B và LED đơn D13.

#define NACA_LED_RGB_PIN 45       // WS2812B
#define NACA_LED_D13_PIN 48       // LED D13
#define NACA_LED_BRIGHTNESS 40    // Độ sáng LED RGB (0-255)
#define NACA_LED_BLINK_MS 500     // Nửa chu kỳ nháy D13 và nháy báo pin yếu
#define NACA_LED_BATTERY_LOW_V 11.5f // Dưới mức này thì báo pin yếu

// Màu LED RGB theo trạng thái. Pin yếu: nháy xen kẽ màu trạng thái với màu đỏ.
// LED D13: nháy đều khi điều khiển tay (firmware còn chạy), sáng liên tục khi chạy tự động, tắt lúc khởi động.
enum NacaLedState : uint8_t
{
    NACA_LED_INIT,       // đỏ: đang khởi động, hiệu chuẩn gyro
    NACA_LED_NO_LINK,    // xanh dương: chưa có hoặc mất sóng RC
    NACA_LED_SAFETY,     // vàng: SAFETY_MODE
    NACA_LED_ARMED,      // xanh lá: đang kích hoạt điều khiển tay
    NACA_LED_HEADLESS,   // xanh ngọc: kích hoạt, mode không đầu
    NACA_LED_AUTO_GREEN, // trắng: bài tự động nút xanh (bắn bóng)
    NACA_LED_AUTO_RED    // tím: bài tự động nút đỏ (gắp khối)
};

void naca_led_init();
void naca_led_set_state(NacaLedState state);
void naca_led_set_battery_low(bool low);
void naca_led_update(); // Gọi liên tục trong loop để nháy LED

#endif // NACA_LED_H

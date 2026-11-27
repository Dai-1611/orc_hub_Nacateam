#include "naca_led.h"

static NacaLedState led_state = NACA_LED_INIT;
static bool battery_low = false;
static bool blink_on = false;
static unsigned long last_blink_ms = 0;

// Ghi màu LED RGB theo trạng thái hiện tại; pin yếu thì nửa chu kỳ nháy hiện màu đỏ
static void led_show()
{
    const uint8_t B = NACA_LED_BRIGHTNESS;
    uint8_t r = 0, g = 0, b = 0;
    switch (led_state)
    {
    case NACA_LED_INIT:
        r = B;
        break;
    case NACA_LED_NO_LINK:
        b = B;
        break;
    case NACA_LED_SAFETY:
        r = B;
        g = B;
        break;
    case NACA_LED_ARMED:
        g = B;
        break;
    case NACA_LED_HEADLESS:
        g = B;
        b = B;
        break;
    case NACA_LED_AUTO_GREEN:
        r = B;
        g = B;
        b = B;
        break;
    case NACA_LED_AUTO_RED:
        r = B;
        b = B;
        break;
    }
    if (battery_low && blink_on)
    {
        r = B;
        g = 0;
        b = 0;
    }
    neopixelWrite(NACA_LED_RGB_PIN, r, g, b);
}

static bool led_state_is_auto()
{
    return led_state == NACA_LED_AUTO_GREEN || led_state == NACA_LED_AUTO_RED;
}

void naca_led_init()
{
    pinMode(NACA_LED_D13_PIN, OUTPUT);
    digitalWrite(NACA_LED_D13_PIN, LOW);
    led_show();
}

void naca_led_set_state(NacaLedState state)
{
    if (state == led_state)
    {
        return;
    }
    led_state = state;
    led_show();
    if (led_state_is_auto())
    {
        digitalWrite(NACA_LED_D13_PIN, HIGH);
    }
}

void naca_led_set_battery_low(bool low)
{
    if (low == battery_low)
    {
        return;
    }
    battery_low = low;
    led_show();
}

void naca_led_update()
{
    if (millis() - last_blink_ms < NACA_LED_BLINK_MS)
    {
        return;
    }
    last_blink_ms = millis();
    blink_on = !blink_on;
    if (!led_state_is_auto())
    {
        digitalWrite(NACA_LED_D13_PIN, blink_on ? HIGH : LOW);
    }
    if (battery_low)
    {
        led_show();
    }
}

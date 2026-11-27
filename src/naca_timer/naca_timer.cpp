#include "naca_timer.h"

volatile bool timer_flag = false;
volatile unsigned long cnt_timer = 0;
hw_timer_t * timer = nullptr;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void naca_timer_init()
{
    timerSemaphore = xSemaphoreCreateBinary(); // Create a binary semaphore for timer synchronization
    timer = timerBegin(0, 80, true); // Timer 0, prescaler 80, count up
    timerAttachInterrupt(timer, &onTimer, true); // Attach the interrupt handler
    timerAlarmWrite(timer, 1000, true); // Set alarm to trigger every 1000 ticks (1ms)
    timerAlarmEnable(timer); // Start the alarm (without this the ISR never fires)
}

void ARDUINO_ISR_ATTR onTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    cnt_timer++;
    timer_flag = true;
    portEXIT_CRITICAL_ISR(&timerMux);
    xSemaphoreGiveFromISR(timerSemaphore, NULL); // Give the semaphore from ISRx
}

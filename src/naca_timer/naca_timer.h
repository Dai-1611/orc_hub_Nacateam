#ifndef NACA_TIMER_H
#define NACA_TIMER_H

#include <Arduino.h>
#include <FreeRTOS.h>
extern volatile bool timer_flag;
extern volatile unsigned long cnt_timer;
extern hw_timer_t * timer;
extern volatile SemaphoreHandle_t timerSemaphore;
extern portMUX_TYPE timerMux;



void naca_timer_init();
void ARDUINO_ISR_ATTR onTimer();

#endif // NACA_TIMER_H
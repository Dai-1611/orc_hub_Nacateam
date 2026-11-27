#ifndef NACA_LOOP_RATE_H
#define NACA_LOOP_RATE_H

// Đo tần số chạy của 1 vòng lặp và in ra Serial mỗi NACA_LOOP_RATE_PRINT_MS.
// Cách dùng:
//   static NacaLoopRate rate = NACA_LOOP_RATE_INIT("manual");
//   naca_loop_rate_begin(rate);      // đầu mỗi lần lặp
//   ...
//   naca_loop_rate_end(rate);        // cuối mỗi lần lặp (không bắt buộc, để đo thời gian xử lý)
//   naca_loop_rate_timer_hit(rate);  // khi xử lý timer_flag (không bắt buộc, để đếm tick 1ms bị lỡ)
// In ra:
//   Hz        : số lần lặp mỗi giây
//   period max: khoảng cách lớn nhất giữa 2 lần lặp (vòng lặp bị khựng lâu nhất)
//   busy      : thời gian xử lý 1 lần lặp (trung bình / lớn nhất), > 1000 us thì không thể đạt 1 kHz
//   timer     : số tick timer 1ms trong khoảng đo, số lần timer_flag được xử lý, số tick bị lỡ
// Thời gian in Serial không bị tính vào period/busy.

#include <Arduino.h>
#include "naca_timer/naca_timer.h"

#define NACA_LOOP_RATE_DEBUG 1        // 0 để tắt hẳn (các hàm thành rỗng)
#define NACA_LOOP_RATE_PRINT_MS 1000  // chu kỳ in

struct NacaLoopRate
{
    const char *name;
    uint32_t calls;
    uint32_t ends;
    uint32_t period_max_us;
    uint32_t busy_max_us;
    uint64_t busy_sum_us;
    uint32_t timer_hits;
    uint32_t last_begin_us;
    uint32_t window_start_us;
    uint32_t window_start_ticks;
    bool started;
};

#define NACA_LOOP_RATE_INIT(n) {n, 0, 0, 0, 0, 0, 0, 0, 0, 0, false}

static inline void naca_loop_rate_reset_window(NacaLoopRate &r, uint32_t now_us)
{
    r.calls = 0;
    r.ends = 0;
    r.period_max_us = 0;
    r.busy_max_us = 0;
    r.busy_sum_us = 0;
    r.timer_hits = 0;
    r.window_start_us = now_us;
    r.window_start_ticks = cnt_timer;
}

static inline void naca_loop_rate_begin(NacaLoopRate &r)
{
#if NACA_LOOP_RATE_DEBUG
    uint32_t now = micros();
    // Lần đầu, hoặc vòng lặp này đã ngưng lâu (vd: auto vừa chạy xong rồi chạy lại) -> bắt đầu khoảng đo mới
    if (!r.started || now - r.last_begin_us > NACA_LOOP_RATE_PRINT_MS * 1000UL)
    {
        r.started = true;
        naca_loop_rate_reset_window(r, now);
        r.last_begin_us = now;
        r.calls = 1;
        return;
    }
    uint32_t period = now - r.last_begin_us;
    if (period > r.period_max_us)
    {
        r.period_max_us = period;
    }
    r.last_begin_us = now;
    r.calls++;

    uint32_t window_us = now - r.window_start_us;
    if (window_us >= NACA_LOOP_RATE_PRINT_MS * 1000UL)
    {
        uint32_t hz = (uint32_t)((uint64_t)r.calls * 1000000ULL / window_us);
        uint32_t busy_avg = r.ends ? (uint32_t)(r.busy_sum_us / r.ends) : 0;
        uint32_t ticks = cnt_timer - r.window_start_ticks;
        Serial.printf("[rate] %-8s %7lu Hz | period max %6lu us | busy avg %5lu us max %6lu us",
                      r.name, (unsigned long)hz, (unsigned long)r.period_max_us,
                      (unsigned long)busy_avg, (unsigned long)r.busy_max_us);
        if (r.timer_hits)
        {
            Serial.printf(" | timer %lu tick, handled %lu, missed %ld",
                          (unsigned long)ticks, (unsigned long)r.timer_hits, (long)ticks - (long)r.timer_hits);
        }
        Serial.println();
        // Không tính thời gian in vào lần lặp kế tiếp
        now = micros();
        naca_loop_rate_reset_window(r, now);
        r.last_begin_us = now;
    }
#endif
}

static inline void naca_loop_rate_end(NacaLoopRate &r)
{
#if NACA_LOOP_RATE_DEBUG
    if (!r.started)
    {
        return;
    }
    uint32_t busy = micros() - r.last_begin_us;
    if (busy > r.busy_max_us)
    {
        r.busy_max_us = busy;
    }
    r.busy_sum_us += busy;
    r.ends++;
#endif
}

static inline void naca_loop_rate_timer_hit(NacaLoopRate &r)
{
#if NACA_LOOP_RATE_DEBUG
    r.timer_hits++;
#endif
}

#endif // NACA_LOOP_RATE_H

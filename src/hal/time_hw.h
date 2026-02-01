#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TIME_HW_STK_BASE (0xE000F000u)

#define STK_CTLR  (*(volatile uint32_t *)(TIME_HW_STK_BASE + 0x00u))
#define STK_SR    (*(volatile uint32_t *)(TIME_HW_STK_BASE + 0x04u))
#define STK_CNTL  (*(volatile uint32_t *)(TIME_HW_STK_BASE + 0x08u))
#define STK_CNTH  (*(volatile uint32_t *)(TIME_HW_STK_BASE + 0x0Cu))
#define STK_CMPLR (*(volatile uint32_t *)(TIME_HW_STK_BASE + 0x10u))
#define STK_CMPHR (*(volatile uint32_t *)(TIME_HW_STK_BASE + 0x14u))

void     time_hw_init(void);
uint32_t time_hw_ticks_per_us(void);
uint32_t time_hw_ticks_per_ms(void);

extern uint32_t time_hw_tpus;
extern uint32_t time_hw_tpms;

static inline __attribute__((always_inline)) int32_t time_diff32(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b);
}
static inline __attribute__((always_inline)) int time_reached32(uint32_t now, uint32_t deadline)
{
    return (time_diff32(now, deadline) >= 0);
}

static inline __attribute__((always_inline)) uint32_t ms_to_ticks32(uint32_t ms)
{
    return ms * time_hw_tpms;
}

static inline __attribute__((always_inline)) uint32_t time_ticks32(void)
{
    return STK_CNTL;
}

uint64_t time_ticks64(void);

uint64_t time_us64(void);
uint64_t time_ms64(void);

static inline __attribute__((always_inline)) uint32_t micros(void)
{
    return (uint32_t)time_us64();
}

static inline __attribute__((always_inline)) uint32_t millis(void)
{
    return (uint32_t)time_ms64();
}

static inline __attribute__((always_inline)) uint64_t get_time64(void)
{
    return time_ms64();
}

static inline __attribute__((always_inline)) void delayTicks32(uint32_t ticks)
{
    if (!ticks) return;
    const uint32_t t0 = STK_CNTL;
    while ((uint32_t)(STK_CNTL - t0) < ticks) {
        __asm__ volatile ("nop");
    }
}

void delay_us(uint32_t us);
void delay(uint32_t ms);

#ifdef __cplusplus
}
#endif

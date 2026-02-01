#include "hal/time_hw.h"
#include "ch32v20x.h"

uint32_t time_hw_tpus = 1;
uint32_t time_hw_tpms = 1000;

static uint8_t g_init = 0;

static inline __attribute__((always_inline)) void ensure_init(void)
{
    if (__builtin_expect(!g_init, 0)) time_hw_init();
}

static inline __attribute__((always_inline)) uint64_t ticks64_raw(void)
{
    uint32_t hi1, lo, hi2;
    do {
        hi1 = STK_CNTH;
        lo  = STK_CNTL;
        hi2 = STK_CNTH;
    } while (hi1 != hi2);
    return ((uint64_t)hi1 << 32) | (uint64_t)lo;
}

void time_hw_init(void)
{
    SystemCoreClockUpdate();

    STK_CTLR = 0;
    STK_SR   = 0;
    STK_CNTL = 0;
    STK_CNTH = 0;

    STK_CMPLR = 0xFFFFFFFFu;
    STK_CMPHR = 0xFFFFFFFFu;

    STK_CTLR = (1u << 3) | (1u << 0);   // STRE=1, STCLK=HCLK/8, STE=1

    uint32_t tpus = (SystemCoreClock / 8u) / 1000000u;
    if (!tpus) tpus = 1;
    time_hw_tpus = tpus;

    uint32_t tpms = tpus * 1000u;
    if (!tpms) tpms = 1;
    time_hw_tpms = tpms;

    g_init = 1;
}

uint32_t time_hw_ticks_per_us(void) { ensure_init(); return time_hw_tpus; }
uint32_t time_hw_ticks_per_ms(void) { ensure_init(); return time_hw_tpms; }

uint64_t time_ticks64(void)
{
    ensure_init();
    return ticks64_raw();
}

uint64_t time_us64(void)
{
    ensure_init();
    const uint64_t t = ticks64_raw();
    const uint32_t d = time_hw_tpus;
    if (__builtin_expect(d == 1u, 0)) return t;
    return t / (uint64_t)d;
}

uint64_t time_ms64(void)
{
    ensure_init();
    const uint64_t t = ticks64_raw();
    const uint32_t d = time_hw_tpms;
    if (__builtin_expect(d == 1u, 0)) return t;
    return t / (uint64_t)d;
}

void delay_us(uint32_t us)
{
    if (!us) return;
    ensure_init();
    uint64_t t = (uint64_t)us * (uint64_t)time_hw_tpus;
    if (t > 0xFFFFFFFFu) t = 0xFFFFFFFFu;
    delayTicks32((uint32_t)t);
}

void delay(uint32_t ms)
{
    if (!ms) return;
    ensure_init();
    uint64_t t = (uint64_t)ms * (uint64_t)time_hw_tpms;
    if (t > 0xFFFFFFFFu) t = 0xFFFFFFFFu;
    delayTicks32((uint32_t)t);
}

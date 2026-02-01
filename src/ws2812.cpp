#include "ws2812.h"
#include "hal/time_hw.h"
#include "hal/irq_wch.h"

// WS2812B timing (datasheet):
//  - TH+TL = 1.25us ±600ns
//  - T0H   = 0.4us  ±150ns
//  - T1H   = 0.85us ±150ns
//  - RES low >= 50us
//
// STK = HCLK/8. Przy HCLK=144MHz => STK=18MHz => 55.56ns/tick.
#define WS2812_TBIT_TICKS  (22u)  // 1.222us @18MHz
#define WS2812_T0H_TICKS   (7u)   // 0.389us @18MHz
#define WS2812_T1H_TICKS   (15u)  // 0.833us @18MHz

static uint32_t g_ws2812_rst_ticks = 0;

#define RGB_H(p, m) do{ (p)->BSHR = (uint32_t)(m); }while(0)
#define RGB_L(p, m) do{ (p)->BCR  = (uint32_t)(m); }while(0)

static inline void enable_gpio_clock(GPIO_TypeDef* p)
{
    if      (p == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (p == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (p == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (p == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
}

static inline __attribute__((always_inline)) void wait_until(uint32_t deadline)
{
    while ((int32_t)(STK_CNTL - deadline) < 0) { }
}

void WS2812_class::init(uint8_t _num, GPIO_TypeDef* _port, uint16_t _pin)
{
    dirty = false;

    if (!g_ws2812_rst_ticks) {
        // 50us według datasheet, damy 100us
        g_ws2812_rst_ticks = 100u * time_hw_ticks_per_us();
        if (!g_ws2812_rst_ticks) g_ws2812_rst_ticks = 1u;
    }

    if (_num > MAX_NUM) _num = MAX_NUM;

    num  = _num;
    port = _port;
    pin  = _pin;

    enable_gpio_clock(port);

    GPIO_InitTypeDef gi = {0};
    gi.GPIO_Speed = GPIO_Speed_50MHz;
    gi.GPIO_Mode  = GPIO_Mode_Out_PP;
    gi.GPIO_Pin   = pin;
    GPIO_Init(port, &gi);

    RGB_L(port, pin);
    clear();
}

void WS2812_class::clear(void)
{
    for (uint32_t i = 0; i < (uint32_t)num; i++) last_grb[i] = 0u;
    dirty = true;
}

void WS2812_class::RST(void)
{
    RGB_L(port, pin);
    delayTicks32(g_ws2812_rst_ticks);
}

void WS2812_class::updata(void)
{
    if (!dirty) return;

    GPIO_TypeDef* const p = port;
    const uint32_t      m = (uint32_t)pin;

    uint32_t irq = irq_save_wch();

    // wyrównanie startu do granicy ticka STK
    uint32_t base = STK_CNTL;
    while (STK_CNTL == base) { }
    base = STK_CNTL;

    for (uint32_t led = 0; led < (uint32_t)num; led++)
    {
        // GRB, MSB-first
        uint32_t v = last_grb[led];

        for (uint32_t k = 0; k < 24u; k++)
        {
            const uint32_t th = (v & 0x800000u) ? WS2812_T1H_TICKS : WS2812_T0H_TICKS;
            v <<= 1;

            RGB_H(p, m);
            base += th;
            wait_until(base);

            RGB_L(p, m);
            base += (WS2812_TBIT_TICKS - th);
            wait_until(base);
        }
    }

    dirty = false;
    irq_restore_wch(irq);
    RST();
}

void WS2812_class::set_RGB(uint8_t R, uint8_t G, uint8_t B, uint8_t index)
{
    if (index >= num) return;

    const uint32_t packed = ((uint32_t)G << 16) | ((uint32_t)R << 8) | (uint32_t)B;
    if (last_grb[index] == packed) return;

    last_grb[index] = packed;
    dirty = true;
}

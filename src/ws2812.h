#pragma once
#include <stdint.h>
#include "ch32v20x.h"

class WS2812_class
{
public:
    static constexpr uint8_t MAX_NUM = 4;

    void init(uint8_t num, GPIO_TypeDef* port, uint16_t pin);

    void clear(void);
    void RST(void);
    void updata(void);

    void set_RGB(uint8_t R, uint8_t G, uint8_t B, uint8_t index);

    inline void set_RGB_online(uint8_t R, uint8_t G, uint8_t B, uint8_t index)
    {
        set_RGB(R, G, B, index);
    }

    inline bool is_dirty() const { return dirty; }

private:
    GPIO_TypeDef* port = nullptr;
    uint16_t      pin  = 0;
    uint8_t       num  = 0;

    // GRB packed: [23:16]=G, [15:8]=R, [7:0]=B
    uint32_t last_grb[MAX_NUM] = {0u, 0u, 0u, 0u};

    bool dirty = false;
};

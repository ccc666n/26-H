#include "delay.h"

#include "ti_msp_dl_config.h"

void delay_us(uint32_t us)
{
    delay_cycles((CPUCLK_FREQ / 1000000U) * us);
}

void delay_ms(uint32_t ms)
{
    delay_cycles((CPUCLK_FREQ / 1000U) * ms);
}

void delay_s(uint32_t s)
{
    while (s-- != 0U) {
        delay_ms(1000U);
    }
}

void delay_1us(uint32_t us)
{
    delay_us(us);
}

void delay_1ms(uint32_t ms)
{
    delay_ms(ms);
}

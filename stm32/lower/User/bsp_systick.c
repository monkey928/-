#include "bsp_systick.h"
#include "app_config.h"

static volatile uint32_t g_ms_ticks = 0;

void SysTick_BspInit(void)
{
    SysTick_Config(APP_CPU_HZ / 1000U);
}

uint32_t SysTick_Millis(void)
{
    return g_ms_ticks;
}

void SysTick_IncTick(void)
{
    g_ms_ticks++;
}

void Delay_ms(uint32_t ms)
{
    uint32_t start = SysTick_Millis();
    while ((uint32_t)(SysTick_Millis() - start) < ms) {
    }
}

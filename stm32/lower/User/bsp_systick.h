#ifndef BSP_SYSTICK_H
#define BSP_SYSTICK_H

#include "stm32f10x.h"

void SysTick_BspInit(void);
uint32_t SysTick_Millis(void);
void SysTick_IncTick(void);
void Delay_ms(uint32_t ms);

#endif

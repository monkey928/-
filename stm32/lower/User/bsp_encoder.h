#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "stm32f10x.h"
#include <stdint.h>

void Encoder_BspInit(void);
int32_t Encoder_GetLeftDelta(void);
int32_t Encoder_GetRightDelta(void);

#endif

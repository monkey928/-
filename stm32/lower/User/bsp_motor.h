#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "stm32f10x.h"
#include <stdint.h>

void Motor_BspInit(void);
void Motor_SetPwm(int16_t left_pwm, int16_t right_pwm);
void Motor_Coast(void);
void Motor_Brake(void);
void Motor_Stop(void);

#endif

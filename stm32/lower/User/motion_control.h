#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "stm32f10x.h"
#include <stdint.h>

typedef enum {
    MOTION_MODE_STOP = 0,
    MOTION_MODE_VELOCITY = 1,
    MOTION_MODE_GOAL = 2,
    MOTION_MODE_EMERGENCY = 3,
    MOTION_MODE_STALL = 4
} MotionMode_t;

typedef struct {
    float left_speed_mm_s;
    float right_speed_mm_s;
    float x_mm;
    float y_mm;
    float theta_mrad;
    uint8_t stall_flags;
    MotionMode_t mode;
} MotionTelemetry_t;

void Motion_Init(void);
void Motion_CommandVelocity(int16_t v_mm_s, int16_t w_mrad_s, uint32_t now_ms);
void Motion_CommandGoal(int16_t x_mm, int16_t y_mm, uint16_t arrive_ms, uint32_t now_ms);
void Motion_Stop(void);
void Motion_EmergencyStop(void);
void Motion_ClearEmergencyStop(void);
void Motion_Update(uint32_t now_ms);
void Motion_GetTelemetry(MotionTelemetry_t *tel);

#endif

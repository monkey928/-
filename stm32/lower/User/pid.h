#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_measurement;
    uint8_t has_prev_measurement;
    float out_min;
    float out_max;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);
void PID_Reset(PID_t *pid);
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt_s);

#endif

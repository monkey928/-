#include "pid.h"
#include "app_math.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->out_min = out_min;
    pid->out_max = out_max;
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->has_prev_measurement = 0U;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt_s)
{
    float error = setpoint - measurement;
    float derivative = 0.0f;
    float p_term;
    float i_term = 0.0f;
    float d_term;
    float integral_next;
    float output_unclamped;
    float output;

    if (dt_s <= 0.000001f) {
        return 0.0f;
    }

    if (pid->has_prev_measurement != 0U) {
        derivative = -(measurement - pid->prev_measurement) / dt_s;
    }
    p_term = pid->kp * error;
    d_term = pid->kd * derivative;

    if (pid->ki > 0.000001f || pid->ki < -0.000001f) {
        float i_min = pid->out_min;
        float i_max = pid->out_max;
        float i_delta;

        integral_next = pid->integral + error * dt_s;
        i_term = pid->ki * integral_next;
        if (i_min > i_max) {
            float tmp = i_min;
            i_min = i_max;
            i_max = tmp;
        }
        i_term = App_ClampFloat(i_term, i_min, i_max);
        integral_next = i_term / pid->ki;

        output_unclamped = p_term + i_term + d_term;
        i_delta = pid->ki * error * dt_s;

        if (!((output_unclamped > pid->out_max && i_delta > 0.0f) ||
              (output_unclamped < pid->out_min && i_delta < 0.0f))) {
            pid->integral = integral_next;
        }
    } else {
        pid->integral = 0.0f;
    }

    output = p_term + pid->ki * pid->integral + d_term;
    output = App_ClampFloat(output, pid->out_min, pid->out_max);

    pid->prev_measurement = measurement;
    pid->has_prev_measurement = 1U;
    return output;
}

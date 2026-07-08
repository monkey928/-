#include "motion_control.h"
#include "app_math.h"
#include "app_config.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "pid.h"
#include <math.h>

typedef struct {
    float x_mm;
    float y_mm;
    float theta_mrad;
} Odom_t;

static PID_t g_left_pid;
static PID_t g_right_pid;
static MotionMode_t g_mode = MOTION_MODE_STOP;
static Odom_t g_odom;
static float g_left_speed_mm_s;
static float g_right_speed_mm_s;
static float g_cmd_v_mm_s;
static float g_cmd_w_mrad_s;
static float g_limited_v_mm_s;
static float g_limited_w_mrad_s;
static float g_goal_x_mm;
static float g_goal_y_mm;
static uint16_t g_goal_arrive_ms;
static uint32_t g_last_cmd_ms;
static uint32_t g_goal_start_ms;
static float g_heading_cos = 1.0f;
static float g_heading_sin = 0.0f;
static uint8_t g_heading_renorm_count;
#if STALL_PROTECTION_ENABLE
static uint16_t g_left_stall_ms;
static uint16_t g_right_stall_ms;
static uint8_t g_stall_flags;
#endif

static float normalize_mrad(float a)
{
    const float pi_mrad = 3141.5926f;
    const float two_pi_mrad = 6283.1852f;
    int32_t turns;

    turns = (int32_t)(a / two_pi_mrad);
    a -= (float)turns * two_pi_mrad;

    if (a > pi_mrad) {
        a -= two_pi_mrad;
    } else if (a < -pi_mrad) {
        a += two_pi_mrad;
    }
    return a;
}

static void fast_sincos(float angle_rad, float *s, float *c)
{
    float a2;

    if (angle_rad < FAST_TRIG_SMALL_ANGLE_RAD &&
        angle_rad > -FAST_TRIG_SMALL_ANGLE_RAD) {
        a2 = angle_rad * angle_rad;
        *s = angle_rad * (1.0f - a2 / 6.0f);
        *c = 1.0f - a2 * 0.5f;
    } else {
        *s = sinf(angle_rad);
        *c = cosf(angle_rad);
    }
}

static void update_heading_vector(float dtheta_mrad)
{
    float s;
    float c;
    float next_cos;
    float next_sin;

    fast_sincos(dtheta_mrad / 1000.0f, &s, &c);
    next_cos = g_heading_cos * c - g_heading_sin * s;
    next_sin = g_heading_sin * c + g_heading_cos * s;
    g_heading_cos = next_cos;
    g_heading_sin = next_sin;

    g_heading_renorm_count++;
    if (g_heading_renorm_count >= HEADING_RENORM_INTERVAL) {
        float norm = sqrtf(g_heading_cos * g_heading_cos + g_heading_sin * g_heading_sin);
        if (norm > 0.000001f) {
            g_heading_cos /= norm;
            g_heading_sin /= norm;
        } else {
            g_heading_cos = 1.0f;
            g_heading_sin = 0.0f;
        }
        g_heading_renorm_count = 0U;
    }
}

static int16_t pwm_from_float(float v)
{
    v = App_ClampFloat(v, -(float)MOTOR_PWM_PERIOD, (float)MOTOR_PWM_PERIOD);
    return (int16_t)v;
}

static void stop_and_reset_pid(void)
{
    Motor_Stop();
    PID_Reset(&g_left_pid);
    PID_Reset(&g_right_pid);
    g_cmd_v_mm_s = 0.0f;
    g_cmd_w_mrad_s = 0.0f;
    g_limited_v_mm_s = 0.0f;
    g_limited_w_mrad_s = 0.0f;
#if STALL_PROTECTION_ENABLE
    g_left_stall_ms = 0U;
    g_right_stall_ms = 0U;
#endif
}

static void update_odometry(float left_delta_mm, float right_delta_mm)
{
    float ds = 0.5f * (left_delta_mm + right_delta_mm);
    float dtheta_mrad = (right_delta_mm - left_delta_mm) * 1000.0f / WHEEL_BASE_MM;
    float half_s;
    float half_c;
    float mid_cos;
    float mid_sin;

    fast_sincos(0.5f * dtheta_mrad / 1000.0f, &half_s, &half_c);
    mid_cos = g_heading_cos * half_c - g_heading_sin * half_s;
    mid_sin = g_heading_sin * half_c + g_heading_cos * half_s;

    g_odom.x_mm += ds * mid_cos;
    g_odom.y_mm += ds * mid_sin;
    g_odom.theta_mrad = normalize_mrad(g_odom.theta_mrad + dtheta_mrad);
    update_heading_vector(dtheta_mrad);
}

static void goal_controller(float *v_mm_s, float *w_mrad_s, uint32_t now_ms)
{
    float dx = g_goal_x_mm - g_odom.x_mm;
    float dy = g_goal_y_mm - g_odom.y_mm;
    float dist = sqrtf(dx * dx + dy * dy);
    float target_theta;
    float bearing;
    float abs_bearing;
    uint32_t elapsed = now_ms - g_goal_start_ms;
    uint32_t remain_ms = 0U;
    uint32_t timeout_ms;
    float desired_v;

    timeout_ms = (g_goal_arrive_ms > 0U) ?
                 ((uint32_t)g_goal_arrive_ms + GOAL_TIMEOUT_EXTRA_MS) :
                 GOAL_DEFAULT_TIMEOUT_MS;

    if (dist < GOAL_POSITION_TOLERANCE_MM || elapsed > timeout_ms) {
        g_mode = MOTION_MODE_STOP;
        *v_mm_s = 0.0f;
        *w_mrad_s = 0.0f;
        return;
    }

    target_theta = atan2f(dy, dx) * 1000.0f;
    bearing = normalize_mrad(target_theta - g_odom.theta_mrad);

    if (g_goal_arrive_ms > 0U && elapsed < (uint32_t)g_goal_arrive_ms) {
        remain_ms = (uint32_t)g_goal_arrive_ms - elapsed;
        if (remain_ms < GOAL_MIN_REMAIN_MS) {
            remain_ms = GOAL_MIN_REMAIN_MS;
        }
        desired_v = dist * 1000.0f / (float)remain_ms;
        if (desired_v < GOAL_MIN_LINEAR_SPEED_MM_S) {
            desired_v = GOAL_MIN_LINEAR_SPEED_MM_S;
        }
    } else {
        desired_v = 2.0f * dist;
        if (desired_v < GOAL_MIN_LINEAR_SPEED_MM_S) {
            /* Keep enough PWM to overcome the motor's 1.5 V starting voltage
             * near the goal; the tolerance check above ends the approach. */
            desired_v = GOAL_MIN_LINEAR_SPEED_MM_S;
        }
    }

    *v_mm_s = App_ClampFloat(desired_v, 0.0f, MAX_LINEAR_SPEED_MM_S);
    *w_mrad_s = App_ClampFloat(GOAL_HEADING_KP * bearing, -MAX_ANGULAR_SPEED_MRAD_S, MAX_ANGULAR_SPEED_MRAD_S);

    abs_bearing = fabsf(bearing);
    if (abs_bearing > 750.0f) {
        *v_mm_s *= 0.25f;
    } else if (abs_bearing > 350.0f) {
        *v_mm_s *= 0.55f;
    }
}

#if STALL_PROTECTION_ENABLE
static float absf_local(float v)
{
    return (v < 0.0f) ? -v : v;
}

static uint8_t wheel_is_stalled(float pwm, float set_mm_s, float speed_mm_s)
{
    return (absf_local(pwm) >= (float)STALL_PWM_THRESHOLD &&
            absf_local(set_mm_s) >= STALL_COMMAND_MIN_MM_S &&
            absf_local(speed_mm_s) <= STALL_SPEED_THRESHOLD_MM_S) ? 1U : 0U;
}

static uint8_t update_stall_timer(uint16_t *stall_ms, uint8_t stalled)
{
    if (stalled) {
        if (*stall_ms < STALL_DETECT_TIME_MS) {
            *stall_ms = (uint16_t)(*stall_ms + CONTROL_PERIOD_MS);
        }
    } else {
        *stall_ms = 0U;
    }

    return (*stall_ms >= STALL_DETECT_TIME_MS) ? 1U : 0U;
}
#endif

static float slew_limit(float target, float current, float max_delta)
{
    float delta = target - current;

    if (delta > max_delta) {
        return current + max_delta;
    }
    if (delta < -max_delta) {
        return current - max_delta;
    }
    return target;
}

static void limit_command_accel(float *v_mm_s, float *w_mrad_s, float dt_s)
{
    g_limited_v_mm_s = slew_limit(*v_mm_s,
                                  g_limited_v_mm_s,
                                  MAX_LINEAR_ACCEL_MM_S2 * dt_s);
    g_limited_w_mrad_s = slew_limit(*w_mrad_s,
                                    g_limited_w_mrad_s,
                                    MAX_ANGULAR_ACCEL_MRAD_S2 * dt_s);
    *v_mm_s = g_limited_v_mm_s;
    *w_mrad_s = g_limited_w_mrad_s;
}

static void clear_pose_and_speed(void)
{
    g_odom.x_mm = 0.0f;
    g_odom.y_mm = 0.0f;
    g_odom.theta_mrad = 0.0f;
    g_left_speed_mm_s = 0.0f;
    g_right_speed_mm_s = 0.0f;
    g_heading_cos = 1.0f;
    g_heading_sin = 0.0f;
    g_heading_renorm_count = 0U;
}

static void update_encoder_feedback_from_ticks(int32_t left_ticks, int32_t right_ticks, float dt_s)
{
    float left_delta_mm;
    float right_delta_mm;

    left_delta_mm = (float)left_ticks / ENCODER_COUNTS_PER_MM;
    right_delta_mm = (float)right_ticks / ENCODER_COUNTS_PER_MM;
    g_left_speed_mm_s = left_delta_mm / dt_s;
    g_right_speed_mm_s = right_delta_mm / dt_s;
    update_odometry(left_delta_mm, right_delta_mm);
}

static void update_encoder_feedback(float dt_s)
{
    update_encoder_feedback_from_ticks(Encoder_GetLeftDelta(), Encoder_GetRightDelta(), dt_s);
}

void Motion_Init(void)
{
    PID_Init(&g_left_pid, LEFT_PID_KP, LEFT_PID_KI, LEFT_PID_KD, -(float)MOTOR_PWM_PERIOD, (float)MOTOR_PWM_PERIOD);
    PID_Init(&g_right_pid, RIGHT_PID_KP, RIGHT_PID_KI, RIGHT_PID_KD, -(float)MOTOR_PWM_PERIOD, (float)MOTOR_PWM_PERIOD);
    clear_pose_and_speed();
    Motion_Stop();
}

void Motion_CommandVelocity(int16_t v_mm_s, int16_t w_mrad_s, uint32_t now_ms)
{
    g_cmd_v_mm_s = App_ClampFloat((float)v_mm_s, -MAX_LINEAR_SPEED_MM_S, MAX_LINEAR_SPEED_MM_S);
    g_cmd_w_mrad_s = App_ClampFloat((float)w_mrad_s, -MAX_ANGULAR_SPEED_MRAD_S, MAX_ANGULAR_SPEED_MRAD_S);
    g_last_cmd_ms = now_ms;
    if (g_mode != MOTION_MODE_EMERGENCY && g_mode != MOTION_MODE_STALL) {
        g_mode = MOTION_MODE_VELOCITY;
    }
}

void Motion_CommandGoal(int16_t x_mm, int16_t y_mm, uint16_t arrive_ms, uint32_t now_ms)
{
    float rel_x = (float)x_mm;
    float rel_y = (float)y_mm;

    g_goal_x_mm = g_odom.x_mm + rel_x * g_heading_cos - rel_y * g_heading_sin;
    g_goal_y_mm = g_odom.y_mm + rel_x * g_heading_sin + rel_y * g_heading_cos;
    g_goal_arrive_ms = arrive_ms;
    g_goal_start_ms = now_ms;
    g_last_cmd_ms = now_ms;
    if (g_mode != MOTION_MODE_EMERGENCY && g_mode != MOTION_MODE_STALL) {
        g_mode = MOTION_MODE_GOAL;
    }
}

void Motion_Stop(void)
{
    if (g_mode != MOTION_MODE_EMERGENCY && g_mode != MOTION_MODE_STALL) {
        g_mode = MOTION_MODE_STOP;
    }
    stop_and_reset_pid();
}

void Motion_EmergencyStop(void)
{
    g_mode = MOTION_MODE_EMERGENCY;
    stop_and_reset_pid();
}

#if STALL_PROTECTION_ENABLE
static void Motion_StallStop(uint8_t flags)
{
    g_stall_flags = flags;
    g_mode = MOTION_MODE_STALL;
    stop_and_reset_pid();
}
#endif

void Motion_ClearEmergencyStop(void)
{
    if (g_mode == MOTION_MODE_EMERGENCY || g_mode == MOTION_MODE_STALL) {
        g_mode = MOTION_MODE_STOP;
    }
#if STALL_PROTECTION_ENABLE
    g_stall_flags = 0U;
#endif
    stop_and_reset_pid();
}

void Motion_Update(uint32_t now_ms)
{
    static uint32_t s_last_update_ms;
    static uint8_t s_has_last_update;
    float dt_s;
    float v_mm_s = 0.0f;
    float w_mrad_s = 0.0f;
    float left_set_mm_s;
    float right_set_mm_s;
    float left_pwm;
    float right_pwm;

    /* Use measured dt: OLED refresh and telemetry can delay this call past
     * CONTROL_PERIOD_MS, and a fixed dt would spike the measured speed. */
    if (s_has_last_update != 0U) {
        uint32_t dt_ms = now_ms - s_last_update_ms;

        if (dt_ms < 1U) {
            dt_ms = 1U;
        } else if (dt_ms > 100U) {
            dt_ms = 100U;
        }
        dt_s = (float)dt_ms / 1000.0f;
    } else {
        dt_s = (float)CONTROL_PERIOD_MS / 1000.0f;
        s_has_last_update = 1U;
    }
    s_last_update_ms = now_ms;

    update_encoder_feedback(dt_s);

    if (g_mode == MOTION_MODE_EMERGENCY || g_mode == MOTION_MODE_STALL ||
        g_mode == MOTION_MODE_STOP) {
        stop_and_reset_pid();
        return;
    }

    if (g_mode == MOTION_MODE_VELOCITY) {
        if ((uint32_t)(now_ms - g_last_cmd_ms) > COMMAND_TIMEOUT_MS) {
            Motion_Stop();
            return;
        }
        v_mm_s = g_cmd_v_mm_s;
        w_mrad_s = g_cmd_w_mrad_s;
    } else if (g_mode == MOTION_MODE_GOAL) {
        goal_controller(&v_mm_s, &w_mrad_s, now_ms);
        if (g_mode == MOTION_MODE_STOP) {
            stop_and_reset_pid();
            return;
        }
    }

    limit_command_accel(&v_mm_s, &w_mrad_s, dt_s);

    left_set_mm_s = v_mm_s - (w_mrad_s * WHEEL_BASE_MM / 2000.0f);
    right_set_mm_s = v_mm_s + (w_mrad_s * WHEEL_BASE_MM / 2000.0f);
    left_set_mm_s = App_ClampFloat(left_set_mm_s, -MAX_WHEEL_SPEED_MM_S, MAX_WHEEL_SPEED_MM_S);
    right_set_mm_s = App_ClampFloat(right_set_mm_s, -MAX_WHEEL_SPEED_MM_S, MAX_WHEEL_SPEED_MM_S);

    left_pwm = PID_Update(&g_left_pid, left_set_mm_s, g_left_speed_mm_s, dt_s);
    right_pwm = PID_Update(&g_right_pid, right_set_mm_s, g_right_speed_mm_s, dt_s);

#if STALL_PROTECTION_ENABLE
    {
        uint8_t flags = 0U;

        if (update_stall_timer(&g_left_stall_ms, wheel_is_stalled(left_pwm, left_set_mm_s, g_left_speed_mm_s))) {
            flags |= 0x01U;
        }
        if (update_stall_timer(&g_right_stall_ms, wheel_is_stalled(right_pwm, right_set_mm_s, g_right_speed_mm_s))) {
            flags |= 0x02U;
        }
        if (flags != 0U) {
            Motion_StallStop(flags);
            return;
        }
    }
#endif

    Motor_SetPwm(pwm_from_float(left_pwm), pwm_from_float(right_pwm));
}

void Motion_GetTelemetry(MotionTelemetry_t *tel)
{
    if (tel == 0) {
        return;
    }

    tel->left_speed_mm_s = g_left_speed_mm_s;
    tel->right_speed_mm_s = g_right_speed_mm_s;
    tel->x_mm = g_odom.x_mm;
    tel->y_mm = g_odom.y_mm;
    tel->theta_mrad = g_odom.theta_mrad;
#if STALL_PROTECTION_ENABLE
    tel->stall_flags = g_stall_flags;
#else
    tel->stall_flags = 0U;
#endif
    tel->mode = g_mode;
}

#include "wheel_selftest.h"
#include "app_config.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_systick.h"

#if WATCHDOG_ENABLE
#include "stm32f10x_iwdg.h"
#endif

/* 90% duty: near full speed so the spin direction and encoder counts are
 * obvious. TB6612 drop at these currents is small; keep wheels off ground. */
#define SELFTEST_PWM            ((int16_t)(MOTOR_PWM_PERIOD * 9U / 10U))
#define SELFTEST_RUN_MS         5000UL
#define SELFTEST_PAUSE_MS       500UL
#define SELFTEST_SAMPLE_MS      10UL

static WheelSelftestResult_t g_result;

static void selftest_feed_watchdog(void)
{
#if WATCHDOG_ENABLE
    IWDG_ReloadCounter();
#endif
}

static void run_phase(int16_t pwm, uint32_t duration_ms,
                      int32_t *left_total, int32_t *right_total)
{
    uint32_t start = SysTick_Millis();
    uint32_t last_sample = start;

    /* Discard counts accumulated before this phase. */
    (void)Encoder_GetLeftDelta();
    (void)Encoder_GetRightDelta();

    Motor_SetPwm(pwm, pwm);

    while ((uint32_t)(SysTick_Millis() - start) < duration_ms) {
        uint32_t now = SysTick_Millis();

        if ((uint32_t)(now - last_sample) >= SELFTEST_SAMPLE_MS) {
            last_sample = now;
            if (left_total != 0) {
                *left_total += Encoder_GetLeftDelta();
            } else {
                (void)Encoder_GetLeftDelta();
            }
            if (right_total != 0) {
                *right_total += Encoder_GetRightDelta();
            } else {
                (void)Encoder_GetRightDelta();
            }
        }
        selftest_feed_watchdog();
    }

    Motor_Stop();
}

void WheelSelftest_Run(void)
{
    g_result.left_forward_counts = 0;
    g_result.right_forward_counts = 0;
    g_result.left_reverse_counts = 0;
    g_result.right_reverse_counts = 0;
    g_result.done = 0U;

    /* Forward 5 s. */
    run_phase(SELFTEST_PWM, SELFTEST_RUN_MS,
              &g_result.left_forward_counts, &g_result.right_forward_counts);

    /* Short pause so the reversal is not an instant direction slam. */
    run_phase(0, SELFTEST_PAUSE_MS, 0, 0);

    /* Reverse 5 s. */
    run_phase((int16_t)(-SELFTEST_PWM), SELFTEST_RUN_MS,
              &g_result.left_reverse_counts, &g_result.right_reverse_counts);

    Motor_Stop();
    g_result.done = 1U;
}

const WheelSelftestResult_t *WheelSelftest_GetResult(void)
{
    return &g_result;
}

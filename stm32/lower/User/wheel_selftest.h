#ifndef WHEEL_SELFTEST_H
#define WHEEL_SELFTEST_H

#include <stdint.h>

/* Encoder counts accumulated during each self-test phase.
 * With correct wiring and encoder signs:
 *   forward counts > 0, reverse counts < 0, left/right roughly equal.
 */
typedef struct {
    int32_t left_forward_counts;
    int32_t right_forward_counts;
    int32_t left_reverse_counts;
    int32_t right_reverse_counts;
    uint8_t done;
} WheelSelftestResult_t;

/* Blocking power-on self test:
 * both wheels forward 5 s -> coast 0.5 s -> both wheels reverse 5 s -> stop.
 * Requires SysTick_BspInit(), Motor_BspInit() and Encoder_BspInit() first.
 * Feeds the IWDG internally when WATCHDOG_ENABLE is set.
 */
void WheelSelftest_Run(void);

const WheelSelftestResult_t *WheelSelftest_GetResult(void);

#endif

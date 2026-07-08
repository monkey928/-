#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "stm32f10x.h"

/* MCU clock: use the usual Keil/SystemInit 72 MHz configuration for STM32F103C8T6. */
#define APP_CPU_HZ                  72000000UL

/* UART link to RK3588. */
#define RK_USART_BAUDRATE           115200U

/* Control loop. */
#define CONTROL_PERIOD_MS           10U
#define COMMAND_TIMEOUT_MS          300U
#define TELEMETRY_PERIOD_MS         50U

/* 0.96 inch SSD1306 OLED. Uses software I2C on PB10=SCL, PB11=SDA. */
#define OLED_ENABLE                 1U
#define OLED_I2C_ADDR               0x3CU
#define OLED_REFRESH_PERIOD_MS      200U
/* 15 cycles ~= 300-350 kHz software I2C, inside the SSD1306 400 kHz spec.
 * Values below ~10 overclock the bus and writes get NACKed (frozen screen). */
#define OLED_I2C_DELAY_CYCLES       15U

/* Independent watchdog. Timeout is approximate because STM32F1 LSI varies. */
#define WATCHDOG_ENABLE             1U
#define WATCHDOG_FREEZE_IN_DEBUG    1U
#define WATCHDOG_TIMEOUT_MS         1000UL
#define WATCHDOG_LSI_HZ             40000UL
#define WATCHDOG_PRESCALER_DIV      64UL
#define WATCHDOG_RELOAD_VALUE       ((WATCHDOG_TIMEOUT_MS * WATCHDOG_LSI_HZ / WATCHDOG_PRESCALER_DIV / 1000UL) - 1UL)

/* Chassis geometry. Tune these to your actual trash-can base.
 * ENCODER_PULSES_PER_REV is the encoder A-channel pulses per motor shaft turn.
 * ENCODER_QUADRATURE_FACTOR is 4 when TIM encoder mode counts all A/B edges.
 * If your encoder data sheet already gives 4x counts, set QUADRATURE_FACTOR to 1.
 * Best calibration: mark one wheel turn and set ENCODER_MEASURED_COUNTS_PER_REV.
 */
#define WHEEL_DIAMETER_MM           100.0f
#define WHEEL_BASE_MM               300.0f
#define ENCODER_MEASURED_COUNTS_PER_REV 3960.0f
#define ENCODER_PULSES_PER_REV      11.0f
#define ENCODER_QUADRATURE_FACTOR   4.0f
#define GEAR_RATIO                  90.0f

#define ENCODER_CALCULATED_COUNTS_PER_REV (ENCODER_PULSES_PER_REV * ENCODER_QUADRATURE_FACTOR * GEAR_RATIO)
#define ENCODER_COUNTS_PER_REV      ((ENCODER_MEASURED_COUNTS_PER_REV > 0.0f) ? ENCODER_MEASURED_COUNTS_PER_REV : ENCODER_CALCULATED_COUNTS_PER_REV)
#define WHEEL_CIRCUMFERENCE_MM      (3.1415926f * WHEEL_DIAMETER_MM)
#define ENCODER_COUNTS_PER_MM       (ENCODER_COUNTS_PER_REV / WHEEL_CIRCUMFERENCE_MM)

/* Change sign here if one encoder counts backward for forward motion. */
#define LEFT_ENCODER_SIGN           1
#define RIGHT_ENCODER_SIGN          -1

/* Motor PWM: TIM3 at 20 kHz from 72 MHz timer clock. */
#define MOTOR_PWM_TIMER_HZ          72000000UL
#define MOTOR_PWM_HZ                20000UL
#define MOTOR_PWM_PERIOD            ((uint16_t)(MOTOR_PWM_TIMER_HZ / MOTOR_PWM_HZ - 1U))

/* TB6612 driver options.
 * Connect STBY to PB5, or tie STBY high and set MOTOR_TB6612_STBY_GPIO to 0.
 * Set MOTOR_STOP_BRAKE_ENABLE to 0 if short-brake current causes resets or heating.
 */
#define MOTOR_TB6612_STBY_GPIO      1U
#define MOTOR_STOP_BRAKE_ENABLE     0U
#define MOTOR_BRAKE_PWM             MOTOR_PWM_PERIOD

/* Speed limits derived from the JGB37-520 motor (12V, 90:1):
 * rated-load 85 RPM with 100 mm wheels = 445 mm/s per wheel.
 * Keep the commanded limits below rated speed so PID has headroom.
 */
#define MAX_LINEAR_SPEED_MM_S       400.0f
#define MAX_ANGULAR_SPEED_MRAD_S    2600.0f
#define MAX_WHEEL_SPEED_MM_S        440.0f
#define MAX_LINEAR_ACCEL_MM_S2      1200.0f
#define MAX_ANGULAR_ACCEL_MRAD_S2   5000.0f
#define GOAL_POSITION_TOLERANCE_MM  45.0f
#define GOAL_TIMEOUT_EXTRA_MS       800U
#define GOAL_DEFAULT_TIMEOUT_MS     1500U
#define GOAL_MIN_REMAIN_MS          120U
#define GOAL_HEADING_KP             3.2f
#define GOAL_MIN_LINEAR_SPEED_MM_S  120.0f

/* STM32F103 has no FPU. Keep fast small-angle math in odometry where possible. */
#define FAST_TRIG_SMALL_ANGLE_RAD   0.20f
#define HEADING_RENORM_INTERVAL     50U

/* Encoder-based stall protection. It cannot replace current sensing, but can stop a jammed wheel. */
#define STALL_PROTECTION_ENABLE     0U
#define STALL_PWM_THRESHOLD         ((uint16_t)(MOTOR_PWM_PERIOD * 60U / 100U))
#define STALL_COMMAND_MIN_MM_S      120.0f
#define STALL_SPEED_THRESHOLD_MM_S  25.0f
#define STALL_DETECT_TIME_MS        500U

/* Default speed PID. These are safe starting values, not final tuned values. */
#define LEFT_PID_KP                 8.0f
#define LEFT_PID_KI                 3.0f
#define LEFT_PID_KD                 0.15f
#define RIGHT_PID_KP                8.0f
#define RIGHT_PID_KI                3.0f
#define RIGHT_PID_KD                0.15f

#endif

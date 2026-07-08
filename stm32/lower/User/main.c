#include "app_config.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_oled.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "motion_control.h"
#include "protocol.h"

#if WATCHDOG_ENABLE
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_iwdg.h"
#endif

#if WATCHDOG_ENABLE
#if WATCHDOG_RELOAD_VALUE > 0x0FFFUL
#error "WATCHDOG_TIMEOUT_MS is too large for IWDG prescaler 64."
#endif
#if WATCHDOG_RELOAD_VALUE < 1UL
#error "WATCHDOG_TIMEOUT_MS is too small for IWDG prescaler 64."
#endif
#endif

static int16_t to_i16(float v)
{
    if (v > 32767.0f) {
        return 32767;
    }
    if (v < -32768.0f) {
        return -32768;
    }
    return (int16_t)v;
}

static void process_command(const ProtocolCommand_t *cmd, uint32_t now_ms)
{
    switch (cmd->type) {
    case PROTO_CMD_VELOCITY:
        Motion_CommandVelocity(cmd->v_mm_s, cmd->w_mrad_s, now_ms);
        break;

    case PROTO_CMD_GOAL:
        Motion_CommandGoal(cmd->x_mm, cmd->y_mm, cmd->arrive_ms, now_ms);
        break;

    case PROTO_CMD_STOP:
        Motion_Stop();
        break;

    case PROTO_CMD_EMERGENCY_STOP:
        Motion_EmergencyStop();
        break;

    case PROTO_CMD_CLEAR_EMERGENCY:
        Motion_ClearEmergencyStop();
        break;

    default:
        break;
    }
}

static void send_telemetry(void)
{
    MotionTelemetry_t m;
    ProtocolTelemetry_t p;

    Motion_GetTelemetry(&m);
    p.left_mm_s = to_i16(m.left_speed_mm_s);
    p.right_mm_s = to_i16(m.right_speed_mm_s);
    p.x_mm = to_i16(m.x_mm);
    p.y_mm = to_i16(m.y_mm);
    p.theta_mrad = to_i16(m.theta_mrad);
    /* Battery ADC was removed by hardware decision; keep protocol field stable. */
    p.battery_mv = 0U;
    p.mode = (uint8_t)m.mode;
    p.stall_flags = m.stall_flags;
    Protocol_SendTelemetry(&p);
}

static void update_oled(void)
{
    MotionTelemetry_t m;
    ProtocolDiagnostics_t d;

    Motion_GetTelemetry(&m);
    Protocol_GetDiagnostics(&d);
    OLED_ShowTelemetry(&m, &d);
}

static void watchdog_init(void)
{
#if WATCHDOG_ENABLE
#if WATCHDOG_FREEZE_IN_DEBUG
    DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);
#endif
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload((uint16_t)WATCHDOG_RELOAD_VALUE);
    IWDG_ReloadCounter();
    IWDG_Enable();
#endif
}

static void watchdog_feed(void)
{
#if WATCHDOG_ENABLE
    IWDG_ReloadCounter();
#endif
}

int main(void)
{
    uint32_t now;
    uint32_t last_control_ms = 0;
    uint32_t last_telemetry_ms = 0;
    uint32_t last_oled_ms = 0;
    ProtocolCommand_t cmd;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SysTick_BspInit();
    USART1_BspInit(RK_USART_BAUDRATE);
    watchdog_init();
    Motor_BspInit();
    Encoder_BspInit();
    Protocol_Init();
    Motion_Init();
    watchdog_feed();
    OLED_BspInit();
    update_oled();
    watchdog_feed();
    now = SysTick_Millis();
    last_control_ms = now;
    last_telemetry_ms = now;
    last_oled_ms = now;

    while (1) {
        now = SysTick_Millis();

        if (Protocol_GetCommand(&cmd)) {
            process_command(&cmd, now);
        }

        if ((uint32_t)(now - last_control_ms) >= CONTROL_PERIOD_MS) {
            last_control_ms = now;
            Motion_Update(now);
        }

        if ((uint32_t)(now - last_telemetry_ms) >= TELEMETRY_PERIOD_MS) {
            last_telemetry_ms = now;
            send_telemetry();
        }

        if ((uint32_t)(now - last_oled_ms) >= OLED_REFRESH_PERIOD_MS) {
            last_oled_ms = now;
            update_oled();
        }

        watchdog_feed();
    }
}

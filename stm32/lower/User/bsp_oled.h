#ifndef BSP_OLED_H
#define BSP_OLED_H

#include "motion_control.h"
#include "protocol.h"

void OLED_BspInit(void);
void OLED_Clear(void);
void OLED_ShowTelemetry(const MotionTelemetry_t *tel, const ProtocolDiagnostics_t *diag);

#endif

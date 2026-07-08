#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "stm32f10x.h"
#include <stdint.h>

#define PROTO_HEAD1                 0xAA
#define PROTO_HEAD2                 0x55
#define PROTO_MAX_PAYLOAD           16
#define PROTO_CMD_QUEUE_SIZE        4

#define CMD_SET_VELOCITY            0x01
#define CMD_SET_GOAL                0x02
#define CMD_STOP                    0x03
#define CMD_EMERGENCY_STOP          0x04
#define CMD_CLEAR_EMERGENCY         0x05

#define TEL_STATUS                  0x81

typedef enum {
    PROTO_CMD_NONE = 0,
    PROTO_CMD_VELOCITY,
    PROTO_CMD_GOAL,
    PROTO_CMD_STOP,
    PROTO_CMD_EMERGENCY_STOP,
    PROTO_CMD_CLEAR_EMERGENCY
} ProtocolCmdType_t;

typedef struct {
    ProtocolCmdType_t type;
    int16_t v_mm_s;
    int16_t w_mrad_s;
    int16_t x_mm;
    int16_t y_mm;
    uint16_t arrive_ms;
} ProtocolCommand_t;

typedef struct {
    int16_t left_mm_s;
    int16_t right_mm_s;
    int16_t x_mm;
    int16_t y_mm;
    int16_t theta_mrad;
    uint16_t battery_mv;
    uint8_t mode;
    uint8_t stall_flags;
} ProtocolTelemetry_t;

typedef struct {
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint32_t length_errors;
    uint32_t queue_drops;
    uint32_t tx_drops;
    uint32_t head1_seen;
    uint32_t bad_head2;
    uint8_t last_frame_type;
    uint8_t last_frame_len;
    uint8_t last_cmd_type;
    uint8_t last_rx_byte;
} ProtocolDiagnostics_t;

void Protocol_Init(void);
void Protocol_FeedByte(uint8_t byte);
uint8_t Protocol_GetCommand(ProtocolCommand_t *cmd);
void Protocol_SendTelemetry(const ProtocolTelemetry_t *tel);
void Protocol_GetDiagnostics(ProtocolDiagnostics_t *diag);

#endif

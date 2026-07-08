#include "protocol.h"
#include "bsp_usart.h"

#define TEL_STATUS_PAYLOAD_LEN      15U
#define TEL_STATUS_FRAME_LEN        (2U + 1U + 1U + TEL_STATUS_PAYLOAD_LEN + 1U)

typedef enum {
    RX_WAIT_HEAD1 = 0,
    RX_WAIT_HEAD2,
    RX_TYPE,
    RX_LEN,
    RX_PAYLOAD,
    RX_CHECKSUM
} RxState_t;

static volatile ProtocolCommand_t g_cmd_queue[PROTO_CMD_QUEUE_SIZE];
static volatile uint8_t g_cmd_head = 0;
static volatile uint8_t g_cmd_tail = 0;
static volatile uint8_t g_cmd_count = 0;

static RxState_t g_rx_state;
static uint8_t g_rx_type;
static uint8_t g_rx_len;
static uint8_t g_rx_index;
static uint8_t g_rx_checksum;
static uint8_t g_payload[PROTO_MAX_PAYLOAD];
static volatile ProtocolDiagnostics_t g_diag;

static int16_t get_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t get_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void put_i16_le(uint8_t *p, int16_t v)
{
    put_u16_le(p, (uint16_t)v);
}

static void rx_resync_from_byte(uint8_t byte)
{
    if (byte == PROTO_HEAD1) {
        g_diag.head1_seen++;
        g_rx_state = RX_WAIT_HEAD2;
    } else {
        g_rx_state = RX_WAIT_HEAD1;
    }
}

static void publish_command(const ProtocolCommand_t *cmd)
{
    uint8_t next_tail;

    if (cmd->type == PROTO_CMD_STOP || cmd->type == PROTO_CMD_EMERGENCY_STOP ||
        cmd->type == PROTO_CMD_CLEAR_EMERGENCY) {
        g_cmd_head = 0;
        g_cmd_tail = 0;
        g_cmd_count = 0;
    }

    if (g_cmd_count >= PROTO_CMD_QUEUE_SIZE) {
        g_cmd_head = (uint8_t)((g_cmd_head + 1U) % PROTO_CMD_QUEUE_SIZE);
        g_cmd_count--;
        g_diag.queue_drops++;
    }

    next_tail = (uint8_t)((g_cmd_tail + 1U) % PROTO_CMD_QUEUE_SIZE);
    g_cmd_queue[g_cmd_tail] = *cmd;
    g_cmd_tail = next_tail;
    g_cmd_count++;
    g_diag.last_cmd_type = (uint8_t)cmd->type;
}

static void decode_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    ProtocolCommand_t cmd;

    cmd.type = PROTO_CMD_NONE;
    cmd.v_mm_s = 0;
    cmd.w_mrad_s = 0;
    cmd.x_mm = 0;
    cmd.y_mm = 0;
    cmd.arrive_ms = 0;

    if (type == CMD_SET_VELOCITY && len == 4) {
        cmd.type = PROTO_CMD_VELOCITY;
        cmd.v_mm_s = get_i16_le(&payload[0]);
        cmd.w_mrad_s = get_i16_le(&payload[2]);
        publish_command(&cmd);
    } else if (type == CMD_SET_GOAL && len == 6) {
        cmd.type = PROTO_CMD_GOAL;
        cmd.x_mm = get_i16_le(&payload[0]);
        cmd.y_mm = get_i16_le(&payload[2]);
        cmd.arrive_ms = get_u16_le(&payload[4]);
        publish_command(&cmd);
    } else if (type == CMD_STOP && len == 0) {
        cmd.type = PROTO_CMD_STOP;
        publish_command(&cmd);
    } else if (type == CMD_EMERGENCY_STOP && len == 0) {
        cmd.type = PROTO_CMD_EMERGENCY_STOP;
        publish_command(&cmd);
    } else if (type == CMD_CLEAR_EMERGENCY && len == 0) {
        cmd.type = PROTO_CMD_CLEAR_EMERGENCY;
        publish_command(&cmd);
    }
}

void Protocol_Init(void)
{
    g_rx_state = RX_WAIT_HEAD1;
    g_cmd_head = 0;
    g_cmd_tail = 0;
    g_cmd_count = 0;
    g_diag.rx_bytes = 0U;
    g_diag.valid_frames = 0U;
    g_diag.checksum_errors = 0U;
    g_diag.length_errors = 0U;
    g_diag.queue_drops = 0U;
    g_diag.tx_drops = 0U;
    g_diag.head1_seen = 0U;
    g_diag.bad_head2 = 0U;
    g_diag.last_frame_type = 0U;
    g_diag.last_frame_len = 0U;
    g_diag.last_cmd_type = 0U;
    g_diag.last_rx_byte = 0U;
}

void Protocol_FeedByte(uint8_t byte)
{
    g_diag.rx_bytes++;
    g_diag.last_rx_byte = byte;

    switch (g_rx_state) {
    case RX_WAIT_HEAD1:
        if (byte == PROTO_HEAD1) {
            rx_resync_from_byte(byte);
        }
        break;

    case RX_WAIT_HEAD2:
        if (byte == PROTO_HEAD2) {
            g_rx_state = RX_TYPE;
        } else {
            g_diag.bad_head2++;
            rx_resync_from_byte(byte);
        }
        break;

    case RX_TYPE:
        g_rx_type = byte;
        g_rx_checksum = byte;
        g_rx_state = RX_LEN;
        break;

    case RX_LEN:
        g_rx_len = byte;
        g_rx_checksum += byte;
        g_rx_index = 0;
        if (g_rx_len > PROTO_MAX_PAYLOAD) {
            g_diag.length_errors++;
            rx_resync_from_byte(byte);
        } else {
            g_rx_state = (g_rx_len == 0) ? RX_CHECKSUM : RX_PAYLOAD;
        }
        break;

    case RX_PAYLOAD:
        g_payload[g_rx_index++] = byte;
        g_rx_checksum += byte;
        if (g_rx_index >= g_rx_len) {
            g_rx_state = RX_CHECKSUM;
        }
        break;

    case RX_CHECKSUM:
        if (byte == g_rx_checksum) {
            g_diag.valid_frames++;
            g_diag.last_frame_type = g_rx_type;
            g_diag.last_frame_len = g_rx_len;
            decode_frame(g_rx_type, g_payload, g_rx_len);
            g_rx_state = RX_WAIT_HEAD1;
        } else {
            g_diag.checksum_errors++;
            rx_resync_from_byte(byte);
        }
        break;

    default:
        g_rx_state = RX_WAIT_HEAD1;
        break;
    }
}

uint8_t Protocol_GetCommand(ProtocolCommand_t *cmd)
{
    uint8_t ret = 0;

    __disable_irq();
    if (g_cmd_count > 0U) {
        *cmd = g_cmd_queue[g_cmd_head];
        g_cmd_head = (uint8_t)((g_cmd_head + 1U) % PROTO_CMD_QUEUE_SIZE);
        g_cmd_count--;
        ret = 1;
    }
    __enable_irq();

    return ret;
}

void Protocol_GetDiagnostics(ProtocolDiagnostics_t *diag)
{
    if (diag == 0) {
        return;
    }

    __disable_irq();
    diag->rx_bytes = g_diag.rx_bytes;
    diag->valid_frames = g_diag.valid_frames;
    diag->checksum_errors = g_diag.checksum_errors;
    diag->length_errors = g_diag.length_errors;
    diag->queue_drops = g_diag.queue_drops;
    diag->tx_drops = g_diag.tx_drops;
    diag->head1_seen = g_diag.head1_seen;
    diag->bad_head2 = g_diag.bad_head2;
    diag->last_frame_type = g_diag.last_frame_type;
    diag->last_frame_len = g_diag.last_frame_len;
    diag->last_cmd_type = g_diag.last_cmd_type;
    diag->last_rx_byte = g_diag.last_rx_byte;
    __enable_irq();
}

void Protocol_SendTelemetry(const ProtocolTelemetry_t *tel)
{
    uint8_t frame[TEL_STATUS_FRAME_LEN];
    uint8_t i;
    uint8_t checksum;

    frame[0] = PROTO_HEAD1;
    frame[1] = PROTO_HEAD2;
    frame[2] = TEL_STATUS;
    frame[3] = TEL_STATUS_PAYLOAD_LEN;
    put_i16_le(&frame[4], tel->left_mm_s);
    put_i16_le(&frame[6], tel->right_mm_s);
    put_i16_le(&frame[8], tel->x_mm);
    put_i16_le(&frame[10], tel->y_mm);
    put_i16_le(&frame[12], tel->theta_mrad);
    put_u16_le(&frame[14], tel->battery_mv);
    frame[16] = tel->mode;
    frame[17] = tel->stall_flags;
    frame[18] = 0;

    checksum = 0;
    for (i = 2; i < (TEL_STATUS_FRAME_LEN - 1U); i++) {
        checksum += frame[i];
    }
    frame[TEL_STATUS_FRAME_LEN - 1U] = checksum;

    if (USART1_SendBuffer(frame, sizeof(frame)) == 0U) {
        g_diag.tx_drops++;
    }
}

#include "bsp_oled.h"
#include "app_config.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include <stdint.h>

#define OLED_PORT       GPIOB
#define OLED_SCL_PIN    GPIO_Pin_10
#define OLED_SDA_PIN    GPIO_Pin_11
#define OLED_WIDTH      128U
#define OLED_PAGES      8U
#define OLED_CHARS      21U

#if OLED_ENABLE
static uint8_t g_oled_ready = 0;

static void oled_delay(void)
{
    volatile uint16_t i;

    for (i = 0; i < OLED_I2C_DELAY_CYCLES; i++) {
    }
}

static void scl_high(void)
{
    GPIO_SetBits(OLED_PORT, OLED_SCL_PIN);
    oled_delay();
}

static void scl_low(void)
{
    GPIO_ResetBits(OLED_PORT, OLED_SCL_PIN);
    oled_delay();
}

static void sda_high(void)
{
    GPIO_SetBits(OLED_PORT, OLED_SDA_PIN);
    oled_delay();
}

static void sda_low(void)
{
    GPIO_ResetBits(OLED_PORT, OLED_SDA_PIN);
    oled_delay();
}

static void i2c_start(void)
{
    sda_high();
    scl_high();
    sda_low();
    scl_low();
}

static void i2c_stop(void)
{
    sda_low();
    scl_high();
    sda_high();
}

static uint8_t i2c_write_byte(uint8_t byte)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0; i < 8U; i++) {
        if ((byte & 0x80U) != 0U) {
            sda_high();
        } else {
            sda_low();
        }
        scl_high();
        scl_low();
        byte <<= 1;
    }

    sda_high();
    scl_high();
    ack = (GPIO_ReadInputDataBit(OLED_PORT, OLED_SDA_PIN) == Bit_RESET) ? 1U : 0U;
    scl_low();
    return ack;
}

static uint8_t oled_begin_write(uint8_t control)
{
    uint8_t ack_addr;
    uint8_t ack_control;

    i2c_start();
    ack_addr = i2c_write_byte((uint8_t)(OLED_I2C_ADDR << 1));
    ack_control = i2c_write_byte(control);
    return (ack_addr != 0U && ack_control != 0U) ? 1U : 0U;
}

static void oled_write_cmd(uint8_t cmd)
{
    if (oled_begin_write(0x00U) != 0U) {
        i2c_write_byte(cmd);
    }
    i2c_stop();
}

static void oled_set_pos(uint8_t page, uint8_t col)
{
    oled_write_cmd((uint8_t)(0xB0U + page));
    oled_write_cmd((uint8_t)(0x00U + (col & 0x0FU)));
    oled_write_cmd((uint8_t)(0x10U + (col >> 4)));
}

static uint8_t oled_begin_data(void)
{
    return oled_begin_write(0x40U);
}

static const uint8_t *font5x7(char c)
{
    static const uint8_t blank[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t dash[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
    static const uint8_t colon[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
    static const uint8_t dot[5] = {0x00U, 0x60U, 0x60U, 0x00U, 0x00U};
    static const uint8_t slash[5] = {0x20U, 0x10U, 0x08U, 0x04U, 0x02U};
    static const uint8_t digits[10][5] = {
        {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
        {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
        {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
        {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
        {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
        {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
        {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
        {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
        {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
        {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}
    };
    static const uint8_t letters[26][5] = {
        {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU},
        {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U},
        {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U},
        {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU},
        {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U},
        {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U},
        {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU},
        {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU},
        {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U},
        {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U},
        {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U},
        {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U},
        {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU},
        {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU},
        {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU},
        {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U},
        {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU},
        {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U},
        {0x46U, 0x49U, 0x49U, 0x49U, 0x31U},
        {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U},
        {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU},
        {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU},
        {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU},
        {0x63U, 0x14U, 0x08U, 0x14U, 0x63U},
        {0x07U, 0x08U, 0x70U, 0x08U, 0x07U},
        {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}
    };

    if (c >= 'a' && c <= 'z') {
        c = (char)(c - ('a' - 'A'));
    }
    if (c >= '0' && c <= '9') {
        return digits[c - '0'];
    }
    if (c >= 'A' && c <= 'Z') {
        return letters[c - 'A'];
    }
    if (c == '-') {
        return dash;
    }
    if (c == ':') {
        return colon;
    }
    if (c == '.') {
        return dot;
    }
    if (c == '/') {
        return slash;
    }
    return blank;
}

static void oled_write_char_stream(char c)
{
    const uint8_t *font = font5x7(c);
    uint8_t i;

    for (i = 0; i < 5U; i++) {
        i2c_write_byte(font[i]);
    }
    i2c_write_byte(0x00U);
}

static void oled_write_line(uint8_t row, const char *text)
{
    uint8_t i;

    if (g_oled_ready == 0U || row >= OLED_PAGES) {
        return;
    }

    oled_set_pos(row, 0U);
    if (oled_begin_data() == 0U) {
        i2c_stop();
        return;
    }
    for (i = 0; i < OLED_CHARS; i++) {
        if (text[i] != '\0') {
            oled_write_char_stream(text[i]);
        } else {
            oled_write_char_stream(' ');
        }
    }
    i2c_stop();
}

static char *line_put_char(char *p, char *end, char c)
{
    if (p < end) {
        *p++ = c;
    }
    return p;
}

static char *line_put_str(char *p, char *end, const char *s)
{
    while (*s != '\0' && p < end) {
        *p++ = *s++;
    }
    return p;
}

static char *line_put_u32(char *p, char *end, uint32_t v)
{
    char tmp[10];
    uint8_t n = 0U;

    do {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v > 0U && n < sizeof(tmp));

    while (n > 0U && p < end) {
        *p++ = tmp[--n];
    }
    return p;
}

static char *line_put_i32(char *p, char *end, int32_t v)
{
    if (v < 0) {
        p = line_put_char(p, end, '-');
        v = -v;
    }
    return line_put_u32(p, end, (uint32_t)v);
}

static void line_finish(char *line, char *p, char *end)
{
    while (p < end) {
        *p++ = ' ';
    }
    *end = '\0';
}

static const char *mode_text(MotionMode_t mode)
{
    switch (mode) {
    case MOTION_MODE_VELOCITY:
        return "VEL";
    case MOTION_MODE_GOAL:
        return "GOAL";
    case MOTION_MODE_EMERGENCY:
        return "EMRG";
    case MOTION_MODE_STALL:
        return "STALL";
    case MOTION_MODE_STOP:
    default:
        return "STOP";
    }
}

void OLED_BspInit(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(OLED_PORT, &gpio);

    sda_high();
    scl_high();

    if (oled_begin_write(0x00U) == 0U) {
        i2c_stop();
        g_oled_ready = 0U;
        return;
    }
    i2c_stop();
    g_oled_ready = 1U;

    oled_write_cmd(0xAEU);
    oled_write_cmd(0xD5U);
    oled_write_cmd(0x80U);
    oled_write_cmd(0xA8U);
    oled_write_cmd(0x3FU);
    oled_write_cmd(0xD3U);
    oled_write_cmd(0x00U);
    oled_write_cmd(0x40U);
    oled_write_cmd(0x8DU);
    oled_write_cmd(0x14U);
    oled_write_cmd(0x20U);
    oled_write_cmd(0x00U);
    oled_write_cmd(0xA1U);
    oled_write_cmd(0xC8U);
    oled_write_cmd(0xDAU);
    oled_write_cmd(0x12U);
    oled_write_cmd(0x81U);
    oled_write_cmd(0x7FU);
    oled_write_cmd(0xD9U);
    oled_write_cmd(0xF1U);
    oled_write_cmd(0xDBU);
    oled_write_cmd(0x40U);
    oled_write_cmd(0xA4U);
    oled_write_cmd(0xA6U);
    oled_write_cmd(0x2EU);
    oled_write_cmd(0xAFU);

    OLED_Clear();
    oled_write_line(0U, "SMART BIN LOWER");
    oled_write_line(1U, "OLED READY");
}

void OLED_Clear(void)
{
    uint8_t page;
    uint8_t col;

    if (g_oled_ready == 0U) {
        return;
    }

    for (page = 0U; page < OLED_PAGES; page++) {
        oled_set_pos(page, 0U);
        if (oled_begin_data() == 0U) {
            i2c_stop();
            return;
        }
        for (col = 0U; col < OLED_WIDTH; col++) {
            i2c_write_byte(0x00U);
        }
        i2c_stop();
    }
}

void OLED_ShowTelemetry(const MotionTelemetry_t *tel, const ProtocolDiagnostics_t *diag)
{
    static uint32_t s_oled_updates = 0U;
    char line[OLED_CHARS + 1U];
    char *p;
    char *end = &line[OLED_CHARS];
    uint32_t rx_errors = 0U;

    if (g_oled_ready == 0U || tel == 0) {
        return;
    }

    if (diag != 0) {
        rx_errors = (uint32_t)diag->checksum_errors + (uint32_t)diag->length_errors;
    }

    s_oled_updates++;

    p = line;
    p = line_put_str(p, end, "UPD:");
    p = line_put_u32(p, end, s_oled_updates);
    p = line_put_str(p, end, " M:");
    p = line_put_str(p, end, mode_text(tel->mode));
    if (tel->stall_flags != 0U) {
        p = line_put_str(p, end, " S:");
        p = line_put_u32(p, end, tel->stall_flags);
    }
    line_finish(line, p, end);
    oled_write_line(0U, line);

    p = line;
    p = line_put_str(p, end, "L:");
    p = line_put_i32(p, end, (int32_t)tel->left_speed_mm_s);
    p = line_put_str(p, end, " R:");
    p = line_put_i32(p, end, (int32_t)tel->right_speed_mm_s);
    line_finish(line, p, end);
    oled_write_line(1U, line);

    p = line;
    p = line_put_str(p, end, "X:");
    p = line_put_i32(p, end, (int32_t)tel->x_mm);
    p = line_put_str(p, end, " Y:");
    p = line_put_i32(p, end, (int32_t)tel->y_mm);
    line_finish(line, p, end);
    oled_write_line(2U, line);

    p = line;
    p = line_put_str(p, end, "RX:");
    p = line_put_u32(p, end, (diag != 0) ? diag->rx_bytes : 0U);
    p = line_put_str(p, end, " OK:");
    p = line_put_u32(p, end, (diag != 0) ? diag->valid_frames : 0U);
    p = line_put_str(p, end, " C:");
    p = line_put_u32(p, end, (diag != 0) ? diag->last_cmd_type : 0U);
    line_finish(line, p, end);
    oled_write_line(3U, line);

    p = line;
    p = line_put_str(p, end, "E:");
    p = line_put_u32(p, end, rx_errors);
    p = line_put_str(p, end, " Q:");
    p = line_put_u32(p, end, (diag != 0) ? diag->queue_drops : 0U);
    p = line_put_str(p, end, " T:");
    p = line_put_u32(p, end, (diag != 0) ? diag->tx_drops : 0U);
    p = line_put_str(p, end, " N:");
    p = line_put_u32(p, end, (diag != 0) ? diag->bad_head2 : 0U);
    line_finish(line, p, end);
    oled_write_line(4U, line);
}
#else
void OLED_BspInit(void)
{
}

void OLED_Clear(void)
{
}

void OLED_ShowTelemetry(const MotionTelemetry_t *tel, const ProtocolDiagnostics_t *diag)
{
    (void)tel;
    (void)diag;
}
#endif

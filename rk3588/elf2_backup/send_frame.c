/*
 * ELF2 RK3588 - GPIO Bit-Bang Frame Sender
 * Frame: 0xAA 0x55 type len payload[N] checksum
 * P6 = TX (GPIO125, gpio3-29, UART9_TX)
 * P8 = RX (GPIO124, gpio3-28, UART9_RX)
 * Compile: aarch64-linux-gnu-gcc -o send_frame send_frame.c
 * Run:     sudo ./send_frame
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>

#define TX_GPIO     125
#define RX_GPIO     124
#define BAUD_RATE   9600
#define FRAME_TYPE  0x01
#define PAYLOAD_LEN 8

#define GPIO_PATH   "/sys/class/gpio"
#define EXPORT      GPIO_PATH "/export"
#define UNEXPORT    GPIO_PATH "/unexport"

static const unsigned long BIT_DELAY_US = (1000000UL / BAUD_RATE);

static void gpio_export(int gpio)
{
    char buf[16], path[64];
    int fd, n;
    snprintf(path, sizeof(path), "%s/gpio%d", GPIO_PATH, gpio);
    if (access(path, F_OK) == 0) return;
    fd = open(EXPORT, O_WRONLY);
    if (fd < 0) { perror("export open"); return; }
    n = snprintf(buf, sizeof(buf), "%d", gpio);
    write(fd, buf, n);
    close(fd);
    usleep(100000);
}

static void gpio_unexport(int gpio)
{
    char buf[16];
    int fd, n;
    fd = open(UNEXPORT, O_WRONLY);
    if (fd < 0) return;
    n = snprintf(buf, sizeof(buf), "%d", gpio);
    write(fd, buf, n);
    close(fd);
}

static void gpio_set_direction(int gpio, const char *dir)
{
    char path[64];
    int fd;
    snprintf(path, sizeof(path), "%s/gpio%d/direction", GPIO_PATH, gpio);
    fd = open(path, O_WRONLY);
    if (fd < 0) { perror("direction open"); return; }
    write(fd, dir, strlen(dir));
    close(fd);
}

static void gpio_write_val(int gpio, int value)
{
    char path[64];
    int fd;
    snprintf(path, sizeof(path), "%s/gpio%d/value", GPIO_PATH, gpio);
    fd = open(path, O_WRONLY);
    if (fd < 0) { perror("value open"); return; }
    write(fd, value ? "1" : "0", 1);
    close(fd);
}

static void send_byte(int gpio, unsigned char byte)
{
    int i;
    gpio_write_val(gpio, 0);
    usleep(BIT_DELAY_US);
    for (i = 0; i < 8; i++) {
        gpio_write_val(gpio, (byte >> i) & 1);
        usleep(BIT_DELAY_US);
    }
    gpio_write_val(gpio, 1);
    usleep(BIT_DELAY_US);
}

static unsigned char calc_checksum(const unsigned char *data, int len)
{
    unsigned char c = 0;
    int i;
    for (i = 0; i < len; i++) c ^= data[i];
    return c;
}

int main(void)
{
    unsigned char frame[256];
    int frame_len, i;
    struct timespec start, end;
    double elapsed_ms, actual_baud;
    int total_bits;

    frame[0] = 0xAA;
    frame[1] = 0x55;
    frame[2] = FRAME_TYPE;
    frame[3] = PAYLOAD_LEN;

    srand((unsigned)time(NULL));
    for (i = 0; i < PAYLOAD_LEN; i++)
        frame[4 + i] = (unsigned char)(rand() & 0xFF);

    frame_len = 4 + PAYLOAD_LEN;
    frame[frame_len] = calc_checksum(frame, frame_len);
    frame_len++;
    total_bits = frame_len * 10;

    printf("============================================================\n");
    printf("  ELF2 GPIO Frame Sender (C, bit-bang)\n");
    printf("============================================================\n");
    printf("  Arch    : aarch64 (RK3588)\n");
    printf("  TX Pin  : GPIO %d (P6, UART9_TX, gpio3-29)\n", TX_GPIO);
    printf("  RX Pin  : GPIO %d (P8, UART9_RX, gpio3-28)\n", RX_GPIO);
    printf("  Baud    : %d bps\n", BAUD_RATE);
    printf("  Delay   : %lu us/bit\n", BIT_DELAY_US);
    printf("------------------------------------------------------------\n");
    printf("  Frame (%d bytes):\n", frame_len);
    printf("    [0] Header1  = 0x%02X\n", frame[0]);
    printf("    [1] Header2  = 0x%02X\n", frame[1]);
    printf("    [2] CmdType  = 0x%02X\n", frame[2]);
    printf("    [3] DataLen  = 0x%02X (%d)\n", frame[3], PAYLOAD_LEN);
    printf("    [4..%d] Payload =", 4 + PAYLOAD_LEN - 1);
    for (i = 0; i < PAYLOAD_LEN; i++)
        printf(" %02X", frame[4 + i]);
    printf("\n");
    printf("    [%d] Checksum = 0x%02X (XOR)\n", frame_len - 1, frame[frame_len - 1]);
    printf("------------------------------------------------------------\n");

    printf("  Exporting GPIOs...\n");
    gpio_export(TX_GPIO);
    gpio_set_direction(TX_GPIO, "out");
    gpio_write_val(TX_GPIO, 1);

    gpio_export(RX_GPIO);
    gpio_set_direction(RX_GPIO, "out");
    gpio_write_val(RX_GPIO, 1);

    usleep(5000);

    printf("  Sending %d bits...\n", total_bits);
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (i = 0; i < frame_len; i++)
        send_byte(TX_GPIO, frame[i]);

    gpio_write_val(TX_GPIO, 1);

    clock_gettime(CLOCK_MONOTONIC, &end);
    elapsed_ms = (end.tv_sec - start.tv_sec) * 1000.0 +
                 (end.tv_nsec - start.tv_nsec) / 1000000.0;
    actual_baud = (elapsed_ms > 0) ? (total_bits * 1000.0 / elapsed_ms) : 0;

    printf("  Done!\n");
    printf("  Time: %.1f ms, Actual baud: %.0f bps\n", elapsed_ms, actual_baud);
    printf("------------------------------------------------------------\n");

    printf("  Hex: ");
    for (i = 0; i < frame_len; i++)
        printf("%02X ", frame[i]);
    printf("\n============================================================\n");

    gpio_unexport(TX_GPIO);
    gpio_unexport(RX_GPIO);

    return 0;
}

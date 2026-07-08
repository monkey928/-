/*
 * ELF2 RK3588 - Hardware UART Frame Sender
 * UART9 (P6=TX gpio3-29, P8=RX gpio3-28) @ 115200 8N1
 * To STM32F103C8T6 PA9(TX)/PA10(RX)
 * Frame: AA 55 type len payload[N] checksum
 * Compile: aarch64-linux-gnu-gcc -o uart_send uart_send.c
 * Run:     sudo ./uart_send
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#define UART_DEV    "/dev/ttyS9"
#define BAUD_RATE   115200
#define FRAME_TYPE  0x01
#define PAYLOAD_LEN 8

static unsigned char calc_checksum(const unsigned char *data, int len)
{
    unsigned char c = 0;
    int i;
    for (i = 0; i < len; i++) c ^= data[i];
    return c;
}

static int uart_open(const char *dev, speed_t speed)
{
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror("open uart"); return -1; }

    struct termios opts;
    tcgetattr(fd, &opts);

    cfsetospeed(&opts, speed);
    cfsetispeed(&opts, speed);

    opts.c_cflag &= ~PARENB;
    opts.c_cflag &= ~CSTOPB;
    opts.c_cflag &= ~CSIZE;
    opts.c_cflag |= CS8;
    opts.c_cflag |= CREAD | CLOCAL;

    opts.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    opts.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    opts.c_oflag &= ~OPOST;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &opts);
    return fd;
}

int main(void)
{
    unsigned char frame[256];
    int frame_len, i, fd, written;
    unsigned char verify;

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

    printf("============================================================\n");
    printf("  ELF2 UART Frame Sender - Hardware UART9\n");
    printf("============================================================\n");
    printf("  Device  : %s\n", UART_DEV);
    printf("  Baud    : %d bps, 8N1\n", BAUD_RATE);
    printf("  TX Pin  : GPIO125 (P6) --> STM32 PA10(RX)\n");
    printf("  RX Pin  : GPIO124 (P8) --> STM32 PA9(TX)\n");
    printf("------------------------------------------------------------\n");
    printf("  Frame (%d bytes):\n", frame_len);
    printf("    [0] Header1  = 0x%02X\n", frame[0]);
    printf("    [1] Header2  = 0x%02X\n", frame[1]);
    printf("    [2] CmdType  = 0x%02X\n", frame[2]);
    printf("    [3] DataLen  = 0x%02X (%d)\n", frame[3], PAYLOAD_LEN);
    printf("    Payload   =");
    for (i = 0; i < PAYLOAD_LEN; i++) printf(" %02X", frame[4 + i]);
    printf("\n    [%d] Checksum = 0x%02X (XOR)\n", frame_len - 1, frame[frame_len - 1]);
    printf("------------------------------------------------------------\n");

    fd = uart_open(UART_DEV, B115200);
    if (fd < 0) {
        fprintf(stderr, "Failed to open UART\n");
        return 1;
    }
    printf("  UART opened OK\n");

    printf("  Sending %d bytes...\n", frame_len);
    written = write(fd, frame, frame_len);
    tcdrain(fd);

    if (written == frame_len)
        printf("  Done! %d bytes written.\n", written);
    else
        printf("  WARNING: wrote %d/%d bytes\n", written, frame_len);

    printf("------------------------------------------------------------\n");
    printf("  Hex: ");
    for (i = 0; i < frame_len; i++) printf("%02X ", frame[i]);
    printf("\n");

    verify = calc_checksum(frame, frame_len - 1);
    printf("  Verify checksum: 0x%02X %s\n", verify,
           verify == frame[frame_len - 1] ? "(OK)" : "(MISMATCH!)");
    printf("============================================================\n");

    close(fd);
    return 0;
}

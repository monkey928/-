/*
 * ELF2 UART Frame Sender - Custom sensor frame
 * Frame: AA 55 01 04 vL vH wL wH checksum
 * UART9 @ 115200 8N1  -->  STM32F103C8T6
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

static unsigned char calc_checksum(const unsigned char *data, int len)
{
    unsigned char c = 0;
    for (int i = 0; i < len; i++) c ^= data[i];
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
    unsigned char frame[32];
    int fd, written;

    /* Simulated sensor data */
    unsigned short voltage = 3230;   /* 3.230V -> 3230 mV */
    unsigned short weight  = 1547;   /* 1.547kg -> 1547 g  */

    /* Build frame: AA 55 01 04 vL vH wL wH checksum */
    frame[0] = 0xAA;           /* Header 1 */
    frame[1] = 0x55;           /* Header 2 */
    frame[2] = 0x01;           /* Command type */
    frame[3] = 0x04;           /* Data length = 4 */
    frame[4] = voltage & 0xFF; /* vL */
    frame[5] = (voltage >> 8) & 0xFF; /* vH */
    frame[6] = weight & 0xFF;  /* wL */
    frame[7] = (weight >> 8) & 0xFF;  /* wH */

    int frame_len = 8;
    frame[frame_len] = calc_checksum(frame, frame_len);
    frame_len++;  /* 9 bytes total */

    /* Print info */
    printf("============================================================\n");
    printf("  ELF2 UART Frame Sender - Sensor Data\n");
    printf("============================================================\n");
    printf("  Device  : /dev/ttyS9 @ 115200 8N1\n");
    printf("  P6(TX)  : GPIO125 --> STM32 PA10(RX)\n");
    printf("  P8(RX)  : GPIO124 --> STM32 PA9(TX)\n");
    printf("------------------------------------------------------------\n");
    printf("  Frame (%d bytes):\n", frame_len);
    printf("    [0] Header1  = 0x%02X\n", frame[0]);
    printf("    [1] Header2  = 0x%02X\n", frame[1]);
    printf("    [2] CmdType  = 0x%02X\n", frame[2]);
    printf("    [3] DataLen  = 0x%02X\n", frame[3]);
    printf("    [4] vL       = 0x%02X\n", frame[4]);
    printf("    [5] vH       = 0x%02X  (voltage = %u mV)\n", frame[5], voltage);
    printf("    [6] wL       = 0x%02X\n", frame[6]);
    printf("    [7] wH       = 0x%02X  (weight  = %u g)\n", frame[7], weight);
    printf("    [8] Checksum = 0x%02X (XOR)\n", frame[8]);
    printf("------------------------------------------------------------\n");

    fd = uart_open("/dev/ttyS9", B115200);
    if (fd < 0) return 1;
    printf("  UART opened OK\n");

    printf("  Sending...\n");
    written = write(fd, frame, frame_len);
    tcdrain(fd);

    printf("  Done! %d bytes written.\n", written);
    printf("------------------------------------------------------------\n");
    printf("  Hex: ");
    for (int i = 0; i < frame_len; i++) printf("%02X ", frame[i]);
    printf("\n");

    unsigned char verify = calc_checksum(frame, frame_len - 1);
    printf("  Verify checksum: 0x%02X %s\n", verify,
           verify == frame[frame_len - 1] ? "(OK)" : "(MISMATCH!)");
    printf("============================================================\n");

    close(fd);
    return 0;
}

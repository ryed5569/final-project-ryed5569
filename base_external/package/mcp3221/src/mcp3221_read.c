#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

static int read_mcp3221_raw12(int bus, int addr, uint16_t *raw12_out)
{
    char dev[32];
    snprintf(dev, sizeof(dev), "/dev/i2c-%d", bus);

    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "open(%s) failed: %s\n", dev, strerror(errno));
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        fprintf(stderr, "ioctl(I2C_SLAVE, 0x%02x) failed: %s\n", addr, strerror(errno));
        close(fd);
        return -1;
    }

    uint8_t buf[2] = {0};
    ssize_t n = read(fd, buf, 2);
    if (n != 2) {
        fprintf(stderr, "read(2) failed: %s\n", (n < 0) ? strerror(errno) : "short read");
        close(fd);
        return -1;
    }
    close(fd);

    uint16_t v16 = ((uint16_t)buf[0] << 8) | buf[1];
    *raw12_out = (v16 >> 4) & 0x0FFF; // observed packing on your target
    return 0;
}

static void msleep(unsigned ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

int main(int argc, char **argv)
{
    int bus = 1;
    int addr = 0x4d;
    double vref = 3.3;
    int loop = 0;
    unsigned period_ms = 100; // 10 Hz default

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--bus") && i + 1 < argc) bus = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--addr") && i + 1 < argc) addr = (int)strtol(argv[++i], NULL, 0);
        else if (!strcmp(argv[i], "--vref") && i + 1 < argc) vref = atof(argv[++i]);
        else if (!strcmp(argv[i], "--loop")) loop = 1;
        else if (!strcmp(argv[i], "--period-ms") && i + 1 < argc) period_ms = (unsigned)atoi(argv[++i]);
        else {
            fprintf(stderr,
                "Usage: %s [--bus N] [--addr 0x4d] [--vref 3.3] [--loop] [--period-ms 100]\n",
                argv[0]);
            return 2;
        }
    }

    do {
        uint16_t raw12 = 0;
        if (read_mcp3221_raw12(bus, addr, &raw12) != 0) return 1;

        // Match your analog assumptions: Vadc = raw * Vref / 4095 (per your notes style)
        double vadc = ((double)raw12 * vref) / 4095.0;

        printf("bus=%d addr=0x%02x raw12=%u vadc=%.6fV\n", bus, addr, raw12, vadc);

        if (!loop) break;
        msleep(period_ms);
    } while (1);

    return 0;
}

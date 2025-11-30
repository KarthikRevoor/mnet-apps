#include "lcd1602_i2c.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <string.h>

/* ----------------- LCD / PCF8574 wiring assumptions -----------------
 * Common mapping (check your backpack):
 *   P0 -> RS
 *   P1 -> RW
 *   P2 -> EN
 *   P3 -> Backlight
 *   P4..P7 -> D4..D7
 * ------------------------------------------------------------------ */

#define LCD_RS   0x01
#define LCD_RW   0x02
#define LCD_EN   0x04
#define LCD_BL   0x08   /* backlight bit */

#define LCD_CMD  0
#define LCD_CHR  1

static int lcd_fd = -1;
static uint8_t lcd_backlight = LCD_BL;

/* ----------------- Low-level I2C helpers ----------------- */

static int lcd_i2c_write(uint8_t data)
{
    if (lcd_fd < 0)
        return -1;

    ssize_t ret = write(lcd_fd, &data, 1);
    if (ret != 1)
        return -1;

    return 0;
}

static int lcd_pulse_enable(uint8_t data)
{
    if (lcd_i2c_write(data | LCD_EN) < 0)
        return -1;

    /* Enable pulse width: a few microseconds is enough */
    usleep(1);

    if (lcd_i2c_write(data & ~LCD_EN) < 0)
        return -1;

    /* Allow command to settle */
    usleep(50);
    return 0;
}

/* Write a single 4-bit nibble (upper 4 bits) plus control bits */
static int lcd_write4(uint8_t nibble, uint8_t mode)
{
    uint8_t data = 0;

    /* Upper nibble goes to D4..D7 (P4..P7) */
    data = (nibble & 0xF0);

    /* Attach RS if data (character) */
    if (mode == LCD_CHR)
        data |= LCD_RS;

    /* Keep backlight state */
    data |= lcd_backlight;

    if (lcd_pulse_enable(data) < 0)
        return -1;

    return 0;
}

/* Write full 8-bit value using two 4-bit transfers */
static int lcd_write_byte(uint8_t value, uint8_t mode)
{
    uint8_t high = value & 0xF0;
    uint8_t low  = (value << 4) & 0xF0;

    if (lcd_write4(high, mode) < 0)
        return -1;
    if (lcd_write4(low, mode) < 0)
        return -1;

    return 0;
}

/* ----------------- Public LCD API ----------------- */

int lcd_init(const char *i2c_dev, int addr)
{
    int fd;
    int ret;

    fd = open(i2c_dev, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "lcd_init: open(%s) failed: %s\n",
                i2c_dev, strerror(errno));
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        fprintf(stderr, "lcd_init: ioctl(I2C_SLAVE, 0x%02x) failed: %s\n",
                addr, strerror(errno));
        close(fd);
        return -1;
    }

    lcd_fd = fd;

    /* Initialisation sequence for HD44780 in 4-bit mode */
    usleep(50000); /* wait >40ms after power-up */

    /* three times 0x30 (8-bit mode) */
    lcd_write4(0x30, LCD_CMD);
    usleep(4500);
    lcd_write4(0x30, LCD_CMD);
    usleep(4500);
    lcd_write4(0x30, LCD_CMD);
    usleep(150);

    /* switch to 4-bit mode */
    lcd_write4(0x20, LCD_CMD);
    usleep(150);

    /* function set: 4-bit, 2 lines, 5x8 font */
    lcd_write_byte(0x28, LCD_CMD);

    /* display off */
    lcd_write_byte(0x08, LCD_CMD);

    /* clear display */
    lcd_write_byte(0x01, LCD_CMD);
    usleep(2000);

    /* entry mode: increment, no shift */
    lcd_write_byte(0x06, LCD_CMD);

    /* display on, cursor off, blink off */
    lcd_write_byte(0x0C, LCD_CMD);

    return 0;
}

void lcd_clear(void)
{
    if (lcd_fd < 0)
        return;
    lcd_write_byte(0x01, LCD_CMD);
    usleep(2000);
}

void lcd_set_cursor(int row, int col)
{
    uint8_t addr;

    if (row < 0) row = 0;
    if (row > 1) row = 1;
    if (col < 0) col = 0;
    if (col > 15) col = 15;

    /* Line addresses: 0x00 and 0x40 for 16x2 */
    addr = (row == 0) ? (0x00 + col) : (0x40 + col);

    lcd_write_byte(0x80 | addr, LCD_CMD);
}

void lcd_print(const char *s)
{
    if (!s || lcd_fd < 0)
        return;

    while (*s) {
        if (*s == '\n') {
            /* crude: jump to start of second line */
            lcd_set_cursor(1, 0);
        } else {
            lcd_write_byte((uint8_t)*s, LCD_CHR);
        }
        s++;
    }
}

void lcd_print_line(int row, const char *s)
{
    int i;

    lcd_set_cursor(row, 0);

    /* Write up to 16 chars, pad with spaces if needed */
    for (i = 0; i < 16; i++) {
        char c = ' ';
        if (s && s[i])
            c = s[i];
        lcd_write_byte((uint8_t)c, LCD_CHR);
    }
}

/* ----------------- Utility: read integer from file ----------------- */

static int read_int_from_file(const char *path, int *out)
{
    FILE *f;
    char buf[64];

    if (!out)
        return -1;

    f = fopen(path, "r");
    if (!f)
        return -1;

    if (!fgets(buf, sizeof(buf), f)) {
        fclose(f);
        return -1;
    }
    fclose(f);

    if (sscanf(buf, "%d", out) != 1)
        return -1;

    return 0;
}

/* ----------------- Main: glue BME + mnet → LCD ----------------- */

int main(void)
{
    const char *i2c_dev = "/dev/i2c-1";
    const int   lcd_addr = 0x27;   // or 0x3F if that’s your module
    int ret = lcd_init(i2c_dev, lcd_addr);

    if (ret < 0) {
        fprintf(stderr, "lcd_1602: lcd_init failed\n");
        return 1;
    }

    lcd_clear();
    lcd_print_line(0, "LCD 1602 ONLINE");
    lcd_print_line(1, "Hello, Karthik!");

    while (1)
        sleep(1);
}


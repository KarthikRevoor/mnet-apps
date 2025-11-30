#ifndef LCD1602_I2C_H
#define LCD1602_I2C_H

int lcd1602_init(const char *i2c_dev, int addr);
void lcd1602_clear(void);
void lcd1602_set_cursor(int row, int col);
void lcd1602_print(const char *s);
void lcd1602_print_line(int row, const char *s);

#endif


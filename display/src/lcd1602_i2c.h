#ifndef LCD_1602_H
#define LCD_1602_H

int lcd_init(const char *i2c_dev, int addr);
void lcd_clear(void);
void lcd_set_cursor(int row, int col);
void lcd_print(const char *s);
void lcd_print_line(int row, const char *s);

#endif


#pragma once
/*
 * tty
 */

#define TTY_CMD 0x00 // TTY command extension
#define LCD_CMD 0x01 // LCD command for ST7789V2
#define LCD_COLOR 0x02 // specify text color 

void tty_init(void);
void tty_write(void const *buf, size_t len);
void tty_lcd_sleep(bool on);
void tty_lcd_color(uint8_t color);

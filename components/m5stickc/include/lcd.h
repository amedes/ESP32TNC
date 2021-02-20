#pragma once
#include "driver/spi_master.h"

#define LCD_TEXT_WIDTH  30
#define LCD_TEXT_HEIGHT 17
#define LCD_VRAM_SIZE (LCD_TEXT_WIDTH * LCD_TEXT_HEIGHT)

//#define LCD_OFF_TIME_VBUS 10
//#define LCD_OFF_TIME_BATT 2

struct LCD {
    spi_device_handle_t spi;
    uint16_t vram[LCD_VRAM_SIZE];
    uint8_t cursor_x;
    uint8_t cursor_y;
    uint8_t color; // 4bit text color, foreground(lower 4bit) and background(upper 4bit)
    uint8_t bcolor; // this color is used for clear and scroll
    uint8_t sleep; // state of sleep mode
};

extern struct LCD lcd;

void lcd_init(spi_device_handle_t spi, int lcd_rst_pin);
uint32_t lcd_get_id(spi_device_handle_t spi);
void lcd_text_init(spi_device_handle_t spi);
void lcd_print(uint8_t const *buf, int n);
void lcd_print_lf(void);
void lcd_sleep(uint8_t on);
void lcd_set_color(uint8_t color);
uint8_t lcd_get_color(void);
void lcd_madctl(uint8_t param);
void lcd_write_bytes(void const *buf, int len);
//void lcd_putchar(uint8_t c);
void lcd_cmd(const uint8_t cmd);
void lcd_data(const uint8_t *data, int len);

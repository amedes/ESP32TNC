/*
 * LCD packet
 *
 * print out a packet to M5StickC LCD screen
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"

#include "m5stickc.h"

#define TAG "lcdp"

#define AX25_ADDR_LEN 7
#define AX25_MIN_PKT_SIZE (AX25_ADDR_LEN * 2 + 1 + 2) // src addr + dst addr + Control + FCS
#define AX25_SSID_MASK 0x0f

#define BUF_SIZE 16

#define LCD_WHITE 0x1f
#define LCD_YELLOW 0x1e
#define LCD_INV_RED 0xc0
#define LCD_INV_GREEN 0xa0
#define LCD_INV_CYAN 0xb0

static char buf[BUF_SIZE];

static void lcd_packet_putchar(int c)
{
    uint8_t ch = c;

    if (ch >= ' ' && ch <= '~') {
	tty_write(&ch, sizeof(ch));
    } else {
	tty_lcd_color(LCD_YELLOW);
	tty_write(buf, snprintf(buf, BUF_SIZE, "<%02x>", ch));
	tty_lcd_color(LCD_WHITE);
    }
}

static void lcd_put_ssid(int ssid)
{
    int n = ssid & AX25_SSID_MASK;

    if (n > 0) tty_write(buf, snprintf(buf, BUF_SIZE, "-%d", n));
}

#if 0
static void lcd_put_number(int num)
{
    tty_write(buf, snprintf(buf, BUF_SIZE, "%d", num));
}
#endif

void lcd_dump_packet(uint8_t *item[2], size_t size[2])
{
    int i;
    int in_addr = true;
    int len = size[0];
    static const uint8_t addr_color[3] = {
	LCD_INV_RED,	// destination
	LCD_INV_GREEN,	// source
	LCD_INV_CYAN,	// repeater
    };
    int n = 0; // count address
   
    if (item[1] != NULL) len += size[1];

    if (len < AX25_MIN_PKT_SIZE) return;

    tty_lcd_color(addr_color[0]); // destination address

    for (i = 1; i < len; i++) {
	int c;

	c = (i < size[0]) ? item[0][i] : item[1][i - size[0]];

	if (in_addr) {
	    if (c & 1) in_addr = false;
	    c >>= 1; // for addr field shift

	    if ((i - 1) % AX25_ADDR_LEN == AX25_ADDR_LEN - 1) { // SSID
		lcd_put_ssid(c);
		lcd_packet_putchar(in_addr ? ',' : ':');

		// change color for src/repeater addr
		if (++n > 2) n = 2;
		tty_lcd_color(addr_color[n]);

	    } else {
		if (c != ' ') lcd_packet_putchar(c);
	    }

	    if (!in_addr) tty_lcd_color(LCD_WHITE); // restore default color

	} else {
	    lcd_packet_putchar(c);
	}
    }
    tty_write("\n", 1);
}

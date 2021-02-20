/*
 * tty
 *
 * emulate teletype terminal
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>

#include "esp_log.h"

#include "lcd.h"
#include "tty.h"
#include "axp192.h"

#define TTY_RINGBUFF_LEN 2048

#define TAG "tty"

static RingbufHandle_t tty_rb;


#define getc_rb(i, buf, buf_size, size) ((i < size) ? ((i < item_size[0]) ? buf[0][i] : buf[1][i - buf_size[0]]) : -1)

void tty_lcd_cmd(uint8_t *item, size_t size)
{
    int i = 1;

    if (i >= size) return;

    int cmd;
    int c = item[i++];

    switch (c) {
	case LCD_CMD:
	    cmd = item[i++];

	    if (cmd >= 0) lcd_cmd(cmd);

	    if (size > i) {
		lcd_data(&item[i], size - i);
	    }
	    break;

	case LCD_COLOR:
	    cmd = item[i++];

	    lcd_set_color(cmd);
	    
	    break;
    }
}

void tty_task(void *arg)
{
    size_t item_size;
    uint8_t *item;

    while (1) {
	item = xRingbufferReceive(tty_rb, &item_size, portMAX_DELAY);
	if (item == NULL) {
	    ESP_LOGW(TAG, "xRingbuffReceive() fail");
	    continue;
	}

	if (item[0]) { // print text

	    lcd_write_bytes(item, item_size);

	} else { // process lcd command

	    tty_lcd_cmd(item, item_size);

	}

	vRingbufferReturnItem(tty_rb, item);
    }
}

void tty_write(void const *buf, size_t len)
{
    if (xRingbufferSend(tty_rb, buf, len, portMAX_DELAY) != pdTRUE) {
	ESP_LOGW(TAG, "xRingbufferSend() fail");
    }
}

void tty_init(void)
{
    tty_rb = xRingbufferCreate(TTY_RINGBUFF_LEN, RINGBUF_TYPE_NOSPLIT);
    assert(tty_rb != NULL);

    assert(xTaskCreatePinnedToCore(tty_task, "tty task", 4096, NULL, tskIDLE_PRIORITY, NULL, tskNO_AFFINITY) == pdPASS);
}


void tty_lcd_sleep(bool on)
{
    uint8_t cmd[3] = { TTY_CMD, LCD_CMD, 0 };
    cmd[2] = on ? 0x10 : 0x11;

    tty_write(cmd, sizeof(cmd));

    if (on) {
	axp192_writeReg(0x12, axp192_readReg(0x12) & 0xfb);
    } else {
	axp192_writeReg(0x12, axp192_readReg(0x12) | 0x04);
    }
}

void tty_lcd_color(uint8_t color)
{
    uint8_t cmd[3] = { TTY_CMD, LCD_COLOR, color };

    tty_write(cmd, sizeof(cmd));
}

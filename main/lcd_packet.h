#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>

void lcd_dump_packet(uint8_t *item[2], size_t size[2]);

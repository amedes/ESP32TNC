#pragma once
/*
 * RGB LED library for M5ATOM
 */
#include "driver/rmt.h"

#define M5ATOM_RMT_TX_CH RMT_CHANNEL_7
#define M5ATOM_LED_BRIGHTNESS 3

#define M5ATOM_LED_GREEN 0
#define M5ATOM_LED_RED 1
#define M5ATOM_LED_BLUE 2
#define M5ATOM_LED_COLOR_MAX 3

extern const rmt_item32_t m5atom_led_bit[];

/*
 * initialize M5ATOM LED routine
 */
void m5atom_led_init(void);

/*
 * on/off M5ATOM RGB LED independently
 *
 * c: color, 0: green, 1: red, 2: blue
 * level: 1: on, 0: off
 */
static inline void m5atom_led_set_level(uint8_t c, uint8_t level)
{
    if (c >= M5ATOM_LED_COLOR_MAX) return;

    RMTMEM.chan[M5ATOM_RMT_TX_CH].data32[c * 8 + M5ATOM_LED_BRIGHTNESS] = m5atom_led_bit[level != 0];
}

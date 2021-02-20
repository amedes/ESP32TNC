/*
 * RGB LED library for M5ATOM
 */
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/gpio.h"

#include "m5atom.h"

#define M5ATOM_LED_GPIO GPIO_NUM_27
//#define M5ATOM_RMT_TX_CH RMT_CHANNEL_7
//#define M5ATOM_LED_BRIGHTNESS 3
#define M5ATOM_LED_BITS 24

//#define M5ATOM_LED_GREEN 0
//#define M5ATOM_LED_RED 1
//#define M5ATOM_LED_BLUE 2
//#define M5ATOM_LED_COLOR_MAX 3

#define M5ATOM_APB_CLK 80
#define M5ATOM_RMT_DIV 20
#define M5ATOM_RMT_CLK (M5ATOM_APB_CLK / M5ATOM_RMT_DIV)
#define WS2812_T0H ((M5ATOM_RMT_CLK * 220 + 999) / 1000)
#define WS2812_T0L ((M5ATOM_RMT_CLK * 580 + 999) / 1000)
#define WS2812_T1H ((M5ATOM_RMT_CLK * 580 + 999) / 1000)
#define WS2812_T1L ((M5ATOM_RMT_CLK * 220 + 999) / 1000)

const rmt_item32_t m5atom_led_bit[] = {
    {{{ WS2812_T0H, 1, WS2812_T0L, 0 }}}, // 220ns high, 580ns low
    {{{ WS2812_T1H, 1, WS2812_T1L, 0 }}}, // 580ns high, 220ns low
    {{{ 32767, 0, 32767, 0 }}}, // reset duration >= 280us
    {{{ 0, 0, 0, 0 }}}, // end of frame
};

void m5atom_led_init(void)
{
    static const rmt_config_t config = {
	.rmt_mode = RMT_MODE_TX,
	.channel = M5ATOM_RMT_TX_CH,
	.clk_div = M5ATOM_RMT_DIV,
	.gpio_num = M5ATOM_LED_GPIO,
	.mem_block_num = 1,
	.tx_config = {
	    .loop_en = true,
	    .idle_level = RMT_IDLE_LEVEL_LOW,
	    .idle_output_en = true,
	},
    };
    int i;

    ESP_ERROR_CHECK(rmt_driver_install(M5ATOM_RMT_TX_CH, 0, 0));
    ESP_ERROR_CHECK(rmt_config(&config));

    // initialize RMT MEM
    for (i = 0; i < M5ATOM_LED_BITS; i++) {
	ESP_ERROR_CHECK(rmt_fill_tx_items(M5ATOM_RMT_TX_CH, &m5atom_led_bit[0], 1, i));
    }

    // reset duration
    ESP_ERROR_CHECK(rmt_fill_tx_items(M5ATOM_RMT_TX_CH, &m5atom_led_bit[2], 1, i++));
    // set end of itmes
    ESP_ERROR_CHECK(rmt_fill_tx_items(M5ATOM_RMT_TX_CH, &m5atom_led_bit[3], 1, i));

    // start RMT TX
    ESP_ERROR_CHECK(rmt_tx_start(M5ATOM_RMT_TX_CH, true));
}

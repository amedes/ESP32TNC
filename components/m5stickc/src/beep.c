/*
 * beeper for M5StickC Plus
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/rmt.h"
#include "driver/gpio.h"
#include "esp_err.h"

#define APB_CLK	(80 * 1000 * 1000)

#define BEEP_CHANNEL	7
#define BEEP_GPIO	GPIO_NUM_2
#define BEEP_DIV	250
#define BEEP_FREQ	4000
#define BEEP_TICKS_MS	(APB_CLK / BEEP_DIV / 1000) // ticks of 1 milli sec
#define BEEP_MAX_TICKS	(32767 * 64 * 2)

void beep(int freq, int msec, bool wait)
{
    if (freq <= 0) return;

    // calculation of frequency
    int duty = APB_CLK / 2 / freq;

    if (duty > 65535) duty = 65535;	// lowest freq, 610Hz
    else if (duty == 0) duty = 1;	// highest freq, 40MHz

    rmt_set_tx_carrier(BEEP_CHANNEL, true, duty, duty, RMT_CARRIER_LEVEL_LOW);

    int ticks = msec * BEEP_TICKS_MS;

    if (ticks > BEEP_MAX_TICKS) ticks = BEEP_MAX_TICKS;
    else if (ticks <= 0) ticks = 1;

    volatile rmt_item32_t *item32p = RMTMEM.chan[BEEP_CHANNEL].data32;
    int items = 0;
    rmt_item32_t item32;

    item32.level0 = 0;
    item32.level1 = 0;

    while (ticks > 0) {

	int duration = (ticks > 32767) ? 32767 : ticks;
	ticks -= duration;

	item32.duration0 = duration;

	duration = (ticks > 32767) ? 32767 : ticks;
	ticks -= duration;

	item32.duration1 = duration;

	*item32p++ = item32;
	items++;

    }

    if (items < 64) {

	item32.duration0 = 0;
	item32.duration1 = 0;

	*item32p++ = item32;
    }

    rmt_tx_start(BEEP_CHANNEL, true);

    if (wait) vTaskDelay((msec + 10) / portTICK_PERIOD_MS);
}

void beep_init(void)
{
    static const rmt_config_t conf = {
	.rmt_mode = RMT_MODE_TX,
	.channel = BEEP_CHANNEL,
	.gpio_num = BEEP_GPIO,
	.clk_div = BEEP_DIV,
	.mem_block_num = 1,
	.tx_config = {
	    .carrier_freq_hz = BEEP_FREQ,
	    .carrier_level = RMT_CARRIER_LEVEL_LOW,
	    .idle_level = RMT_IDLE_LEVEL_HIGH,
	    .carrier_duty_percent = 50,
	    .carrier_en = true,
	    .loop_en = false,
	    .idle_output_en = true,
	},
    };

    ESP_ERROR_CHECK(rmt_config(&conf));
    ESP_ERROR_CHECK(rmt_driver_install(conf.channel, 0, 0));

    beep(BEEP_FREQ, 100, false);
}

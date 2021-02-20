/*
 * ledc.c
 *
 * LED brightness control for M5StickC
 *
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/ledc.h"
#include "driver/gpio.h"

#define LED_FREQ	50
#define LED_CHANNEL	LEDC_CHANNEL_0
#define LED_GPIO	GPIO_NUM_10
#define LED_TIMER	LEDC_TIMER_0
#define LED_MODE	LEDC_HIGH_SPEED_MODE
#define LED_DUTY_RES	LEDC_TIMER_8_BIT
#define LED_DUTY	(1 << LED_DUTY_RES)
#define LED_DUTY_ON	(LED_DUTY - LED_DUTY/16)
#define LED_DUTY_OFF	(LED_DUTY)
#define LED_HPOINT	0x0000

// LED on/off
void led_set_level(bool on)
{
    if (on) {
	ledc_set_duty_and_update(LED_MODE, LED_CHANNEL, LED_DUTY_ON, LED_HPOINT);
    } else {
	ledc_set_duty_and_update(LED_MODE, LED_CHANNEL, LED_DUTY_OFF, LED_HPOINT);
    }
}

void led_init(void)
{
    ledc_timer_config_t ledc_timer = {
	.speed_mode = LED_MODE,
	.duty_resolution = LED_DUTY_RES,
	.timer_num = LED_TIMER,
	.freq_hz = LED_FREQ,
	.clk_cfg = LEDC_AUTO_CLK,

    };
    ledc_channel_config_t ledc_channel = {
	.gpio_num = LED_GPIO,
	.speed_mode = LED_MODE,
	.channel = LED_CHANNEL,
	.timer_sel = LED_TIMER,
	.duty = LED_DUTY_OFF,
	.hpoint = LED_HPOINT,
    };

#if 0
    ESP_ERROR_CHECK(gpio_reset_pin(LED_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT_OD));

    printf("GPIO %d = %d\n", BEEP_GPIO, gpio_get_level(BEEP_GPIO));
#endif

    //return;

#define ESP_INTR_FLAG_DEFAULT (0)
    
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    //ESP_ERROR_CHECK(ledc_timer_pause(LED_MODE, LED_TIMER));
    ESP_ERROR_CHECK(ledc_fade_func_install(ESP_INTR_FLAG_DEFAULT));
    ESP_ERROR_CHECK(ledc_set_duty_and_update(LED_MODE, LED_CHANNEL, LED_DUTY_OFF, LED_HPOINT));
}

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "soc/i2s_periph.h"
#include "soc/sens_periph.h"
#include "soc/syscon_periph.h"
#include "esp_idf_version.h"

#include "config.h"
#include "tnc.h"

#define TAG "I2S"

//i2s number
#define I2S_NUM           (0)

//i2s sample rate
#ifdef SOFTMODEM_PORTS
#define I2S_SAMPLE_RATE   (SAMPLING_RATE * SOFTMODEM_PORTS)
#else
#define I2S_SAMPLE_RATE   (SAMPLING_RATE * TNC_PORTS)
#endif

//#define I2S_DMA_BUF_COUNT	2
#if TNC_PORTS >= 2
#define I2S_DMA_BUF_COUNT	(TNC_PORTS * 8)
#else
#define I2S_DMA_BUF_COUNT	8
#endif
#define I2S_DMA_BUF_LEN		256

//I2S built-in ADC unit
#define I2S_ADC_UNIT              ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CHANNEL           ADC1_CHANNEL_0 // GPIO39

#define ESP_INTR_FLAG_DEFAULT	0

/*
 * i2s initialize
 */
void i2s_init(tcb_t tcb[])
{
	i2s_port_t i2s_num = I2S_NUM;
	i2s_config_t i2s_config = {
            .mode = I2S_MODE_MASTER | I2S_MODE_RX
#ifdef M5STICKC_AUDIO
		| I2S_MODE_PDM
#else
		| I2S_MODE_ADC_BUILT_IN
#endif
		    ,
	    .sample_rate =  I2S_SAMPLE_RATE,
	    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
	    // for ESP-IDF v4.2 or later
	    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
	    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
#endif
	    //.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
	    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
	    //.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
	    .intr_alloc_flags = ESP_INTR_FLAG_DEFAULT,
	    .dma_buf_count = I2S_DMA_BUF_COUNT,
	    .dma_buf_len = I2S_DMA_BUF_LEN,
#ifdef M5STICKC_AUDIO
	    .use_apll = true,
#else
	    .use_apll = false,
#endif
	};

	//install and start i2s driver
	ESP_ERROR_CHECK(i2s_driver_install(i2s_num, &i2s_config, 0, NULL));

#ifdef M5STICKC_AUDIO

	// initialize input pin for SPM1423
	static const i2s_pin_config_t pin_config = {
	    .bck_io_num = I2S_PIN_NO_CHANGE,
	    .ws_io_num = GPIO_NUM_0,	// CLK
	    .data_out_num = I2S_PIN_NO_CHANGE,
	    .data_in_num = GPIO_NUM_34,	// DATA
	};
	ESP_ERROR_CHECK(i2s_set_pin(i2s_num, &pin_config));
	ESP_ERROR_CHECK(i2s_set_clk(i2s_num, I2S_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO));

	// set down sample rate to 8 * 16
	ESP_ERROR_CHECK(i2s_set_pdm_rx_down_sample(i2s_num, I2S_PDM_DSR_16S));

#else // !M5STICKC_AUDIO

	//init ADC pad
	ESP_ERROR_CHECK(i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL));
	// enable adc2
	//ESP_ERROR_CHECK(i2s_set_adc_mode(ADC_UNIT_2, ADC2_CHANNEL_0));
	// enable adc
	ESP_ERROR_CHECK(i2s_adc_enable(i2s_num));

	// delay for I2S bug workaround?
	vTaskDelay(10 / portTICK_PERIOD_MS);

	// ***IMPORTANT*** enable continuous adc sampling
	SYSCON.saradc_ctrl2.meas_num_limit = 0;

	ESP_LOGI(TAG, "sampling rate: %d Hz", SAMPLING_RATE);
	ESP_LOGI(TAG, "sample delay: %d", DELAYED_N);
	ESP_LOGI(TAG, "delay: %f us", DELAYED_N * 1000000.0 / SAMPLING_RATE);

	// channel, attenation, bit width
	SYSCON.saradc_sar1_patt_tab[0] = ((ADC1_CHANNEL_0 << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_0) << 24;

	int num_ports = 0;
	// setup multi-channel scanning mode
	for (int i = 0; i < TNC_PORTS; i++) {

#if defined(ENABLE_TCM3105) && !defined(TCM3105_ADC)
		if (i == TCM3105_PORT) {
			// enable TCM3105 for the port but do not use ADC, skip ADC setting 
			continue;
		}
#endif

	    int ch = TNC_ADC_CH[i];
	    ESP_LOGI(TAG, "TNC port %d, adc channel %d", i, ch);

	    SYSCON.saradc_sar1_patt_tab[i / 4] &= ~(0xff << (3 - (i % 4)) * 8);
#ifdef FX25TNCR2
	    SYSCON.saradc_sar1_patt_tab[i / 4] |= ((ch << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_6) << (3 - (i % 4)) * 8; // attenation 6dB, input voltage 2.2 Vpp
#else
	    SYSCON.saradc_sar1_patt_tab[i / 4] |= ((ch << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_0) << (3 - (i % 4)) * 8; // attenation 0dB, input voltage 1.1 Vpp
#endif
	    ESP_LOGI(TAG, "sart1_patt_tab[%d] = %08x", i/4, SYSCON.saradc_sar1_patt_tab[i/4]);
		num_ports++;
	}

	SYSCON.saradc_ctrl.sar1_patt_len = num_ports - 1; // set pattern length

	// sample time setting
	SYSCON.saradc_ctrl.sar_clk_div = 32;
	SYSCON.saradc_fsm.sample_cycle = 16;
	ESP_LOGI(TAG, "sar_clk_div = %d", SYSCON.saradc_ctrl.sar_clk_div);
	ESP_LOGI(TAG, "sample_cycle = %d", SYSCON.saradc_fsm.sample_cycle);

#if I2S_SAMPLE_RATE == 18000
	// sample rate true 18000Hz setting
	I2S0.clkm_conf.clkm_div_num = 222;
	I2S0.clkm_conf.clkm_div_b = 2;
	I2S0.clkm_conf.clkm_div_a = 9;
	I2S0.sample_rate_conf.rx_bck_div_num = 20;
#elif I2S_SAMPLE_RATE == 36000
	// sample rate true 36000Hz setting
	I2S0.clkm_conf.clkm_div_num = 222;
	I2S0.clkm_conf.clkm_div_b = 2;
	I2S0.clkm_conf.clkm_div_a = 9;
	I2S0.sample_rate_conf.rx_bck_div_num = 10;
#endif
	ESP_LOGI(TAG, "clkm_div_num = %d", I2S0.clkm_conf.clkm_div_num);
	ESP_LOGI(TAG, "clkm_div_b = %d", I2S0.clkm_conf.clkm_div_b);
	ESP_LOGI(TAG, "clkm_div_a = %d", I2S0.clkm_conf.clkm_div_a);
	ESP_LOGI(TAG, "rx_bck_div_num = %d", I2S0.sample_rate_conf.rx_bck_div_num);

#endif // M5STICKC_AUDIO
}

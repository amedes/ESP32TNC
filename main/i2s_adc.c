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

#include "tnc.h"

#define TAG "I2S"

//#define I2S_DAC_OUTPUT 1
#define MULTI_CHANNEL 1
//#define ENABLE_ADC2 1

//i2s number
#define I2S_NUM           (0)

//i2s sample rate
#ifdef ENABLE_ADC2
#define I2S_SAMPLE_RATE   (SAMPLING_RATE * TNC_PORTS * 2)
#else
#define I2S_SAMPLE_RATE   (SAMPLING_RATE * TNC_PORTS * 1)
#endif

//#define I2S_DMA_BUF_COUNT	2
#if TNC_PORTS >= 2
#define I2S_DMA_BUF_COUNT	TNC_PORTS
#else
#define I2S_DMA_BUF_COUNT	2
#endif
#define I2S_DMA_BUF_LEN		1024

//I2S built-in ADC unit
#define I2S_ADC_UNIT              ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CHANNEL           ADC1_CHANNEL_0 // GPIO39

/*
 * i2s initialize
 */
void i2s_init(tcb_t tcb[])
{
	i2s_port_t i2s_num = I2S_NUM;
	i2s_config_t i2s_config = {
            .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN
#ifdef I2S_DAC_OUTPUT
		    | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN
#endif
		    ,
	    .sample_rate =  I2S_SAMPLE_RATE,
	    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
	    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
	    //.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
	    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
	    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
	    .dma_buf_count = I2S_DMA_BUF_COUNT,
	    .dma_buf_len = I2S_DMA_BUF_LEN,
	    .use_apll = false,
	};

	//install and start i2s driver
	ESP_ERROR_CHECK(i2s_driver_install(i2s_num, &i2s_config, 0, NULL));
	// stop I2S for setup
	//ESP_ERROR_CHECK(i2s_stop(i2s_num));
	// start I2S
	//ESP_ERROR_CHECK(i2s_start(i2s_num));

#ifdef I2S_DAC_OUTPUT
	//init DAC pad
	ESP_ERROR_CHECK(i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN));
#endif

	//init ADC pad
	ESP_ERROR_CHECK(i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL));
	// enable adc2
	//ESP_ERROR_CHECK(i2s_set_adc_mode(ADC_UNIT_2, ADC2_CHANNEL_0));
	// enable adc
	ESP_ERROR_CHECK(i2s_adc_enable(i2s_num));

	// delay for I2S bug workaround?
        vTaskDelay(10 / portTICK_PERIOD_MS);

	//ESP_ERROR_CHECK(adc_set_data_inv(I2S_ADC_UNIT, true));
	
	// ***IMPORTANT*** enable continuous adc sampling
	SYSCON.saradc_ctrl2.meas_num_limit = 0;

	ESP_LOGI(TAG, "sampling rate: %d Hz", SAMPLING_RATE);
	ESP_LOGI(TAG, "sample delay: %d", DELAYED_N);
	ESP_LOGI(TAG, "delay: %f us", DELAYED_N * 1000000.0 / SAMPLING_RATE);

#if 0

#if SAMPLING_RATE == 6728

#if TNC_PORTS == 6
#define DIV_N 56
#define DIV_A 53
#define DIV_B 33
#define DIV_M 35
#else
#define DIV_M 24
#define DIV_N 247
#define DIV_A 63
#define DIV_B 46
#endif

#elif SAMPLING_RATE == 13456

#if TNC_PORTS == 6
#define DIV_M 4
#define DIV_N 247
#define DIV_A 43
#define DIV_B 31
#else
#define DIV_M 12
#define DIV_N 247
#define DIV_A 43
#define DIV_B 31
#endif

#elif SAMPLING_RATE == 13200

#if TNC_PORTS == 6
#define DIV_M 4
#define DIV_N 252
#define DIV_A 59
#define DIV_B 31
#else
#define DIV_M 16
#define DIV_N 189
#define DIV_A 33
#define DIV_B 13
#endif

#elif SAMPLING_RATE == 13440

#if TNC_PORTS == 6
#define DIV_M 4
#define DIV_N 248
#define DIV_A 63
#define DIV_B 1
#else
#define DIV_M 12
#define DIV_N 248
#define DIV_A 63
#define DIV_B 1
#endif

#elif SAMPLING_RATE == 13441

#if TNC_PORTS == 6
#define DIV_M 4
#define DIV_N 248
#define DIV_A 63
#define DIV_B 0
#else
#define DIV_M 12
#define DIV_N 248
#define DIV_A 63
#define DIV_B 0
#endif

#elif SAMPLING_RATE == 20184

#if TNC_PORTS == 6
#define DIV_M 3
#define DIV_N 220
#define DIV_A 56
#define DIV_B 11
#else
#define DIV_M 9
#define DIV_N 220
#define DIV_A 56
#define DIV_B 11
#endif

#endif

#ifdef DIV_M
	// set accurate sampling rate, 80736 Hz (6728 * 6 * 2)
	// 160 MHz / (247 + 31/43) / 8 = 80736.0120... Hz
	I2S0.clkm_conf.clkm_div_num = DIV_N;
	I2S0.clkm_conf.clkm_div_a = DIV_A;
	I2S0.clkm_conf.clkm_div_b = DIV_B;
	I2S0.sample_rate_conf.rx_bck_div_num = DIV_M;
	ESP_LOGI(TAG, "PLL_D2: Req RATE: %u, real rate: %d.%03d, N = %d, b/a = %d/%d, M = %d",
		SAMPLING_RATE * TNC_PORTS,

#define D2_CLK (160 * 1000 * 1000)
#define DIVIDEND ((double)D2_CLK * DIV_A)
#define DIVISOR ((DIV_N * DIV_A + DIV_B) * DIV_M)
#define REAL ((double)D2_CLK / 2.0 / DIV_M / (DIV_N + (double)DIV_B / DIV_A))

		(int)REAL, (int)((REAL - floor(REAL)) * 1000.0),
		I2S0.clkm_conf.clkm_div_num,
		I2S0.clkm_conf.clkm_div_b,
		I2S0.clkm_conf.clkm_div_a,
		I2S0.sample_rate_conf.rx_bck_div_num);
#endif

#endif

	// channel, attenation, bit width
	SYSCON.saradc_sar1_patt_tab[0] = ((ADC1_CHANNEL_0 << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_0) << 24;

#ifdef MULTI_CHANNEL
	// setup multi-channel scanning mode
	int ch;

	for (int i = 0; i < TNC_PORTS; i++) {

#if 0
	    ch = (i == 0) ? 0 : i + 2;
#else
	    ch = TNC_ADC_CH[i];
	    ESP_LOGI(TAG, "TNC port %d, adc channel %d", i, ch);
#endif

	    SYSCON.saradc_sar1_patt_tab[i / 4] &= ~(0xff << (3 - (i % 4)) * 8);
#ifdef FX25TNCR2
	    SYSCON.saradc_sar1_patt_tab[i / 4] |= ((ch << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_6) << (3 - (i % 4)) * 8; // attenation 6dB, input voltage 2.2 Vpp
#else
	    SYSCON.saradc_sar1_patt_tab[i / 4] |= ((ch << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_0) << (3 - (i % 4)) * 8; // attenation 0dB, input voltage 1.1 Vpp
#endif
	    ESP_LOGI(TAG, "sart1_patt_tab[%d] = %08x", i/4, SYSCON.saradc_sar1_patt_tab[i/4]);
	    ch++;
	}
    	SYSCON.saradc_ctrl.sar1_patt_len = TNC_PORTS - 1; // set pattern length

	// adc1 controlled by DIG
        SENS.sar_read_ctrl.sar1_dig_force = true;
	SENS.sar_meas_start1.meas1_start_force = true;
	SENS.sar_meas_start1.sar1_en_pad_force = true;
	SENS.sar_touch_ctrl1.xpd_hall_force = true;
	SENS.sar_touch_ctrl1.hall_phase_force = true;
#endif

}

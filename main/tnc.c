/* 
 * tnc.c
 *
 * TNC for ESP32 WiFi module

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "driver/adc.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp32/rom/crc.h"
#include "esp_timer.h"

#include "config.h"

#include "tnc.h"
#include "bell202.h"
#include "i2s_adc.h"
#include "decode.h"
#include "uart.h"
#include "timer.h"
#include "send.h"
#include "wifi.h"
#include "filter.h"
#include "fx25_decode.h"

#ifdef M5ATOM
#include "m5atom.h"
#endif

#ifdef ENABLE_TCM3105
#include "tcm3105.h"
#endif

//#define TNC_PORTS 6 // move to tcb.h
//#define TNC_PORTS 12

static const char TAG[] = "tnc";

// TNC control block
tcb_t tcb[TNC_PORTS];

// adc channel to *tcb table
tcb_t *adc_ch_tcb[8];

#define I2SBUF_SIZE 1024

#define I2S_BUSY_PIN GPIO_NUM_26
#define SEND_BUSY_PIN GPIO_NUM_27

#if defined(ENABLE_TCM3105) && !defined(TCM3105_ADC)

#define BIT_DURATION ((80 * 1000 * 1000 + BAUD_RATE/2) / BAUD_RATE)
#define MAX_DURATION (32 * BIT_DURATION)

// read capture queue
static void read_cap_queue(void *arg)
{
	uint32_t ts;
	tcb_t *tp = (tcb_t *)arg;

	while (1) {
		//gpio_set_level(tp->cdt_led_pin, !tp->cdt_led_on);
		if (xQueueReceive(cap_queue, &ts, portMAX_DELAY) != pdTRUE) {
			ESP_LOGW(TAG, "read capture queue fail");
			continue;
		}
		//gpio_set_level(tp->cdt_led_pin, tp->cdt_led_on);

		int duration = (uint32_t)(ts - tp->prev_ts);
		tp->prev_ts = ts;

		if (duration > MAX_DURATION) { // may be something wrong
#ifdef DEBUG
#ifdef ENABLE_FX25
			ESP_LOGI(TAG, "duration = %d, pll_adj = %d, state = %d, fx25_state = %d", duration / BIT_DURATION, tp->pll_adj, tp->state, tp->fx25_state);
#else
			ESP_LOGI(TAG, "duration = %d, pll_adj = %d, state = %d", duration / BIT_DURATION, tp->pll_adj, tp->state);
#endif
#endif
			tp->pll_adj = 0;
			tp->state = FLAG;
#ifdef FX25_ENABLE
			tp->fx25_state = FX25_FINDTAG;
#endif
			continue;
		}

		duration += tp->pll_adj;
		int timing = BIT_DURATION/2;
		int edge = ts & 1; // 0: positive edge, 1: negative edge

		if (timing <= duration) {
			int bit;

			do {
				bit = (edge == tp->nrzi) ? 1 : 0;
				tp->nrzi = edge;
				decode_bit(tp, bit);
#ifdef FX25_ENABLE
				fx25_decode_bit(tp, bit);
#endif
				timing += BIT_DURATION;
			} while (timing <= duration);

			tp->pll_adj = duration - timing + BIT_DURATION/2;
			tp->pll_adj -= tp->pll_adj >> 2; // * (3/4)
			
		}
		continue;

#if 0
		if (duration + tp->pll_adj < BIT_DURATION/2) { // may be noise
			tp->pll_adj += duration;
			continue;
		}
#endif

#if 1

#if 0
		int bits = (duration + BIT_DURATION/2) / BIT_DURATION;
#else
		int bits = (duration + tp->pll_adj + BIT_DURATION/2) / BIT_DURATION;
		tp->pll_adj = duration - bits * BIT_DURATION;
		tp->pll_adj -= tp->pll_adj >> 2; // * (3/4)
		//tp->pll_adj >>= 1; // * (1/2)
#endif

		decode_bit(tp, 0);
#ifdef FX25_ENABLE
		fx25_decode_bit(tp, 0);
#endif
		while (--bits > 0) {
			decode_bit(tp, 1);
#ifdef FX25_ENABLE
			fx25_decode_bit(tp, 1);
#endif
		}
#else
		// decode NRZI
		if (duration >= BIT_DURATION/2 /*- tp->pll_adj*/) {
			decode_bit(tp, 0);
#ifdef FX25_ENABLE
			fx25_decode_bit(tp, 0);
#endif
			duration -= BIT_DURATION/2 /*- tp->pll_adj*/;
		
			while (duration >= BIT_DURATION) {
				decode_bit(tp, 1);
#ifdef FX25_ENABLE
				fx25_decode_bit(tp, 1);
#endif
				duration -= BIT_DURATION;
			}
		}

		// PLL calculation
		tp->pll_adj = duration - BIT_DURATION/2;
		tp->pll_adj -= tp->pll_adj >> 2; // (1 - 1/4) = 3/4 = 0.75
		if (abs(tp->pll_adj) > BIT_DURATION/2) {
			tp->pll_adj = 0;
		}
#endif
	}
}

#endif // ENABLE_TCM3105

#ifdef ENABLE_SOFTMODEM

// read adc data task
static void read_i2s_adc(void *arg)
{
	static uint16_t buf[I2SBUF_SIZE];
	size_t size;

	while (1)
	{

		if (i2s_read(I2S_NUM_0, buf, I2SBUF_SIZE, &size, portMAX_DELAY) != ESP_OK)
		{
			ESP_LOGI(TAG, "i2s_read() fail");
			continue;
		}

		//gpio_set_level(I2S_BUSY_PIN, 1); // busy

		for (int i = 0; i < size / sizeof(uint16_t); i++)
		{
#if SOFTMODEM_PORTS == 1
			uint16_t adc = buf[i ^ 1]; // I2S ADC storing ADC data in Bigendian order
#else
			uint16_t adc = buf[i];
#endif

#ifdef M5STICKC_AUDIO
			demodulator(&tcb[0], (int16_t)adc + 32768);
#else
			int ch = adc >> 12;
			tcb_t *tp = adc_ch_tcb[ch];

			if (tp)
			{
				// decode adc sample
				if (tp->enable_tcm3105) {
					decode(tp, (adc & 0xfff) >= 2048); // process modem RXD signal
				} else {
					demodulator(tp, adc & 0xfff);
				}
			}
#endif
		}
	}
}

#endif // ENABLE_SOFTMODEM

static const uint8_t CDT_LED_PIN[] = {
#if defined(FX25TNCR1)
	GPIO_LED_PIN,
#elif defined(FX25TNCR2)
	14,
	19,
#elif defined(FX25TNCR3)
	15,
	2,
	0,
	4,
	0,
	4,
#elif defined(FX25TNCR4)
	15, 2,
#elif defined(M5ATOM)
	-1, -1, // using internal RGB LED with RMT SoC
#elif defined(M5STICKC)
	-1, // using internal Red LED with PWM LED SoC
#else
	2, 2, 2, 2, 2, 2, // assume LED connected to GPIO2
#endif
};

// LED on values
static const uint8_t CDT_LED_ON[] = {
#if defined(FX25TNCR1)
	1,
#elif defined(FX25TNCR2)
	0, 0, // CDT LED drived with open drain
#elif defined(FX25TNCR3)
	1,
	1,
	1,
	1,
	1,
	1,
#elif defined(FX25TNCR4)
	1, 1,
#elif defined(M5STICKC) || defined(M5ATOM)
	0, 0,	// not used this value
#else
	1,
	1,
	1,
	1,
	1,
	1,
#endif
};

static const int8_t PTT_PIN[] = {
#if defined(FX25TNCR1)
	GPIO_PTT_PIN,
#elif defined(FX25TNCR2) || defined(FX25TNCR3) || defined(FX25TNCR4)
	23,
	22,
	21,
	19,
	18,
	5,
#elif defined(M5ATOM)
	19, 21, // GPIO19, 21
#elif defined(M5STICKC)
#ifdef M5STICKC_AUDIO
	-1,		// disable PTT
#else
	0, // GPIO0
#endif
#else
	23,
	22,
	21,
	19,
	18,
	5,
//15, 2, 15, 2, 15, 2,
#endif
};

#ifdef FX25TNCR2
static const uint8_t STA_LED_PIN[] = {
	27,
	21,
};
#endif

#ifdef FX25_ENABLE
// FX.25 parity
static const uint8_t FX25_PARITY[] = {
#ifdef FX25_PARITY_0
	0, 0, 0, 0, 0, 0,
#endif
#ifdef FX25_PARITY_16
	16,	16,	16,	16,	16,	16,
#endif
#ifdef FX25_PARITY_32
	32,	32,	32,	32,	32,	32,
#endif
#ifdef FX25_PARITY_64
	64,	64,	64,	64,	64,	64,
#endif
};
#endif

// TNC adc input channel list
// index: port No. 0-5
// value: adc channel
const uint8_t TNC_ADC_CH[] = {
#if defined(FX25TNCR1)
#if GPIO_RXD_PIN == 36
	0,
#elif GPIO_RXD_PIN == 39
	3,
#elif GPIO_RXD_PIN == 34
	6,
#elif GPIO_RXD_PIN == 35
	7,
#elif GPIO_RXD_PIN == 32
	4,
#else
	5,
#endif
#elif defined(FX25TNCR2) || defined(FX25TNCR3)
	0, 3, 6, 7, 4, 5, // GPIO 36, 39, 34, 35, 32, 33,
#elif defined(FX25TNCR4)
#ifdef TCM3105_ADC
	0, 6, // GPIO 36, 34
#else
	0, 3, // GPIO 36, 39
#endif
#elif defined(M5ATOM)
	5, 6,	// GPIO 33, 34
#elif defined(M5STICKC)
	0,		// GPIO 36
#else
	0, 3, 6, 7, 4, 5, // GPIO 36, 39, 34, 35, 32, 33,
#endif
};

// KISS default parameter

// fullduplex default value
const uint8_t kiss_fullduplex[] = {
#ifdef KISS_FULLDUPLEX
	true,
	true,
	true,
	true,
	true,
	true,
#else
	false,
	false,
	false,
	false,
	false,
	false,
#endif
};

// SlotTime default value
const uint8_t kiss_slottime[] = {
#ifdef KISS_SLOTTIME
	KISS_SLOTTIME,
	KISS_SLOTTIME,
	KISS_SLOTTIME,
	KISS_SLOTTIME,
	KISS_SLOTTIME,
	KISS_SLOTTIME,
#else
	10,
	10,
	10,
	10,
	10,
	10,
#endif
};

// TXDELAY default value
const uint8_t kiss_txdelay[] = {
#ifdef KISS_TXDELAY
	KISS_TXDELAY,
	KISS_TXDELAY,
	KISS_TXDELAY,
	KISS_TXDELAY,
	KISS_TXDELAY,
	KISS_TXDELAY,
#else
	50,
	50,
	50,
	50,
	50,
	50,
#endif
};

// persistence P default value
const uint8_t kiss_persistence_p[] = {
#ifdef KISS_P
	KISS_P,
	KISS_P,
	KISS_P,
	KISS_P,
	KISS_P,
	KISS_P,
#else
	63,
	63,
	63,
	63,
	63,
	63,
#endif
};

static filter_t bpf[TNC_PORTS];
static filter_t lpf[TNC_PORTS];

void tnc_init(tcb_t *tcb, int ports)
{
	// filter initialize
	filter_param_t flt = {
		.size = FIR_LPF_N,
		.sampling_freq = SAMPLING_RATE,
		.pass_freq = 0,
		.cutoff_freq = 1200,
	};
	int16_t *lpf_an, *bpf_an;

	// LPF
	lpf_an = filter_coeff(&flt);
	// BPF
	flt.size = FIR_BPF_N;
	flt.pass_freq = 1000;
	flt.cutoff_freq = 2500;
	bpf_an = filter_coeff(&flt);

#ifdef M5STICKC
	ESP_ERROR_CHECK(gpio_reset_pin(GPIO_NUM_25));
	ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_25, GPIO_MODE_DISABLE));
	ESP_ERROR_CHECK(gpio_pullup_dis(GPIO_NUM_25));
	ESP_ERROR_CHECK(gpio_pulldown_dis(GPIO_NUM_25));
#endif

	// initialize TNC Control Block
	for (int i = 0; i < ports; i++)
	{
		tcb_t *tp = &tcb[i];
		uint8_t ch = TNC_ADC_CH[i];

		adc_ch_tcb[ch] = tp;

		tp->port = i;
		tp->kiss_type = i << 4;
		tp->avg = 2048;
		tp->cdt = false;
		tp->cdt_lvl = 0;
		tp->cdt_led_pin = CDT_LED_PIN[i];
		tp->cdt_led_on = CDT_LED_ON[i];
		ESP_LOGI(TAG, "cdt_led_pin = %d, cdt_led_on = %d, port = %d", tp->cdt_led_pin, tp->cdt_led_on, tp->port);
		tp->ptt_pin = PTT_PIN[i];
#ifdef FX25TNCR2
		tp->sta_led_pin = STA_LED_PIN[i];
#endif

#ifdef FX25_ENABLE
		tp->fx25_parity = FX25_PARITY[i]; // FX.25 parity
#endif

		// CDT LED gpio initialize
		if (tp->cdt_led_pin >= GPIO_NUM_0 && tp->cdt_led_pin <= GPIO_NUM_39)
		{
			ESP_ERROR_CHECK(gpio_reset_pin(tp->cdt_led_pin));
			ESP_ERROR_CHECK(gpio_set_direction(tp->cdt_led_pin,
											   (tp->cdt_led_on) ? GPIO_MODE_OUTPUT : GPIO_MODE_OUTPUT_OD));
			ESP_ERROR_CHECK(gpio_set_level(tp->cdt_led_pin, !tp->cdt_led_on));
			ESP_LOGI(TAG, "port = %d, cdt led gpio = %d", tp->port, tp->cdt_led_pin);
		}

#ifdef DEBUG
		ESP_LOGI(TAG, "ptt_pin = %d, port = %d", tp->ptt_pin, tp->port);
#endif
		// PTT gpio initialize
		if (tp->ptt_pin >= 0) {

			ESP_ERROR_CHECK(gpio_reset_pin(tp->ptt_pin));
			ESP_ERROR_CHECK(gpio_set_direction(tp->ptt_pin, GPIO_MODE_OUTPUT));
			ESP_ERROR_CHECK(gpio_set_level(tp->ptt_pin, 0));

			ESP_LOGI(TAG, "port = %d, ptt gpio = %d", tp->port, tp->ptt_pin);
		}

#ifdef FX25TNCR2
		// STA LED gpio initialize
		gpio_reset_pin(tp->sta_led_pin);
		gpio_set_direction(tp->sta_led_pin, GPIO_MODE_OUTPUT);
		gpio_set_level(tp->sta_led_pin, 0);
#endif

#ifdef ENABLE_TCM3105
		// enable TCM3105
		if (i == TCM3105_PORT) {
			tp->enable_tcm3105 = true;
			ESP_LOGI(TAG, "enable_tcm3105: port = %d", TCM3105_PORT);
		} else {
			tp->enable_tcm3105 = false;
		}
#endif

		tp->cdt_sem = xSemaphoreCreateBinary();
		assert(tp->cdt_sem);
		assert(xSemaphoreGive(tp->cdt_sem) == pdTRUE); // initialize

		tp->ptt = 0;

		// KISS default parameter
		tp->fullDuplex = kiss_fullduplex[i]; // half duplex
		//tp->fullDuplex = true; // full duplex
		tp->SlotTime = kiss_slottime[i];		   // 100ms
		tp->TXDELAY = kiss_txdelay[i];			   // 500ms
		tp->persistence_P = kiss_persistence_p[i]; // P = 0.25

		// LPF
		tp->lpf = &lpf[i];
		filter_init(tp->lpf, lpf_an, FIR_LPF_N);
		// BPF
		tp->bpf = &bpf[i];
		filter_init(tp->bpf, bpf_an, FIR_BPF_N);
	}

	// create decode task
#if defined(ENABLE_TCM3105) && !defined(TCM3105_ADC)
	ESP_LOGI(TAG, "create raad_cap_queue task");
	assert(xTaskCreatePinnedToCore(read_cap_queue, "read capture q", 4096, &tcb[TCM3105_PORT], tskIDLE_PRIORITY + 5, NULL, 1) == pdPASS); // pinned CPU 1
#endif

#ifdef ENABLE_SOFTMODEM
	assert(xTaskCreatePinnedToCore(read_i2s_adc, "read i2s", 4096, NULL, tskIDLE_PRIORITY + 5, NULL, 1) == pdPASS); // pinned CPU 1
#endif
}

//#define USEQUEUE 1

#define BUSY_PORT 2

#ifdef USEQUEUE
void packet_send(tcb_t *tp, uint8_t const *buf, size_t size)
#else
void packet_send(tcb_t *tp, uint8_t const *buf, size_t size)
#endif
{
	uint8_t const *p = buf;
#ifdef USEQUEUE
	uint8_t data = 0; // data to modem
#else
	uint32_t data = 0; // data to modem
#endif
	int data_bits = 0; // number of bits
	uint32_t fcs;
	int count_ones = 0;
	int insert_zero = false;
	int do_bitstuffing = true; // 1: do bit stuffing, 0: do not

	if (size <= 0)
		return;

	fcs = crc16_le(0, buf, size); // CRC-16/X.25

#define AX25_FLAG 0x7e

	// send start flag
	static const uint8_t flag = AX25_FLAG;

	if (tp->port == BUSY_PORT) //gpio_set_level(SEND_BUSY_PIN, 0); // free

#ifdef USEQUEUE
		xQueueSend(tp->queue, &flag, portMAX_DELAY);
#else
		xRingbufferSend(tp->queue, &flag, sizeof(flag), portMAX_DELAY);
#endif

	if (tp->port == BUSY_PORT) //gpio_set_level(SEND_BUSY_PIN, 1); // busy

		for (int i = 0; i < size + 2; i++)
		{				   // +2 means FCS and end flag
			uint32_t bitq; // bit queue
			int bitq_bits; // number of bits

			if (i < size)
			{
				bitq = *p++; // send data
				bitq_bits = 8;
			}
			else if (i == size)
			{
				bitq = fcs; // send FCS, size is 2 bytes
				bitq_bits = 16;
			}
			else
			{
				bitq = 0x7e; // send end flag
				bitq_bits = 8;
				do_bitstuffing = false; // do not bit stuffing
			}

			//while (bitq > 1) { // bit queue is not empty
			while (bitq_bits-- > 0)
			{
				int bit;

				if (insert_zero)
				{

					bit = 0;
					insert_zero = false;
				}
				else
				{

					bit = bitq & 1;
					bitq >>= 1;

					// bit stuffing
					if (do_bitstuffing)
					{

						if (bit)
						{

#define BIT_STUFFING_BITS 5

							if (++count_ones >= BIT_STUFFING_BITS)
							{ // need bit stuffing
								insert_zero = true;
								bitq_bits++;
								count_ones = 0;
							}
						}
						else
						{
							count_ones = 0;
						}
					}
				}

				//data >>= 1;
				//data |= bit << 7; // insert the bit to MSb
				data |= bit << data_bits;

#ifdef USEQUEUE
				if (++data_bits >= 8)
				{ // filled all 8 bits
#else
				if (++data_bits >= 32)
				{ // filled all 32 bits
#endif

					//if (tp->port == BUSY_PORT) gpio_set_level(SEND_BUSY_PIN, 0); // free

#ifdef USEQUEUE
					xQueueSend(tp->queue, &data, portMAX_DELAY);
#else
					xRingbufferSend(tp->queue, &data, sizeof(data), portMAX_DELAY);
#endif
					//if (tp->port == BUSY_PORT) gpio_set_level(SEND_BUSY_PIN, 1); // busy

					data = 0;
					data_bits = 0;
				}
			}
		}

	if (data_bits > 0)
	{ // there is a fraction of a byte
		int byte_size = (data_bits + 7) / 8;

		//if (tp->port == BUSY_PORT) gpio_set_level(SEND_BUSY_PIN, 0); // free

#ifdef USEQUEUE
		xQueueSend(tp->queue, &data, portMAX_DELAY);
#else
		xRingbufferSend(tp->queue, &data, byte_size, portMAX_DELAY);
#endif

		//if (tp->port == BUSY_PORT) gpio_set_level(SEND_BUSY_PIN, 1); // busy
	}
}

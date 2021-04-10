/* AX.25 decoder

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
#include "esp32/rom/crc.h"

#include "config.h"

#include "tnc.h"
#include "bell202.h"
#include "i2s_adc.h"
#include "uart.h"
#include "decode.h"

#ifdef FX25_ENABLE
#include "fx25_decode.h"
#endif

#ifdef M5STICKC
#include "led.h"
#endif

#define TAG "decode"
//static RingbufHandle_t ringbuf;

#define BAUD_RATE 1200
//#define SAMPLING_RATE 6726
#define BIT_DURATION (SAMPLING_RATE / BAUD_RATE)

#define AX25_ADDR_LEN 7
#define AX25_MIN_PKT_SIZE (AX25_ADDR_LEN * 2 + 1 + 2) // src addr + dst addr + Control + FCS
#define AX25_SSID_MASK 0x0f

static int ax25_check_fcs(uint8_t data[], int len)
{
	//uint16_t fcs, crc;

	if (len < AX25_MIN_PKT_SIZE)
		return false;

#if 0
    fcs = data[len - 1] << 8 | data[len - 2];
    crc = crc16_le(0, data, len - 2);

    static int count = 0;
    if (fcs == crc && ++count > 10) {
	ESP_LOGI(TAG, "GOOD_CRC = %04x", crc16_le(0, data, len));
	count = 0;
    }

    return fcs == crc;
#else

#define GOOD_CRC 0x0f47

	return crc16_le(0, data, len) == GOOD_CRC;
#endif
}

#if 0
static void output_packet(tcb_t *tp, uint8_t data[], int len)
{
    int i;
    int in_addr = 1;
    int fcs;
    int crc;

    if (len < AX25_MIN_PKT_SIZE) return;

    fcs = data[len - 1] << 8 | data[len - 2];
    crc = crc16_le(0, data, len - 2);
    if (fcs != crc) return;

    //printf("%d:%d:", tp->port, ++tp->pkts);

    for (i = 0; i < len - 2; i++) {
	int c = data[i];
	int d = c >> 1;

	if (in_addr) {
	    if (i % AX25_ADDR_LEN == AX25_ADDR_LEN - 1) { // SSID
		if (d & AX25_SSID_MASK) printf("-%d", d & AX25_SSID_MASK);
		putchar(',');
	    } else {
		if (d >= ' ' && d <= '~') putchar(d);
		else printf("<%02x>", d);
	    }
	    in_addr = (c & 1) == 0;
	} else {
	    if (c >= ' ' && c <= '~') putchar(c);
	    else printf("<%02x>", c);
	}
    }
    printf("<%02x%02x>\n", data[len-1], data[len-2]);
}
#endif

#define AX25_FLAG 0x7e
#define AX25_MASK 0xfc		// bit mask of MSb six bits
#define AX25_EOP 0xfc		// end of packet, 7e << 1
#define AX25_STUFF_BIT 0x7c // bit stuffing bit, five of continuous one bits
//#define DATA_LEN 256 // maximum packet size
//#define AX25_MIN_PKT_SIZE (7 * 2 + 1 + 1 + 2) // call sign * 2 + control + PID + FCS
#define AX25_FLAG_BITS 6

void decode_bit(tcb_t *tp, uint8_t bit)
{
	//static uint8_t state = FLAG;
	//static uint8_t flag = 0;
	//static uint8_t data[DATA_LEN];
	//static int data_cnt = 0;
	//static uint8_t data_byte = 0;
	//static uint8_t data_bit_cnt = 0;

	tp->flag >>= 1;
	tp->flag |= bit << 7;

	switch (tp->state)
	{
	case FLAG:
		if (tp->flag == AX25_FLAG)
		{ // found flag
			tp->state = DATA;
			tp->data_cnt = 0;
			tp->data_bit_cnt = 0;
			//cnt = (edge + SAMPLING_N/2) % SAMPLING_N; // bit sync
			//ESP_LOGI(TAG, "found AX25_FALG");
		}
		break;

	case DATA:
		if ((tp->flag & AX25_MASK) == AX25_EOP) { // AX.25 flag, end of packet

			if (tp->data_bit_cnt == AX25_FLAG_BITS && tp->data_cnt >= AX25_MIN_PKT_SIZE) {
				if (ax25_check_fcs(tp->data, tp->data_cnt)) { // FCS ok
					tp->pkts++;
#ifdef FX25_ENABLE
					tp->decode_time = xTaskGetTickCount();
#endif
					//if (xRingbufferSend(uart_rb, tp->data, tp->data_cnt, portMAX_DELAY) != pdTRUE) {
					if (xRingbufferSend(uart_rb, &tp->kiss_type, tp->data_cnt - 2 + 1, 0) != pdTRUE) { // kiss_type leads data[], discard packet if ringbuffer is full
						ESP_LOGW(TAG, "xRingbufferSend() fail, port = %d", tp->port);
					}

					// output bit count
					// //ESP_LOGI(TAG, "data_cnt = %d, data_bit_cnt = %d, port = %d", tp->data_cnt, tp->data_bit_cnt, tp->port);
				} else {
#ifdef DEBUG
					ESP_LOGI(TAG, "FCS error, size = %d, port = %d", tp->data_cnt, tp->port);
#endif
				}
			}
			tp->state = FLAG;
			break;
		}

		if ((tp->flag & AX25_MASK) == AX25_STUFF_BIT)
			break; // delete bit stuffing bit

		tp->data_byte >>= 1;
		tp->data_byte |= bit << 7;
		tp->data_bit_cnt++;
		if (tp->data_bit_cnt >= 8)
		{
			if (tp->data_cnt < DATA_LEN) {
				tp->data[tp->data_cnt++] = tp->data_byte;
				tp->data_bit_cnt = 0;
			} else {
				ESP_LOGW(TAG, "buffer overflow");
				tp->state = FLAG;
			}
		}
	}
}

#define PLL_DIV (SAMPLING_RATE / BAUD_RATE)
#define PLL_INC	((1LLU << 32) / PLL_DIV)

static void decode(tcb_t *tp, int val)
{
	int32_t prev_clk = tp->pll_clock;

	tp->pll_clock += PLL_INC;

	if (tp->pll_clock < prev_clk) {
		int bit;

		// decode NRZI
		bit = (val == tp->nrzi) ? 1 : 0;
		tp->nrzi = val;

		// process one bit
		decode_bit(tp, bit);
#ifdef FX25_ENABLE
		fx25_decode_bit(tp, bit);
#endif
	}

	if (val != tp->pval) { // detect edge

		tp->pll_clock -= tp->pll_clock >> 2;  // (1 - 1/4) = 3/4 = 0.75
		tp->pval = val;
	}
}

/*
 * decode AFSK, carrier detect, decode AX.25
 */
void demodulator(tcb_t *tp, uint16_t adc)
{
	int val;
	int level;

#define AVERAGE_N 8
#define CDT_AVG_N 128

	// update average value
	//tp->avg = (tp->avg * (AVERAGE_N - 1) + adc + AVERAGE_N/2) / AVERAGE_N;
#if 1
	tp->avg_sum += adc - tp->avg_buf[tp->avg_idx];
	tp->avg_buf[tp->avg_idx++] = adc;
	if (tp->avg_idx >= TCB_AVG_N)
		tp->avg_idx -= TCB_AVG_N;
	tp->avg = tp->avg_sum / TCB_AVG_N;
#endif

	// carrier detect
	val = (int)adc - tp->avg;
	tp->cdt_lvl = (tp->cdt_lvl * (CDT_AVG_N - 1) + val * val + CDT_AVG_N / 2) / CDT_AVG_N;

#define CDT_THR_LOW 8192
//#define CDT_THR_LOW (8192 * 2) // for M5StickC Plus, noise level too high?
#define CDT_THR_HIGH (CDT_THR_LOW * 4) // low +3dB

	// simulate carrier detect signal
	if (!tp->cdt && tp->cdt_lvl > CDT_THR_HIGH)	{ // CDT on
		xSemaphoreTake(tp->cdt_sem, 0); // take semaphore for CDT
#ifdef M5ATOM
		m5atom_led_set_level(M5ATOM_LED_GREEN, 1);
#elif defined(M5STICKC)
		led_set_level(1);
#else
		gpio_set_level(tp->cdt_led_pin, tp->cdt_led_on);
#endif
		tp->cdt = true;
#ifdef DEBUG
		ESP_LOGI(TAG, "CDT on, port = %d", tp->port);
#endif
	} else if (tp->cdt && tp->cdt_lvl < CDT_THR_LOW) { // CDT off
		xSemaphoreGive(tp->cdt_sem); // give semaphore for CDT
#ifdef M5ATOM
		m5atom_led_set_level(M5ATOM_LED_GREEN, 0);
#elif defined(M5STICKC)
		led_set_level(0);
#else
		gpio_set_level(tp->cdt_led_pin, !tp->cdt_led_on);
#endif
		tp->cdt = false;
#ifdef DEBUG
		ESP_LOGI(TAG, "CDT off, port = %d", tp->port);
#endif
	}

#ifdef DEBUG
	static int count = 0;
	if (count > SAMPLING_RATE * 10)
	{
		ESP_LOGI(TAG, "adc: %u, avg: %d, cdt: %d, port = %d", adc, tp->avg, tp->cdt_lvl, tp->port);
		if (tp->port == 0)
			count = 0;
	}
	if (tp->port == 0)
		++count;
#endif

	if (tp->cdt)
	{ // decode when cdt is on

		// BPF, 900 - 2500 Hz
		val = filter(tp->bpf, val);

#define HYSTERERSIS 0

		// deocde bell 202 AFSK from ADC value
		level = bell202_decode(tp, val); // level < 0: mark, level > 0: space
		if (tp->bit)
		{ // if mark
			if (level > HYSTERERSIS)
			{
				tp->bit = 0; // space
			}
		}
		else
		{ // if space
			if (level < -HYSTERERSIS)
			{
				tp->bit = 1; // mark
			}
		}

		// decode AX.25 packet, decode NRZI, delete bit stuffing bit, etc...
		decode(tp, tp->bit);
	}
}

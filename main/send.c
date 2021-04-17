/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp32/rom/crc.h"
#include "driver/gpio.h"

#include "tnc.h"
#include "fx25.h"

#ifdef M5ATOM
#include "m5atom.h"
#endif

static const char TAG[] = "send";


#define BUSY_PORT 2

void send_bytes(tcb_t *tp, void const *data, size_t data_len)
{
	xRingbufferSend(tp->queue, data, data_len, portMAX_DELAY);
}

static void packet_send(tcb_t *tp, uint8_t *buf, size_t size)
{
    uint32_t data = 0; // data to modem
    int data_bits = 0; // number of bits
    int count_ones = 0;
    int insert_zero = false;
    int do_bitstuffing = true; // 1: do bit stuffing, 0: do not
    uint16_t fcs;

    int packet_size = size;

    if (packet_size <= 0) return;

    //ESP_LOGI(TAG, "packet_send(), size = %d, port = %d", packet_size, tp->port);

    fcs = crc16_le(0, buf, size); // CRC-16/X.25

#define AX25_FLAG 0x7e

    // send start flag
    static const uint8_t flag = AX25_FLAG;

    send_bytes(tp, &flag, sizeof(flag));

    //ESP_LOGI(TAG, "send start flag, port = %d", tp->port);

    for (int i = 0; i < packet_size + 2; i++) { // send buf[0..packet_size], FCS and End flag
		uint32_t bitq; // bit queue
		int bitq_bits; // number of bits

		if (i < packet_size) { // send buf data
	    	bitq = buf[i];
	    	bitq_bits = 8;
		} else if (i == packet_size) { // send FCS
		    bitq = fcs;
	    	bitq_bits = 16;
		} else { // send End flag
		    bitq = AX25_FLAG;
		    bitq_bits = 8;
		    do_bitstuffing = false;
		}

		while (bitq_bits-- > 0) {
	    	int bit;

	    	if (insert_zero) {

				bit = 0;
				insert_zero = false;

	    	} else {

				bit = bitq & 1;
				bitq >>= 1;

	    		// do bit stuffing
	    		if (do_bitstuffing) {

		   			if (bit) {

#define BIT_STUFFING_BITS 5
		    
		    			if (++count_ones >= BIT_STUFFING_BITS) { // insert zero
			    			insert_zero = true;
			    			bitq_bits++;
			    			count_ones = 0;
						}

		   			} else {
		    			count_ones = 0;
		   			}

				}

	    	}

	    	data |= bit << data_bits;

	    	if (++data_bits >= 32) { // filled all 32 bits
				send_bytes(tp, &data, sizeof(data));

	    		data = 0;
				data_bits = 0;

			}

		} // while (bitq_bits-- > 0)

    } // for (i = 0; ..

    if (data_bits > 0) { // there is data to be sent

		int byte_size = (data_bits + 7) / 8;
		send_bytes(tp, &data, byte_size);

    	//ESP_LOGI(TAG, "packet_send(): send extra %d bytes, port = %d", byte_size, tp->port);
    }

}

static void send_task(void *arg)
{
    tcb_t *tp = (tcb_t *)arg; // pointer to TCB
    unsigned int seed = (unsigned int)esp_timer_get_time();

#define BAUD_RATE 1200
#define TIME_UNIT 10 // msec
#define TIME_UNIT_BITS (BAUD_RATE * TIME_UNIT / 1000)

    while (1) {

#define TXD_BYTES(t) ((t * 12 + 7) / 8)

	uint8_t *item;
	size_t itemsize;

	// receive from queue
	if ((item = xRingbufferReceive(tp->input_rb, &itemsize, portMAX_DELAY)) == NULL) {
	    ESP_LOGW(TAG, "xRingbufferReceive() fail, port = %d", tp->port);
	    continue;
	}

	//ESP_LOGI(TAG, "xRingbufferReceiveSplit(), size = %d, port = %d", itemsize[0] + (item[1]) ? itemsize[1] : 0, tp->port);

#ifdef FX25TNCR2
	gpio_set_level(tp->sta_led_pin, 1); // STA LED on
#endif

	// begin to send packet
	if (!tp->ptt) { // if ptt is off

	    ESP_LOGD(TAG, "PTT is off, port = %d", tp->port);

	    while (!tp->fullDuplex) { // if half duplex

	    	// check and wait for CDT off
	    	if (xSemaphoreTake(tp->cdt_sem, portMAX_DELAY) != pdTRUE) {
		    ESP_LOGW(TAG, "xSemaphoreTake() fail, port = %d", tp->port);
		    continue;
		}

		// give samephore
		if (xSemaphoreGive(tp->cdt_sem) != pdTRUE) {
		    ESP_LOGW(TAG, "xSemaphoreGive() fail, port = %d", tp->port);
		}

		// P-persistence
		if ((rand_r(&seed) & 0xff) <= tp->persistence_P) break; // exit loop 

		// wait slot time
		vTaskDelay(tp->SlotTime * 10 / portTICK_PERIOD_MS);

	    } // while (!tp->fullDuplex)

	    // transmitter on

#ifndef M5STICKC_AUDIO
	    gpio_set_level(tp->ptt_pin, 1); // PTT on
#endif
#ifdef M5ATOM
	    m5atom_led_set_level(M5ATOM_LED_RED, 1); // PTT LED on
#endif
	    tp->ptt = true;
#ifdef DEBUG
	    ESP_LOGI(TAG, "PTT on, gpio = %d, port = %d", tp->ptt_pin, tp->port);
	    ESP_LOGI(TAG, "sending preamble, TXDELAY = %d, bytes = %d, port = %d", tp->TXDELAY, TXD_BYTES(tp->TXDELAY), tp->port);
#endif
	    // wait for TXDELAY
	    for (int i = 0; i < TXD_BYTES(tp->TXDELAY); i++) {
			uint8_t data = 0x7e; // flag (7E)
		
			send_bytes(tp, &data, sizeof(data));
	    }

	    //ESP_LOGI(TAG, "sent preamble, port = %d", tp->port);

	} // if (!tp->ptt)

#ifdef FX25_ENABLE
	if (tp->fx25_parity > 0) {
#ifdef DEBUG
	    ESP_LOGI(TAG, "send FX.25, parity = %d", tp->fx25_parity);
#endif
	    if (fx25_send_packet(tp, item, itemsize, tp->fx25_parity) != 0) {
#ifdef DEBUG
		    ESP_LOGI(TAG, "send FX.25 packet fail");
#endif
	    	packet_send(tp, item, itemsize);		
		}
	} else {
	    packet_send(tp, item, itemsize);
	}
#else
	packet_send(tp, item, itemsize);
#endif

	// return item
	vRingbufferReturnItem(tp->input_rb, item);

    } // while (1)
}

#define RINGBUF_SIZE (1024 * 2)

void send_init(tcb_t tcb[])
{
#define TX_QUEUE_SIZE 256

#define SEND_TASKS 6

    for (int i = 0; i < TNC_PORTS; i++) {

	// output queue
    	tcb[i].queue = xRingbufferCreate(TX_QUEUE_SIZE, RINGBUF_TYPE_BYTEBUF);
    	assert(tcb[i].queue != NULL);

	// input queue
		tcb[i].input_rb = xRingbufferCreate(RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    	assert(tcb[i].input_rb != NULL);

	// create send task
    	assert(xTaskCreatePinnedToCore(send_task, "send task", 4096, &tcb[i], tskIDLE_PRIORITY + 2, &tcb[i].task, 1) == pdPASS);
    }

}

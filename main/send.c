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

#ifdef M5ATOM
#include "m5atom.h"
#endif

static const char TAG[] = "send";

//#define USEQUEUE 1

#define BUSY_PORT 2

#if 0

#ifdef USEQUEUE
static void packet_send(tcb_t *tp, uint8_t const *buf, size_t size)
#else
static void packet_send(tcb_t *tp, uint8_t const *buf, size_t size)
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

    if (size <= 0) return;

    fcs = crc16_le(0, buf, size); // CRC-16/X.25

#define AX25_FLAG 0x7e

    // send start flag
    static const uint8_t flag = AX25_FLAG;

    if (tp->port == BUSY_PORT) gpio_set_level(SEND_BUSY_PIN, 0); // free

#ifdef USEQUEUE
    xQueueSend(tp->queue, &flag, portMAX_DELAY);
#else
    xRingbufferSend(tp->queue, &flag, sizeof(flag), portMAX_DELAY);
#endif

    if (tp->port == BUSY_PORT) gpio_set_level(SEND_BUSY_PIN, 1); // busy

    for (int i = 0; i < size + 2; i++) { // +2 means FCS and end flag
	uint32_t bitq; // bit queue
	int bitq_bits; // number of bits
       
	if (i < size) {
	    bitq = *p++; // send data
	    bitq_bits  = 8;
	} else if (i == size) {
	    bitq = fcs; // send FCS, size is 2 bytes
	    bitq_bits  = 16;
	} else {
	    bitq = 0x7e; // send end flag
	    bitq_bits = 8;
	    do_bitstuffing = false; // do not bit stuffing
	}

	//while (bitq > 1) { // bit queue is not empty
	while (bitq_bits-- > 0) {
	    int bit;

	    if (insert_zero) {

		bit = 0;
		insert_zero = false;

	    } else {

		bit = bitq & 1;
		bitq >>= 1;

	    	// bit stuffing
	    	if (do_bitstuffing) {

		   if (bit) {

#define BIT_STUFFING_BITS 5
		    
		    	if (++count_ones >= BIT_STUFFING_BITS) { // need bit stuffing
			    insert_zero = true;
			    bitq_bits++;
			    count_ones = 0;
			}

		   } else {
		    count_ones = 0;
		   }

		}

	    }

	    //data >>= 1;
	    //data |= bit << 7; // insert the bit to MSb
	    data |= bit << data_bits;

#ifdef USEQUEUE
	    if (++data_bits >= 8) { // filled all 8 bits
#else
	    if (++data_bits >= 32) { // filled all 32 bits
#endif

		if (tp->port == BUSY_PORT) gpio_set_level(SEND_BUSY_PIN, 0); // free

#ifdef USEQUEUE
	    	xQueueSend(tp->queue, &data, portMAX_DELAY);
#else	
	    	xRingbufferSend(tp->queue, &data, sizeof(data), portMAX_DELAY);
#endif
		if (tp->port == BUSY_PORT) gpio_set_level(SEND_BUSY_PIN, 1); // busy

	    	data = 0;
		data_bits = 0;

	    }
	}
    }

    if (data_bits > 0) { // there is a fraction of a byte

	if (tp->port == BUSY_PORT) gpio_set_level(SEND_BUSY_PIN, 0); // free

#ifdef USEQUEUE
	xQueueSend(tp->queue, &data, portMAX_DELAY);
#else
	int byte_size = (data_bits + 7) / 8;
	xRingbufferSend(tp->queue, &data, byte_size, portMAX_DELAY);
#endif

	if (tp->port == BUSY_PORT) gpio_set_level(SEND_BUSY_PIN, 1); // busy

    }
}
#else

static void packet_send_split(tcb_t *tp, uint8_t *buf[2], size_t size[2])
{
#ifdef USEQUEUE
    uint8_t data = 0; // data to modem
#else
    uint32_t data = 0; // data to modem
#endif
    int data_bits = 0; // number of bits
    int count_ones = 0;
    int insert_zero = false;
    int do_bitstuffing = true; // 1: do bit stuffing, 0: do not
    uint16_t fcs;

    int packet_size = size[0];
    if (buf[1]) packet_size += size[1];

    if (packet_size <= 0) return;

    //ESP_LOGI(TAG, "packet_send_split(), size = %d, port = %d", packet_size, tp->port);

    fcs = crc16_le(0, buf[0], size[0]); // CRC-16/X.25
    if (buf[1]) fcs = crc16_le(fcs, buf[1], size[1]);

#define AX25_FLAG 0x7e

    // send start flag
    static const uint8_t flag = AX25_FLAG;

#ifdef USEQUEUE
    xQueueSend(tp->queue, &flag, portMAX_DELAY);
#else
    xRingbufferSend(tp->queue, &flag, sizeof(flag), portMAX_DELAY);
#endif

    //ESP_LOGI(TAG, "send start flag, port = %d", tp->port);

    for (int i = 0; i < packet_size + 2; i++) { // send buf[0..1], FCS and End flag
	uint32_t bitq; // bit queue
	int bitq_bits; // number of bits

	if (i < size[0]) { // send buf[0] data
	    bitq = buf[0][i];
	    bitq_bits = 8;
	} else if (i < packet_size) { // send buf[1] data
	    bitq = buf[1][i - size[0]];
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

#ifdef USEQUEUE
	    if (++data_bits >= 8) { // filled all 8 bits
	    	xQueueSend(tp->queue, &data, portMAX_DELAY);
#else
	    if (++data_bits >= 32) { // filled all 32 bits
	    	xRingbufferSend(tp->queue, &data, sizeof(data), portMAX_DELAY);
#endif

	    	data = 0;
		data_bits = 0;

	    }

	} // while (bitq_bits-- > 0)

    } // for (i = 0; ..

    if (data_bits > 0) { // there is data to be sent

#ifdef USEQUEUE
	xQueueSend(tp->queue, &data, portMAX_DELAY);
#else
	int byte_size = (data_bits + 7) / 8;
	xRingbufferSend(tp->queue, &data, byte_size, portMAX_DELAY);

    	//ESP_LOGI(TAG, "packet_send_split(): send extra %d bytes, port = %d", byte_size, tp->port);
#endif

    }

}

#endif

static void send_task(void *arg)
{
    tcb_t *tp = (tcb_t *)arg; // pointer to TCB
    unsigned int seed = (unsigned int)esp_timer_get_time();

#define BAUD_RATE 1200
#define TIME_UNIT 10 // msec
#define TIME_UNIT_BITS (BAUD_RATE * TIME_UNIT / 1000)

    while (1) {

#define TXD_BYTES(t) ((t * 12 + 7) / 8)

	uint8_t *item[2];
	size_t itemsize[2];

	// receive from queue
	if (xRingbufferReceiveSplit(tp->ringbuf, (void **)&item[0], (void **)&item[1], &itemsize[0], &itemsize[1], portMAX_DELAY) != pdTRUE) {
	    ESP_LOGW(TAG, "xRingbufferReceiveSplit() fail, port = %d", tp->port);
	    continue;
	}

	//ESP_LOGI(TAG, "xRingbufferReceiveSplit(), size = %d, port = %d", itemsize[0] + (item[1]) ? itemsize[1] : 0, tp->port);

#ifdef FX25TNCR2
	gpio_set_level(tp->sta_led_pin, 1);
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
#ifdef M5ATOM
	    m5atom_led_set_level(M5ATOM_LED_RED, 1);
#endif
#ifndef M5STICKC_AUDIO
	    gpio_set_level(tp->ptt_pin, 1);
#endif
	    tp->ptt = true;

	    ESP_LOGD(TAG, "PTT on, gpio = %d, port = %d", tp->ptt_pin, tp->port);
	    
	    ESP_LOGD(TAG, "sending preamble, TXDELAY = %d, bytes = %d, port = %d", tp->TXDELAY, TXD_BYTES(tp->TXDELAY), tp->port);

	    // wait for TXDELAY
	    for (int i = 0; i < TXD_BYTES(tp->TXDELAY); i++) {
		uint8_t data = 0x7e; // flag (7E)
		//uint8_t data = 0x55; // flag (7E)
#ifdef USEQUEUE
		xQueueSend(tp->queue, &data, portMAX_DELAY);
#else
		xRingbufferSend(tp->queue, &data, sizeof(data), portMAX_DELAY);
#endif
	    }

	    //ESP_LOGI(TAG, "sent preamble, port = %d", tp->port);

	} // if (!tp->ptt)

	packet_send_split(tp, item, itemsize);

	// return item
	vRingbufferReturnItem(tp->ringbuf, item[0]);
	if (item[1]) vRingbufferReturnItem(tp->ringbuf, item[1]);

    } // while (1)
}

#define RINGBUF_SIZE (1024 * 2)

void send_init(tcb_t tcb[])
{
#define TX_QUEUE_SIZE 256

#define SEND_TASKS 6

    for (int i = 0; i < TNC_PORTS; i++) {

	// output queue
#ifdef USEQUEUE
    	tcb[i].queue = xQueueCreate(TX_QUEUE_SIZE, sizeof(uint8_t));
#else
    	tcb[i].queue = xRingbufferCreate(TX_QUEUE_SIZE, RINGBUF_TYPE_BYTEBUF);
#endif
    	assert(tcb[i].queue != NULL);

	// input queue
	tcb[i].ringbuf = xRingbufferCreate(RINGBUF_SIZE, RINGBUF_TYPE_ALLOWSPLIT);
    	assert(tcb[i].ringbuf != NULL);

	// create send task
    	assert(xTaskCreatePinnedToCore(send_task, "send task", 4096, &tcb[i], tskIDLE_PRIORITY + 2, &tcb[i].task, 1) == pdPASS);
    }

}

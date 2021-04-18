#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "driver/adc.h"
#include "esp32/rom/crc.h"
#include "driver/uart.h"

#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "config.h"

#include "tnc.h"
#include "bell202.h"
#include "i2s_adc.h"
#include "kiss.h"

#ifdef M5STICKC
#include "lcd_packet.h"
#endif

#define TAG "uart"

RingbufHandle_t uart_rb;
TaskHandle_t task;

static kcb_t kcb = {
    .data_state = DATA_IDLE,
    .data_size = 0,
#ifdef TEXT_MODE
    .monitor_mode = true,
#else
    .monitor_mode = false,
#endif
};


static const uart_port_t uart_num = UART_NUM_0;

#define AX25_ADDR_LEN 7
#define AX25_MIN_PKT_SIZE (AX25_ADDR_LEN * 2 + 1 + 2) // src addr + dst addr + Control + FCS
#define AX25_SSID_MASK 0x0f

#define BUF_SIZE 16

static char buf[BUF_SIZE];

static void uart_putchar(int c)
{
    char ch = c;

    if (ch >= ' ' && ch <= '~') {
		uart_write_bytes(uart_num, &ch, sizeof(ch));
    } else {
		uart_write_bytes(uart_num, buf, snprintf(buf, BUF_SIZE, "<%02x>", ch));
    }
}

static void uart_put_ssid(int ssid)
{
    int n = ssid & AX25_SSID_MASK;

    if (n > 0) uart_write_bytes(uart_num, buf, snprintf(buf, BUF_SIZE, "-%d", n));
}

static void uart_put_number(int num)
{
    uart_write_bytes(uart_num, buf, snprintf(buf, BUF_SIZE, "%d", num));
}

static void ax25_dump_packet(uint8_t *item, size_t size)
{
    int i;
    int in_addr = true;
    int len = size;
    //uint8_t *data = (uint8_t *)item[0];
   
    if (len < AX25_MIN_PKT_SIZE) return;

    //fcs = data[len - 1] << 8 | data[len - 2];
    //crc = crc16_le(0, data, len - 2);
    //if (fcs != crc) return;

    //printf("%d:%d:", tp->port, ++tp->pkts);

    static int pkts[TNC_PORTS]; // packet counter

    i = item[0] >> 4; // port number
    uart_put_number(i); // port number
    uart_putchar(':');
    uart_put_number(++pkts[i]);
    uart_putchar(':');
    uart_put_number(len);
    uart_putchar(':');

    for (i = 1; i < len; i++) {
		int c;

		c = item[i];

		if (in_addr) {
		    in_addr = (c & 1) == 0;
	    	c >>= 1; // for addr field shift

	    	if ((i - 1) % AX25_ADDR_LEN == AX25_ADDR_LEN - 1) { // SSID
				uart_put_ssid(c);
				uart_putchar(in_addr ? ',' : ':');
			} else {
				if (c != ' ') uart_putchar(c);
	    	}
		} else {
	    	uart_putchar(c);
		}
    }
    uart_write_bytes(uart_num, "\r\n", 2);
    //printf("<%02x%02x>\n", data[len-1], data[len-2]);
}

#define KISS_FEND 0xc0
#define KISS_FESC 0xdb
#define KISS_TFEND 0xdc
#define KISS_TFESC 0xdd

// make kiss frame and send it to uart
static void kiss_output_packet(uint8_t *item, size_t size)
{
	uint8_t *buf = item;
	if (buf == NULL) return;

    // frame start
    uart_write_bytes(uart_num, "\xc0", 1); // FEND

	size_t len = size;
	int top = 0;
	int i;

	for (i = 0; i < len; i++) {

	    uint8_t c = buf[i];

	    switch (c) {
			case KISS_FEND:
		    	buf[i] = KISS_FESC;
		    	/* FALLTHROUGH */

			case KISS_FESC:
		    	uart_write_bytes(uart_num, (void *)&buf[top], i + 1 - top);

		    	if (c == KISS_FEND) {
		    		buf[i] = KISS_TFEND;
		    	} else { // c == KISS_FESC
					buf[i] = KISS_TFESC;
		    	}
		    	top = i;
	    }

	}

	if (i - top > 0) {
	    uart_write_bytes(uart_num, (void *)&buf[top], i - top);
	}

    // frame end
    uart_write_bytes(uart_num, "\xc0", 1); // FEND
}

#define TCP_CONNS 8

typedef struct TCPCB {
    RingbufHandle_t ringbuf;
} tcpcb_t;

static uint8_t tcp_conn = false;
static tcpcb_t tcpcb[TCP_CONNS];

//static RingbufHandle_t tcp_ringbuf = NULL;

int uart_add_ringbuf(RingbufHandle_t rb)
{
    for (int i = 0; i < TCP_CONNS; i++) {

    	if (tcpcb[i].ringbuf == NULL) {
	    	tcpcb[i].ringbuf = rb;
	    	tcp_conn = true;

	    	return true;
		}
    }

    return false;
}

int uart_delete_ringbuf(RingbufHandle_t rb)
{
    for (int i = 0; i < TCP_CONNS; i++) {

    	if (tcpcb[i].ringbuf == rb) {
	    	tcpcb[i].ringbuf = NULL;

	    	return true;
		}
    }

    return false;
}

static void tcp_output_packet(void *item, size_t size)
{
    int cnt = 0;

    for (int i = 0; i < TCP_CONNS; i++) {
		if (tcpcb[i].ringbuf) {
		    if (xRingbufferSend(tcpcb[i].ringbuf, item, size, 0) != pdTRUE) { // nowait
				ESP_LOGW(TAG, "tcp_output_packet(): xRingbufferSend() fail");
	    	}
	    	cnt++;
		}
    }
    if (cnt == 0) tcp_conn = false;
}

#define UDP_CONNS 8

typedef struct UDPCB {
    struct netconn *conn;
    TickType_t expire;
    ip_addr_t addr;
    uint16_t port;
} udpcb_t;

static udpcb_t udpcb[UDP_CONNS];
static uint8_t udp_conn = false;

#define UDP_DEST_EXPIRE 600 // destination expire time (sec)

int uart_add_udp(struct netconn *conn, ip_addr_t *dst, int port)
{
    TickType_t now = xTaskGetTickCount();

    for (int i = 0; i < UDP_CONNS; i++) {
		udpcb_t *up = &udpcb[i];

		if (up->port && up->expire < now) up->port = 0; // expire

		if (ip_addr_cmp(&up->addr, dst) && up->port == port) {
		    // update expire time
	    	up->expire = now + (UDP_DEST_EXPIRE * 1000) / portTICK_PERIOD_MS;

	    	return false;
		}
    }

    for (int i = 0; i < UDP_CONNS; i++) {
		udpcb_t *up = &udpcb[i];

		if (up->port == 0) {
		    up->conn = conn;
	    	up->expire = now + (UDP_DEST_EXPIRE * 1000) / portTICK_PERIOD_MS;
	    	ip_addr_copy(up->addr, *dst);
	    	up->port = port;
	    	udp_conn = true;
	    
	    	return true;
		}
    }

    return false;
}

int uart_delete_udp(ip_addr_t *dst, int port)
{
    TickType_t now = xTaskGetTickCount();

    int cnt = 0;
    for (int i = 0; i < UDP_CONNS; i++) {
		udpcb_t *up = &udpcb[i];

		if (ip_addr_cmp(&up->addr, dst) && up->port == port) {
	    	up->port = 0;

	    	return true;
		}

		if (up->expire < now) {
	    	up->port = 0;
		}

		if (up->port) cnt++;
	}
	if (cnt == 0) udp_conn = false;

    return false;
}

void udp_output_packet(void *item, size_t size)
{
    struct netbuf *nbuf = netbuf_new();

    if (netbuf_ref(nbuf, item, size) != ERR_OK) {
		ESP_LOGW(TAG, "netbuf_ref() fail");

		netbuf_delete(nbuf);

		return;
    }

    int cnt = 0;
    TickType_t now = xTaskGetTickCount();
    for (int i = 0; i < UDP_CONNS; i++) {
		udpcb_t *up = &udpcb[i];

		if (up->port) {

	    	if (up->expire < now) { // expire
				up->port = 0;
				continue;
	    	}

	    	err_t err;
	    	if ((err = netconn_sendto(up->conn, nbuf, &up->addr, up->port)) != ERR_OK) {
				ESP_LOGW(TAG, "udp: netconn_sendto() fail, err = %d", err);
				if (err != ERR_MEM) up->port = 0;
				continue;
	    	}
	    	cnt++;
		}
    }
    if (cnt == 0) udp_conn = false;

    netbuf_delete(nbuf);
}

static void uart_task(void *p)
{
    RingbufHandle_t rb = (RingbufHandle_t)p;
    uint8_t *item;
    size_t size;

    while (1) {
		if ((item = xRingbufferReceive(rb, &size, portMAX_DELAY)) == NULL) {
		    ESP_LOGW(TAG, "xRingbufferReceive() fail");
	    	continue;
		}

		if (kcb.monitor_mode) {

		    // output ascii text format
	    	ax25_dump_packet(item, size);

		} else {

		    // output kiss frame
	    	kiss_output_packet(item, size);

		}

		if (tcp_conn) tcp_output_packet(item, size);
		if (udp_conn) udp_output_packet(item, size);
#ifdef M5STICKC
		lcd_dump_packet(item, size);
#endif

		vRingbufferReturnItem(rb, item);
    }
}

#define DATA_READ_BUF_SIZE 128

// read size bytes data form uart
static void uart_data_read(size_t size)
{
    static uint8_t buf[DATA_READ_BUF_SIZE];
    size_t remain = size;
    int len;
    int read_size;

    while (remain > 0) {

		if (remain > DATA_READ_BUF_SIZE) {
		    read_size = DATA_READ_BUF_SIZE;
		} else {
		    read_size = remain;
		}

		len = uart_read_bytes(uart_num, buf, read_size, portMAX_DELAY);
		if (len < 0) {
	    	ESP_LOGW(TAG, "uart_read_bytes() fail");
	    	return;
		}

		for (int i = 0; i < len; i++) {
		    kiss_process_char(&kcb, buf[i]);
		}

		remain -= len;
    }
}

static void uart_event_task(void *arg)
{
    QueueHandle_t uart_queue = (QueueHandle_t)arg;
    uart_event_t event;


    while (1) {
		if (xQueueReceive(uart_queue, &event, portMAX_DELAY) != pdTRUE) {
		    ESP_LOGW(TAG, "xQueueReceive(event_queue) fail");
	    	continue;
		}

		switch (event.type) {

		    case UART_DATA:
			//ESP_LOGI(TAG, "UART_DATA: size = %d", event.size);
			//uart_get_buffered_data_len(uart_num, &size);
			//ESP_LOGI(TAG, "uart_get_buffered_data_len: size = %d", size);
			uart_data_read(event.size);
#if 0
			for (int i = 0; i < event.size; i++) {
			    uint8_t tmp;
		    	uart_read_bytes(uart_num, &tmp, sizeof(tmp), portMAX_DELAY);
		    	ESP_LOGI(TAG, "UART_DATA: char = %02x, %c", tmp, (tmp >= ' ' && tmp <= '~') ? tmp : '.');
			}
#endif
			break;

	    	case UART_FIFO_OVF:
				ESP_LOGW(TAG, "UART_FIFO_OVF");
				uart_flush_input(uart_num);
				xQueueReset(uart_queue);
				break;

	    	case UART_BUFFER_FULL:
				ESP_LOGW(TAG, "UART_BUFFER_FULL");
				uart_flush_input(uart_num);
				xQueueReset(uart_queue);
				break;

	    	case UART_BREAK:
				ESP_LOGW(TAG, "UART_BREAK");
				break;

			case UART_PARITY_ERR:
				ESP_LOGW(TAG, "UART_PARITY_ERR");
				break;

	    	case UART_FRAME_ERR:
				ESP_LOGW(TAG, "UART_FRAME_ERR");
				break;

	    	case UART_PATTERN_DET:
				ESP_LOGW(TAG, "UART_PATTERN_DET");
				break;

	    	default:
				ESP_LOGW(TAG, "uart event type: %d", event.type);
		}
    }
}

#define UART_RB_SIZE (1024 * 8)
#define UART_RX_BUF_SIZE 1024
#define UART_TX_BUF_SIZE 1024
#define UART_EVENT_QUEUE_SIZE 16

void uart_init(void)
{
    uart_config_t uart_config = {
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		//.source_clk = UART_SCLK_APB,
    };
    QueueHandle_t event_queue;
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, UART_EVENT_QUEUE_SIZE, &event_queue, 0));

    assert(xTaskCreatePinnedToCore(uart_event_task, "uart event task", 1024 * 4, event_queue, tskIDLE_PRIORITY + 0, NULL, tskNO_AFFINITY) == pdPASS);

    uart_rb = xRingbufferCreate(UART_RB_SIZE, RINGBUF_TYPE_NOSPLIT);
    if (uart_rb == NULL) {
		ESP_LOGE(TAG, "xRingbufferCreate() fail");
		abort();
    }

    if (xTaskCreatePinnedToCore(uart_task, "uart task", 1024 * 4, uart_rb, tskIDLE_PRIORITY + 0, &task, tskNO_AFFINITY) != pdPASS) {
		ESP_LOGE(TAG, "xTaskCreatePinnedToCore() fail");
		abort();
    }
}

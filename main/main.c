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
#include "driver/gpio.h"
#include "driver/dac.h"
#include "driver/adc.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp32/rom/crc.h"
#include "esp_timer.h"
#include "esp_system.h"

#include "tnc.h"
#include "bell202.h"
#include "i2s_adc.h"
#include "decode.h"
#include "uart.h"
#include "timer.h"
#include "send.h"
#include "wifi.h"
#include "filter.h"

#include "config.h"
#include "send.h"

#ifdef M5STICKC
#include "m5stickc.h"
#include "m5tnc_logo.h"
#endif

#ifdef ENABLE_TCM3105
#include "tcm3105.h"
#endif

#ifdef CONFIG_BME280_EXISTS
#include "BME280.h"   /* for BME280 APRS transmit test */
#endif

#define TAG "main"

#ifdef BEACON

#define PACKET_SIZE (240 - 2) // remain the room for FCS

static int make_packet(tcb_t *tp, uint8_t *packet, int pkt_len)
{
    static const uint8_t address_field[] = {
    	'B' << 1, 'E' << 1, 'A' << 1, 'C' << 1, 'O' << 1, 'N' << 1,
    	0xe0, // command(V2), ssid
	    'N' << 1, 'O' << 1, 'C' << 1, 'A' << 1, 'L' << 1, 'L' << 1,
	    0x61, // command(V2), ssid, end of address field
	    0x03, // control
	    0xf0, // PID
    };
    //static uint8_t packet[PACKET_SIZE];
    uint8_t *p = packet;
    static long epoch = 0;
    struct timeval tv;
    long t;

    if (!epoch) {
	    gettimeofday(&tv, NULL);
	    epoch = tv.tv_sec;
    }

    gettimeofday(&tv, NULL);
    t = tv.tv_sec;

    memcpy(p, address_field, sizeof(address_field));
    p += sizeof(address_field);

    static uint32_t cnt = 0;
    cnt++;
    p += sprintf((char *)p, "seq=%d, time=%lds, prot=", cnt, t - epoch);
#ifdef FX25_ENABLE
    if (tp->fx25_parity == 0) {
        p += sprintf((char *)p, "AX.25 ");
    } else {
        p += sprintf((char *)p, "FX.25/%d ", tp->fx25_parity);
    }
#else
    p += sprintf((char *)p, "AX.25 ");
#endif
    p += snprintf((char *)p, pkt_len - (p - packet), "%s", BEACON_TEXT);

    return p - packet;
}

static void send_packet_task(void *arg)
{
    tcb_t *tp = (tcb_t *)arg;
    uint8_t *packet;
    int pkt_len;
    static char src_addr[CALLSIGN_LEN];
    
    make_address(src_addr, BEACON_CALLSIGN);

    while (1) {

	    vTaskDelay(BEACON_INTERVAL * 1000 / portTICK_PERIOD_MS);

        packet = malloc(PACKET_SIZE);
        if (packet == NULL) {
            ESP_LOGW(TAG, "malloc() fail");
            continue;
        }

	    pkt_len = make_packet(tp, packet, PACKET_SIZE);
	    memcpy(&packet[CALLSIGN_LEN], src_addr, CALLSIGN_LEN); // src address

#if 0
        send_packet(tp, packet, pkt_len, SEND_DEFAULT_PARITY, 0); // no wait (0), discard the packet if buffer full
#else
        // send the packet to all ports
        for (int i = 0; i < TNC_PORTS; i++) {
            send_packet(&tcb[i], packet, pkt_len, SEND_DEFAULT_PARITY, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
#endif
        free(packet);
#if 0
	    if (xRingbufferSend(tp->input_rb, packet, pkt_len, portMAX_DELAY) != pdTRUE) {
	        ESP_LOGW(TAG, "xRingbufferSend() fail, port = %d", tp->port);
	        continue;
	    }
#endif
    }
}
#endif // BEACON

void app_main(void)
{
    ESP_LOGI(TAG, 
#ifdef FX25TNCR1
        "FX.25 KISS TNC rev.1 (TCM3105 version)"
#endif
#ifdef FX25TNCR2
        "FX.25 KISS TNC rev.2 (SMD version)"
#endif
#ifdef FX25TNCr3
        "FX.25 KISS TNC rev.3 (6ports version)"
#endif
#ifdef FX25TNCR4
        "FX.25 KISS TNC rev.4 (softmodem and TCM3105 version)"
#endif
#ifdef M5STICKC
        "M5TNC M5StickC Plus version"
#endif
#ifdef M5ATOM
        "M5TNC M5Atom version"
#endif
    );

    ESP_LOGI(TAG, "TNC_PORTS = %d", TNC_PORTS);

#ifdef SOFTMODEM_PORTS
    ESP_LOGI(TAG, "SOFTMODEM_PORTS = %d", SOFTMODEM_PORTS);
#endif
#ifdef ENABLE_TCM3105
    ESP_LOGI(TAG, "ENABLE_TCM3105: true");
#ifdef TCM3105_ADC
    ESP_LOGI(TAG, "TCM3105_ADC: true");
#endif
#endif

#ifdef FX25TNCR4
    // initialize TCM3105 check pin
    ESP_LOGI(TAG, "GPIO_ENABLE_TCM3105_PIN GPIO = %d, value = %d", GPIO_ENABLE_TCM3105_PIN, gpio_get_level(GPIO_ENABLE_TCM3105_PIN));
#endif

#ifdef M5STICKC
    // M5StickC library initialize
    m5stickc_init();

    decode_image(lcd.spi, m5tnc_logo); // M5TNC logo

    static const uint8_t s[] = "M5TNC for M5StickC Plus\n";
    tty_write(s, sizeof(s) - 1);
#endif

    // initialize UART
    uart_init();


#ifdef ENABLE_SOFTMODEM
    ESP_LOGI(TAG, "enable softmodem");

    // initialize Bell202 modem
    bell202_init();

    // initialize I2S
    i2s_init(tcb);
#endif

    // initialize send task
    send_init(tcb);

    // initialize SIGMADELTA & TIMER
    timer_initialize(tcb, TNC_PORTS);

#ifdef USE_WIFI
    // WiFi: connect to AP
    wifi_start();
#endif

#ifdef M5ATOM
    // M5Atom RGB LED
    m5atom_led_init();

    for (int i = (1 << M5ATOM_LED_COLOR_MAX) - 1; i >= 0; i--) {
	    m5atom_led_set_level(M5ATOM_LED_GREEN, (i >> M5ATOM_LED_GREEN) & 1);
	    m5atom_led_set_level(M5ATOM_LED_RED, (i >> M5ATOM_LED_RED) & 1);
	    m5atom_led_set_level(M5ATOM_LED_BLUE, (i >> M5ATOM_LED_BLUE) & 1);
	    vTaskDelay(200 / portTICK_PERIOD_MS);
    }
#endif

#define CAPTURE_QUEUE_SIZE 1024 // capture queue size

#if defined(ENABLE_TCM3105) && !defined(TCM3105_ADC)
    // initialize TCM3105 check pin
    cap_queue = xQueueCreate(CAPTURE_QUEUE_SIZE, sizeof(uint32_t));
    assert(cap_queue != NULL);
#endif

    // initialize TNC control block and start TNC decode task
    tnc_init(tcb, TNC_PORTS);

#ifdef ENABLE_TCM3105
    // initialize TCM3105 support routines
    tcm3105_init();
#endif

#ifdef BEACON
    // send beacon packet
    assert(xTaskCreatePinnedToCore(send_packet_task, "send packet", 4096, &tcb[0], tskIDLE_PRIORITY, NULL, 0) == pdPASS);
#endif

#ifdef CONFIG_BME280_EXISTS
    //  BME280 APRS transmit test
    ESP_ERROR_CHECK(i2c_master_init()); /* I2C interface initialize */
    BME280_setup(); /* BME280 setup */
    vTaskDelay(2 * 1000 / portTICK_PERIOD_MS); /* wait more than BME280 measurment period */
    xTaskCreate(BME280_aprs_task, "BME280_aprs_task_0", 1024 * 2, (void *)0, 10, NULL);
#endif

    vTaskDelete(NULL);
}

/* WiFi station Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wps.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
//#include "esp_heap_caps.h"

#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "config.h"

#ifdef USE_WIFI

#include "wifi.h"
#include "uart.h"
#include "tnc.h"
#include "kiss.h"

#ifdef CONFIG_ESP_WIFI_SOFTAP
#include "softap.h"
#endif

//#define USE_WPS 1

/* The examples use WiFi configuration that you can set via 'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event 
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;

static const char *TAG = "wifi";

static int s_retry_num = 0;

#define WPS_TEST_MODE WPS_TYPE_PBC

#define PIN2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5], (a)[6], (a)[7] 
#define PINSTR "%c%c%c%c%c%c%c%c"

static esp_wps_config_t config = WPS_CONFIG_INIT_DEFAULT(WPS_TEST_MODE);

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        s_retry_num = 0;
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
	ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
        ESP_LOGI(TAG,"connect to the AP fail");
	ESP_LOGI(TAG, "ssid: %s", event->event_info.disconnected.ssid);
        {
            if (s_retry_num < 10) {
                ESP_ERROR_CHECK(esp_wifi_connect());
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                s_retry_num++;
                ESP_LOGI(TAG,"retry to connect to the AP");
            } else {
#if 0
		ESP_ERROR_CHECK(esp_wifi_wps_enable(&config));
		ESP_ERROR_CHECK(esp_wifi_wps_start(0));
		ESP_LOGI(TAG, "retry to wps...");
#endif
                s_retry_num = 0;
	    }
            break;
        }
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
	ESP_LOGI(TAG, "SYSTEM_EVENT_STA_WPS_ER_SUCCESS");
	ESP_ERROR_CHECK(esp_wifi_wps_disable());
        ESP_ERROR_CHECK(esp_wifi_connect());
	break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
	ESP_LOGI(TAG, "SYSTEM_EVENT_STA_WPS_ER_FAILED/TIMEOUT");
	ESP_ERROR_CHECK(esp_wifi_wps_disable());
	ESP_ERROR_CHECK(esp_wifi_wps_enable(&config));
	ESP_ERROR_CHECK(esp_wifi_wps_start(0));
	break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
	ESP_LOGI(TAG, "SYSTEM_EVENT_STA_WPS_ER_PIN");
	ESP_LOGI(TAG, "WPS_PIN = "PINSTR, PIN2STR(event->event_info.sta_er_pin.pin_code));
	break;
    default:
        break;
    }
    return ESP_OK;
}

static void callback(struct netconn *conn, enum netconn_evt event, u16_t len)
{
    static const char *evt_name[] = {
	    "RCVPLUS",
	    "RCVMINUS",
	    "SENDPLUS",
	    "SENDMINUS",
	    "ERROR",
    };

    ESP_LOGD(TAG, "event: %s, len: %d", evt_name[event], len);
}

#define FEND 0xc0
#define FESC 0xdb
#define TFEND 0xdc
#define TFESC 0xdd

#define RECVTIMEOUT (60*1000)	// 60 sec

// do process incoming TCP connection
void tcp_client(struct netconn *conn)
{
    struct netbuf *nbuf;
    err_t xErr;
    kcb_t *kp = malloc(sizeof(kcb_t));

    if (kp == NULL) {
	ESP_LOGW(TAG, "tcp_client(): malloc fail");
	return; 
    }

    kp->wait = portMAX_DELAY; // do flow control for TCP client
    //kp->wait = 0; // do flow control for TCP client
    kp->data_state = DATA_IDLE;

    //netconn_set_recvtimeout(conn, RECVTIMEOUT);
    while ((xErr = netconn_recv(conn, &nbuf)) == ERR_OK) {
	ESP_LOGD(TAG, "netconn_recv(): %d byte", netbuf_len(nbuf));

	// do process KISS protocol (SLIP)
	do {
	    uint8_t *data;
	    u16_t len;
	    int i;

	    netbuf_data(nbuf, (void **)&data, &len);

	    for (i = 0; i < len; i++) {
		kiss_process_char(kp, data[i]);
	    }

	} while (netbuf_next(nbuf) >= 0);

	netbuf_delete(nbuf);
	ESP_LOGD(TAG, "netbuf_delete()");
    }
    ESP_LOGD(TAG, "netconn_recv(): %d", xErr);

    free(kp);
}

#ifndef TCP_MSS
#define TCP_MSS 1440
#endif
#define KISS_TCP_MSS TCP_MSS
#define AF_TX_BUF_SIZE KISS_TCP_MSS

static void tcp_send_split(struct netconn *conn, uint8_t *item[], size_t size[])
{
    static char af_buf[AF_TX_BUF_SIZE];
    int ai = 0;
    int bi = 0;
    uint8_t c;
    uint8_t next_byte = 0;
    //int usec;
    uint8_t *buf;
    int i;
    int len;
    int total = 0;

#ifndef CONFIG_TNC_DEMO_MODE
    af_buf[ai++] = FEND;
    //af_buf[ai++] = 0x00; // channel 0, data frame
#endif

    for (i = 0; i < 2; i++) {

	buf = item[i];
	if (buf == NULL) break;
	len = size[i];
	total += len;
	bi = 0;

        while (bi < len) {
       
	    if (next_byte) {
	        c = next_byte;
	        next_byte = 0;
	    } else {
	        c = buf[bi];
	    }

#ifdef CONFIG_TNC_DEMO_MODE

#define LF '\n'
#define CR '\r'

	    if (c == LF) {
		c = LF;
		next_byte = CR;
	    } else {
		bi++;
	    }
#else
	    if (c == FEND) {
	        c = FESC;
	        next_byte = TFEND;
	    } else if (c == FESC) {
	        c = FESC;
		next_byte = TFESC;
	    } else {
	        bi++;
	    }
#endif

	    af_buf[ai++] = c;

	    if (ai >= AF_TX_BUF_SIZE) {
	        netconn_write(conn, af_buf, ai, NETCONN_COPY);
	        ai = 0;
	    }
	}
    }

#ifndef CONFIG_TNC_DEMO_MODE
    af_buf[ai++] = FEND;
#endif

    // send KISS frame to TCP client
    netconn_write(conn, af_buf, ai, NETCONN_COPY);
}

//static RingbufHandle_t tcp_ringbuf = NULL;

#define TCP_RINGBUF_SIZE (2048)

struct RINGCONN {
    RingbufHandle_t ringbuf;
    struct netconn *conn;
};

void tcp_writer_task(void *p)
{
    struct RINGCONN *rcp = (struct RINGCONN *)p;
    uint8_t *item[2];
    size_t size[2];

    while (1) {
        if (xRingbufferReceiveSplit(rcp->ringbuf, (void **)&item[0], (void **)&item[1], &size[0], &size[1], portMAX_DELAY) == pdTRUE) { 

	    if (item[0]) {
	        tcp_send_split(rcp->conn, item, size);
	        vRingbufferReturnItem(rcp->ringbuf, item[0]);
	        if (item[1]) vRingbufferReturnItem(rcp->ringbuf, item[1]);
	    }
	} else {
	    ESP_LOGW(TAG, "xRingbufferReceiveSplit() fail");
	}
    }
}

void tcp_reader_task(void *p)
{
    struct netconn *newconn = (struct netconn *)p;
    struct RINGCONN rc;
    TaskHandle_t tcp_writer = NULL;

    rc.conn = newconn;
    rc.ringbuf = xRingbufferCreate(TCP_RINGBUF_SIZE, RINGBUF_TYPE_ALLOWSPLIT);
    if (rc.ringbuf == NULL) {
	ESP_LOGW(TAG, "xRingbufferCreate() fail");
    }
    if (rc.ringbuf != NULL) {

	// register ringbuf
	if (uart_add_ringbuf(rc.ringbuf)) {
	    if (xTaskCreatePinnedToCore(tcp_writer_task, "tcp_write", 1024*4, &rc, tskIDLE_PRIORITY+0, &tcp_writer, tskNO_AFFINITY) == pdPASS) {
		tcp_client(newconn);
		ESP_LOGD(TAG, "tcp_client() returned");
		vTaskDelete(tcp_writer); // stop writer task
	    }
	    // deregister ringbuf
	    uart_delete_ringbuf(rc.ringbuf);
	}
	vRingbufferDelete(rc.ringbuf);
    }

    netconn_close(newconn);
    netconn_delete(newconn);
    ESP_LOGD(TAG, "netconn_delete()");

    vTaskDelete(NULL); // delete this task
}

static void wifi_task(void *p)
{
    err_t xErr;
    struct netconn *conn;
    struct netconn *newconn;
    TaskHandle_t tcp_reader = NULL;

    while (1) {
#ifndef CONFIG_ESP_WIFI_SOFTAP
	xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
	ESP_LOGI(TAG, "WiFi connected");
#endif

	// TCP
	conn = netconn_new_with_callback(NETCONN_TCP, callback);
	if (conn == NULL) continue;
	xErr = netconn_bind(conn, IP_ADDR_ANY, TCP_PORT);
	if (xErr != ERR_OK) continue;
	xErr = netconn_listen(conn);
	if (xErr != ERR_OK) continue;

	while (1) {
	    xErr = netconn_accept(conn, &newconn);
	    ESP_LOGD(TAG, "netconn_accept(): %d", xErr);
	    if (xErr != ERR_OK) continue;

	    // create tcp reader task
	    if (xTaskCreatePinnedToCore(tcp_reader_task, "tcp_read", 1024*4, newconn, tskIDLE_PRIORITY+0, &tcp_reader, tskNO_AFFINITY) != pdTRUE) {
		ESP_LOGD(TAG, "xTaskCreate(tcp_reader_task) fail");
	    }
#if 0
	    tcp_ringbuf = xRingbufferCreate(TCP_RINGBUF_SIZE, RINGBUF_TYPE_ALLOWSPLIT);
	    if (tcp_ringbuf) {
		// register ringbuf 
		uart_add_ringbuf(tcp_ringbuf);

	    	if (xTaskCreate(tcp_writer_task, "tcp_write", 1024*4, newconn, tskIDLE_PRIORITY, &tcp_writer) == pdPASS) {

	            tcp_client(newconn);
		    ESP_LOGI(TAG, "tcp_client() returned");

		    vTaskDelete(tcp_writer); // stop writer task 
	        }
	        netconn_close(newconn);
	        netconn_delete(newconn);
	        ESP_LOGI(TAG, "netconn_delete()");

		// deregister ringbuf
		uart_delete_ringbuf();

		vRingbufferDelete(tcp_ringbuf);
	    }
#endif
	}
    }
}

static void udp_task(void *arg)
{
    err_t xErr;
    struct netconn *conn;
    struct netbuf *nbuf;

    while (1) {
#ifndef CONFIG_ESP_WIFI_SOFTAP
	xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
	ESP_LOGI(TAG, "udp: WiFi connected");
#endif

	// UCP
	if ((conn = netconn_new_with_callback(NETCONN_UDP, callback)) == NULL) {
	    ESP_LOGW(TAG, "udp: netconn_new() fail");
	    vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
	    continue;
	}
	if (netconn_bind(conn, IP_ADDR_ANY, UDP_PORT) != ERR_OK) {
	    ESP_LOGW(TAG, "udp: netconn_bind() fail");
	    netconn_delete(conn);
	    vTaskDelay(10 * 1000 / portTICK_PERIOD_MS);
	    continue;
	}

    	while ((xErr = netconn_recv(conn, &nbuf)) == ERR_OK) {
	    ESP_LOGD(TAG, "netconn_recv(): %d byte", netbuf_len(nbuf));
	    static kcb_t kcb;
	    kcb.data_size = 0;
	    kcb.data_state = DATA_INFRAME;

	    // add destination IP and port No. for UDP client
	    uart_add_udp(conn, netbuf_fromaddr(nbuf), netbuf_fromport(nbuf));

	    size_t len = netbuf_len(nbuf);
	    if (len <= DATA_BUF_SIZE) {

	    	do {
		    uint8_t *data;
		    u16_t len;

		    netbuf_data(nbuf, (void **)&data, &len);

#if 0
		    for (int i = 0; i < len; i++) {
			int c = data[i];

			if (c >= ' ' && c <= '~') printf("%c", c);
			else printf("<%02x>", c);
		    }
		    printf("\n");
#endif

		    memcpy(&kcb.data_buf[kcb.data_size], data, len);
		    kcb.data_size += len;

	    	} while (netbuf_next(nbuf) >= 0);

		// send received packet to the air
	    	kiss_process_frame(&kcb);

	    } else {
		ESP_LOGW(TAG, "incoming UDP packet too larg, size = %d", len);
	    }

	    netbuf_delete(nbuf);
	    ESP_LOGD(TAG, "netbuf_delete()");

	}
	ESP_LOGW(TAG, "udp: netconn_recv(): %d", xErr);
	netconn_delete(conn);

    }

}

#define HTTP_PORT 80
#define HTTP_BUFSIZ 200 

extern int total_pkts;
extern int fx25_decode_pkts;
extern int ax25_decode_pkts;
extern int tag_error_pkts;
extern int rs_decode_err;
extern int fx25_fcs_err;

#ifdef CONFIG_ESP_WIFI_SOFTAP
static const char http_tag[] = "http";

static void http_task(void *p)
{
    err_t xErr;
    struct netconn *conn;
    struct netconn *newconn;
    static char http_head[] = "HTTP/1.0 200 OK\r\n\r\n<html><head><title>Statistics</title></head><body>";
    static char http_buf[HTTP_BUFSIZ];
    int len;

    conn = netconn_new(NETCONN_TCP);
    if (conn == NULL) {
	ESP_LOGD(http_tag, "netconn_new() fail");
	vTaskDelete(NULL); // terminate task
    }

    xErr = netconn_bind(conn, IP_ADDR_ANY, HTTP_PORT);
    if (xErr != ERR_OK) {
	ESP_LOGD(http_tag, "netconn_bind() fail");
	vTaskDelete(NULL);
    }

    xErr = netconn_listen(conn);
    if (xErr != ERR_OK) {
	ESP_LOGD(http_tag, "netconn_listen() fail");
	vTaskDelete(NULL);
    }

    while (1) {
	xErr = netconn_accept(conn, &newconn);
	if (xErr != ERR_OK) continue;

	netconn_write(newconn, http_head, sizeof(http_head) - 1, NETCONN_NOCOPY);

	len = snprintf(http_buf, HTTP_BUFSIZ,
			"Total: %d pkts<br>"
			"FX25: %d pkts<br>"
			"AX25: %d pkts<br>"
			"FX25%%: %d %%<br>"
			"AX25%%: %d %%<br>"
			"Tag err: %d<br>"
			"RS err: %d<br>"
			"FCS err: %d<br>"
			"</body></html>",
			total_pkts, fx25_decode_pkts, ax25_decode_pkts,
			fx25_decode_pkts*100/total_pkts, ax25_decode_pkts*100/total_pkts,
			tag_error_pkts, rs_decode_err, fx25_fcs_err);

	netconn_write(newconn, http_buf, len, NETCONN_COPY);
	netconn_close(newconn);
    }
}
#endif

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
        },
    };

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

#ifdef USE_WPS
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_LOGI(TAG, "start wps...");

    ESP_ERROR_CHECK(esp_wifi_wps_enable(&config));
    ESP_ERROR_CHECK(esp_wifi_wps_start(0));
#endif
}

void wifi_start(void)
{
#ifndef CONFIG_ESP_WIFI_SOFTAP
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
#else
    softap_init();
    configASSERT(xTaskCreatePinnedToCore(http_task, "http_task", 4096, NULL, tskIDLE_PRIORITY+1, NULL, tskNO_AFFINITY) == pdPASS);
#endif
    assert(xTaskCreatePinnedToCore(wifi_task, "wifi_task", 4096, NULL, tskIDLE_PRIORITY+0, NULL, tskNO_AFFINITY) == pdPASS);
    assert(xTaskCreatePinnedToCore(udp_task, "udp_task", 4096, NULL, tskIDLE_PRIORITY+0, NULL, tskNO_AFFINITY) == pdPASS);
}
#endif // USE_WIFI

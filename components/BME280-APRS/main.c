/*
 * FX.25 KISS TNC for ESP-WROOM-32 WiFi module
 *
 *   Bell 202 modem chip: TCM3105
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <sys/time.h>

/* #include <freertos/freertos.h> */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_clk.h>
#include <esp_sleep.h>
#include <driver/uart.h>
#include <esp_log.h>

#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "config.h"
#include "rmt.h"
#include "gpio.h"
#include "mcpwm.h"
#include "fx25_encode.h"
#include "fx25_decode.h"
#include "rs.h"
#include "ax25.h"
#include "fx25tag.h"
#include "bit_stuffing.h"
#include "ax25_decode.h"
#include "uart.h"
#include "wifi.h"
#include "tx_cntl.h"

#include "BME280.h"   /* for BME280 APRS transmit test */


#ifdef CONFIG_ESP_DAC
#include "dac.h"
#endif

#ifdef CONFIG_ESP_LEDC
#include "ledc.h"
#endif

#include "packet_table.h"

//#define USE_NOTIFICATION 1 // use xTaskNotification() for TX and RX synchronization
//#define USE_ACK 1
#define ACK_QUEUE_LEN 1
#define TAG "main"

#define CHECK_MISS 1	// check error correction miss

//#define FX25_DATA_SIZE (4 + 8 + 255 + 1) // pre + tag + rs + post
#define PARITY_SYMBOLS 16 // len of parity

#define BIT_TIME ((80000000 + BAUD/2) / BAUD) // APB clock 80MHz, 1200bps
#define BIT_TIME2 (BIT_TIME / 2)

#define FX25_BUF_SIZE (4 + 8 + 255*5 + 1) // maximum FX.25 packet size, first byte is type indicator
#define AX25_BUF_SIZE (239*5 - 2)
#define AX25_NRZI_SIZE (((AX25_BUF_SIZE * 8 * 6 + 4) / 5 + 7) / 8)

#define RXD_QUEUE_LEN 1024

#define BIT_LEN_MAX 32 // threthold for packet termination

// KISS mode asynchronouse frame format
#define FEND 0xc0	// frame end
#define FESC 0xdb	// frame escape
#define TFEND 0xdc	// transposed frame end
#define TFESC 0xdd	// transposed trame escape

#define UART_NUM UART_NUM_0
#define PRINT_ESC 1	// escape non-printable character for debug

//static int tx_pkts = 0;
//static int ax25_pkts = 0;
//static int ax25_nrzi_pkts = 0;
static int ax25_nrzi_ec_try = 0;
static int ax25_nrzi_ec_ok = 0;
static int ax25_nrzi_ec_miss = 0;
//static int fx25_pkts = 0;
static int fx25_rs_try = 0;
static int fx25_rs_ok = 0;
static int fx25_rs_miss = 0;
static int ax25_nrzi_bits = 0;
static int fx25_bits = 0;

#ifdef CHECK_MISS
static uint8_t *test_packet;
static int test_packet_len;
static uint8_t *test_fx25_buf;
static int test_fx25_len;
#endif

#ifdef CONFIG_TNC_BEACON
static TaskHandle_t tx_task_handle;
#endif

#ifdef USE_ACK
static QueueHandle_t ack_queue;
#endif

#if 0
static void print_buf(uint8_t buf[], int len)
{
    for (int i = 0; i < len; i++) {
		printf("%02x ", buf[i]);
    }
    printf("\n");
}
#endif

#ifdef CONFIG_TNC_STATISTICS
static struct FX25_STAT {
    uint8_t from[7];
    int count;
    int fcs_error;
    int rs_decode;
    int len;
    int rs_code;
    int rs_info;
    int error_bytes;
    int ax25_ok;
    int ax25_cnt;
} fx25_stat = { "", 0, 0, 0, 0, 0, 0, 0, 0, 0 };
#endif

static void print_diff(uint8_t buf0[], uint8_t buf1[], int len)
{
    for (int i = 0; i < len; i++) {
		int c0 = buf0[i];
		int c1 = buf1[i];

		if (c0 == c1) {
			printf("%02x ", c0);
		} else {
			printf("(%02x>%02x) ", c0, c1);
		}
    }
    printf("\n");
}

#ifdef CONFIG_TNC_DEMO_MODE

int ax25_decode_pkts = 0;
int fx25_decode_pkts = 0;
int total_pkts = 0;
static uint8_t old_seq = 0x80; 
extern int tag_error_pkts;
int rs_decode_err = 0;
int fx25_fcs_err = 0;

#define INFO_BUF_SIZE 256

static char info_buf[INFO_BUF_SIZE];
static int info_len;

static void packet_print(uint8_t buf[], int len)
{
    uint8_t in_addr = 1;

    for (int i = 0; i < len; i++) {
		uint8_t c = buf[i];
		uint8_t d = c >> 1;

		if (in_addr) { // address field
			if (c & 1) in_addr = 0; // end of addr
			if (i % 7 == 6) { // SSID
				uint8_t ssid = d & 0x0f;

				if (ssid > 0) info_len += snprintf(info_buf + info_len, INFO_BUF_SIZE - info_len, "-%d", ssid);
				info_len += snprintf(info_buf + info_len, INFO_BUF_SIZE - info_len, "%c", (in_addr) ? ',' : ':');
			} else if (((d >= 'A') && (d <= 'Z')) || ((d >= '0') && (d <= '9'))) { // 
				info_len += snprintf(info_buf + info_len, INFO_BUF_SIZE - info_len, "%c", d);
			} else if (d != ' ') {
				info_len += snprintf(info_buf + info_len, INFO_BUF_SIZE - info_len, "<%02x>", c);
			}
		} else {
			if ((c >= ' ') && (c <= '~')) { // printable char.
				info_len += snprintf(info_buf + info_len, INFO_BUF_SIZE - info_len, "%c", c);
			} else {
				info_len += snprintf(info_buf + info_len, INFO_BUF_SIZE - info_len, "<%02x>", c);
			}
		}
    }
    info_len += snprintf(info_buf + info_len, INFO_BUF_SIZE - info_len, "\n");
}


static void output_fx25_info(int tag_no, uint8_t *ax25_buf, int ax25_len, uint8_t *fx25_buf, uint8_t *err_buf, int rs_result)
{
    info_len = snprintf(info_buf, INFO_BUF_SIZE, "FX25:%d:", ax25_len);
    packet_print(ax25_buf, ax25_len - 2);
    packet_output((uint8_t *)info_buf, info_len);
    //printf("\tFX25 info: Tag_0%X, RS(%d, %d)\n", tag_no, tags[tag_no].rs_code, tags[tag_no].rs_info);
    fx25_decode_pkts++;

    // calculate total packets
    if (old_seq & 0x80) {
		total_pkts++;
		old_seq = ax25_buf[13] >> 1;
    } else {
		uint8_t seq = ax25_buf[13] >> 1;
		uint8_t diff = (seq - old_seq) & 0x7f;

		if (diff < 8) {
			total_pkts += (seq - old_seq) & 0x7f;
			old_seq = seq;
		} else { // resync sequence
			total_pkts++;
			old_seq = 0x80;
		}
    }

    // print error value
    if (err_buf) {
		info_len = snprintf(info_buf, INFO_BUF_SIZE, "\tFX25 info: %d error(s) corrected\n", rs_result);
		packet_output((uint8_t *)info_buf, info_len);

        int errs = 0;
        for (int i = 1; i < tags[tag_no].rs_code; i++) { // skip fx25_buf[0], because it is always no error
			uint8_t e = err_buf[i] ^ fx25_buf[i];
			if (e > 0) {
				info_len = snprintf(info_buf, INFO_BUF_SIZE, "\tFX25 info: error correction: No.%d, e(%d) = %02x\n", ++errs, i, e);
				packet_output((uint8_t *)info_buf, info_len);
			}
		}
    }

    if (fx25_decode_pkts % 5 == 0) {
		info_len = snprintf(info_buf, INFO_BUF_SIZE, "---- Statistics information ----\n");
		packet_output((uint8_t *)info_buf, info_len);

		info_len = snprintf(info_buf, INFO_BUF_SIZE, "Total: %d pkts, FX25: %d pkts, AX25: %d pkts, FX25%%: %d%%, AX25%%: %d%%, Tag err: %d, RS err: %d, FCS err: %d\n",
			total_pkts, fx25_decode_pkts, ax25_decode_pkts, fx25_decode_pkts * 100 / total_pkts, ax25_decode_pkts * 100 / total_pkts, tag_error_pkts, rs_decode_err, fx25_fcs_err);
		packet_output((uint8_t *)info_buf, info_len);

		info_len = snprintf(info_buf, INFO_BUF_SIZE, "--------------------------------\n");
		packet_output((uint8_t *)info_buf, info_len);
    }
}
#endif

#ifdef CONFIG_TNC_STATISTICS
#define APRS_MSG_LEN 128
#define AX25_ADDR_LEN 7
#define AX25_ADDR_SSID 6
#define AX25_MSG_TOP (7 + 7 + 1 + 1)

#ifdef CONFIG_TNC_APRS_IS_SEND
void fx25_send_aprsis(const char aprs_msg[])
{
    // send to APRS-IS
    static char aprs_udp_msg[APRS_MSG_LEN] = "user " CONFIG_TNC_APRS_IS_CALLSIGN " pass " CONFIG_TNC_APRS_IS_PASSCODE "\r\n" CONFIG_TNC_APRS_IS_CALLSIGN ">APZF25,TCPIP*:";
    static int udp_msg_top = 0;

    if (udp_msg_top == 0) udp_msg_top = strlen(aprs_udp_msg);

    aprs_udp_msg[udp_msg_top] = '\0';

    strncat(aprs_udp_msg, &aprs_msg[AX25_MSG_TOP], APRS_MSG_LEN-1);
    ESP_LOGI(TAG, "aprs_udp_msg: %s", aprs_udp_msg);

    struct netconn *conn = NULL;
    struct ip_addr addr;
    struct netbuf *nbuf = NULL;
    err_t err;

    do {
        err = netconn_gethostbyname(CONFIG_TNC_APRS_IS_SERVER, &addr);
		if (err != ERR_OK) break;
		ESP_LOGI(TAG, "netconn_gethostbyname()");

        conn = netconn_new(NETCONN_UDP);
        if (conn == NULL) break;
		ESP_LOGI(TAG, "netconn_new()");

        err = netconn_connect(conn, &addr, CONFIG_TNC_APRS_IS_PORT);
        if (err != ERR_OK) break;
		ESP_LOGI(TAG, "netconn_connect()");

        nbuf = netbuf_new();
        if (nbuf == NULL) break;
		ESP_LOGI(TAG, "netbuf_new()");

        err = netbuf_ref(nbuf, aprs_udp_msg, strlen(aprs_udp_msg));
        if (err != ERR_OK) break;
		ESP_LOGI(TAG, "netbuf_ref()");
	
        err = netconn_send(conn, nbuf);
        if (err != ERR_OK) break;
		ESP_LOGI(TAG, "netconn_send():");
    } while (0);

    if (nbuf) netbuf_delete(nbuf);
    if (conn) netconn_delete(conn);

    return;
}    
#endif

void fx25_output_statistics(struct FX25_STAT *fx25p)
{
    static uint8_t aprs_msg[APRS_MSG_LEN];
    int index;
    char src_addr[10];
    int ssid;
    int i;
    // dst 'APZF25'
    const char dst_addr[AX25_ADDR_LEN] = { 'A' << 1, 'P' << 1, 'Z' << 1, 'F' << 1, '2' << 1, '5' << 1, 0x60 };

#if 0
    printf("fx25_output_statistics(): source_callsign: ");
    for (i = 0; i < 7; i++) {
		uint8_t c = source_callsign[i];

		if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z')) printf("%c", c);
		else printf("<%02x>", c);
    }
    printf("\n");
#endif

    memcpy(aprs_msg, dst_addr, AX25_ADDR_LEN); index = AX25_ADDR_LEN;
    aprs_msg[index - 1] |= source_callsign[AX25_ADDR_LEN - 1] & 0x80; // SSID, copy command/response bit
    memcpy(aprs_msg + index, source_callsign, AX25_ADDR_LEN); index += AX25_ADDR_LEN;
    aprs_msg[index - 1] |= 0x01; // SSID, set extension bit (LSb)

    aprs_msg[index++] = 0x03; // control, UI frame
    aprs_msg[index++] = 0xf0; // PID, no layer 3 protocol

    // Addressee
    memcpy(&aprs_msg[index], ":FX25INFO :", 11); index += 11;

    // decode callsign to ASCII
    for (i = 0; i < 6; i++) {
		uint8_t c = fx25p->from[i] >> 1;

		if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z')) {
			src_addr[i] = c;
		} else {
			break;
		}
    }
    src_addr[i] = '\0';

    ssid = (fx25p->from[AX25_ADDR_SSID] >> 1)  & 0x0f;
    if (ssid > 0) snprintf(&src_addr[i], 10 - i, "-%d", ssid);

    if (fx25p->ax25_ok) fx25p->ax25_cnt++;

    index += snprintf((char *)&aprs_msg[index], APRS_MSG_LEN - index - 2,
		    "%s cnt=%d ax=%d FCS_err=%d RS_dec=%d len=%d RS=%d,%d e_sym=%d",
		    src_addr,
		    fx25p->count,
		    fx25p->ax25_cnt,
		    fx25p->fcs_error,
		    fx25p->rs_decode,
		    fx25p->len,
		    fx25p->rs_code,
		    fx25p->rs_info,
		    fx25p->error_bytes);

    if (aprs_msg[index - 1] == 0) index--; // delete '\0'

    // output packet to uart
    packet_output(aprs_msg, index); // to uart

#ifdef CONFIG_TNC_APRS_MESSAGES
    // send to aprs via RF
    if (source_callsign[AX25_ADDR_SSID] != 0) fx25_send_packet(aprs_msg, index, 0, AX25_MODE); // send to air
#endif

#ifdef CONFIG_TNC_APRS_IS_SEND
    // send to aprs via Internet
    fx25_send_aprsis((char *)aprs_msg);
#endif

}
#endif

/*
 * AX.25 receive processing
 *
 * rxd: last edge time
 * rxd0: previous edge time
 */
static uint64_t ax25_bit_sum[2] = { 0, 0 };
static uint64_t ax25_bit_cnt[2] = { 0, 0 };

static uint64_t fx25_bit_sum[2] = { 0, 0 };
static uint64_t fx25_bit_cnt[2] = { 0, 0 };

//static const int bit_time[3] = { 66666, 66667, 66667 };

static int ax25_rx(uint32_t rxd, uint32_t rxd0)
{
    static int bit_tm = 0;
    static int bit_sum = 0;
    uint32_t t = rxd - rxd0;
    int level = rxd & 1; // 0: positive edge, 1: negative edge
    int pkt_ack = 0;
    int decode_ok = 0;
    //static int bit_time_cnt = 0;

    if (t > BIT_LEN_MAX * BIT_TIME) {

		// bit sync and reset
		bit_tm = 0;
		bit_sum = 0;
		ax25_nrzi_bit(-1, NULL, 0);

#ifdef CONFIG_TNC_STATISTICS
		fx25_stat.ax25_ok = 0; // clear AX.25 decode success flag
#endif

		return 0;
    }

    // length of the edge duration convert to number of bits
    int bits = (t + BIT_TIME2) / BIT_TIME;

    while (bits-- > 0) {
    //while (bit_tm + t >= BIT_TIME) {
    //while (bit_tm + t >= bit_time[bit_time_cnt]) {
		static uint8_t ax25_nrzi_buf[AX25_NRZI_SIZE];
		uint32_t addt = BIT_TIME - bit_tm;
		//uint32_t addt = bit_time[bit_time_cnt] - bit_tm;
		//bit_time_cnt = (bit_time_cnt + 1) % 3;

		if (level) bit_sum += addt;
		t -= addt;

		int lv = bit_sum >= BIT_TIME2;

		// statistics
		ax25_bit_sum[lv] += bit_sum;
		ax25_bit_cnt[lv]++;

		bit_sum = 0;
		bit_tm = 0;

		//int nrzi_len = ax25_nrzi_bit(lv, ax25_nrzi_buf, AX25_NRZI_SIZE);
		int nrzi_len = ax25_nrzi_bit(level, ax25_nrzi_buf, AX25_NRZI_SIZE);

		if (nrzi_len == -2) { // flag found

			//printf("ax25_rx: bit sync, t = %u\n", t);
			// bit sync
			t = ((t + BIT_TIME2) / BIT_TIME) * BIT_TIME;
		}

		if (nrzi_len <= 0) continue;

//#define AX25_MIN_LEN (7 * 2 + 1 + 1 + 2) // address * 2 + Control + Info + FCS
#define AX25_MIN_LEN 1

		if (nrzi_len >= AX25_MIN_LEN * 8) {
			static uint8_t ax25_buf[AX25_BUF_SIZE];
			int bit_len = nrzi_decode(ax25_buf, AX25_BUF_SIZE, ax25_nrzi_buf, nrzi_len);

#if 0
			if (bit_len < 0) {
				printf("ax25_rx: ax25_nrzi_buf, nrzi_len = %d\n", nrzi_len);
				print_buf(ax25_nrzi_buf, (nrzi_len + 7) / 8);
			}
#endif

			if ((bit_len >= AX25_MIN_LEN * 8) && (bit_len % 8 == 0)) {
				int ax25_len = bit_len / 8;

				if (ax25_fcs_check(ax25_buf, ax25_len)) {
					// success decoding AX.25 packet
#ifdef CONFIG_TNC_DEMO_MODE
					info_len = snprintf(info_buf, INFO_BUF_SIZE, "AX25:%d:", ax25_len);
					packet_print(ax25_buf, ax25_len - 2);
					packet_output((uint8_t *)info_buf, info_len);
					ax25_decode_pkts++;
#else

#ifdef CONFIG_TNC_STATISTICS
					fx25_stat.ax25_ok = 1; // AX.25 decode success
#else
					// stop decoding FX.25 packet
					fx25_decode_bit(-1, NULL, 0);
#endif

					// output packet to serial
					packet_output(ax25_buf, ax25_len - 2);
#endif
					pkt_ack++;
					ax25_nrzi_bits++;
					decode_ok++;
				}
			}
#if 0
			if (!decode_ok && (nrzi_len < 512)) {

				static uint8_t err_buf[AX25_NRZI_SIZE];
				memcpy(err_buf, ax25_nrzi_buf, (nrzi_len + 7) / 8);

				// Error Correction
				ax25_nrzi_ec_try++;
				int ax25_len = ax25_bec(ax25_buf, AX25_BUF_SIZE, ax25_nrzi_buf, nrzi_len);

				if (ax25_len > 0) { // EC success

					// output packet to serial
					ESP_LOGI(TAG, "ax25_rx: AX25 BEC success, ax25_nrzi_buf[], nrzi_len = %d", nrzi_len);
					//print_diff(err_buf, ax25_nrzi_buf, (nrzi_len + 7) / 8);
					packet_output(ax25_buf, ax25_len - 2);
					pkt_ack++;
					ax25_nrzi_bits++;
					ax25_nrzi_ec_ok++;

#ifdef CHECK_MISS
					if (memcmp(test_packet, ax25_buf, test_packet_len) != 0) { // error correction miss
						ax25_nrzi_ec_miss++;
						//print_diff(test_packet, ax25_buf, test_packet_len);
					}
#endif

				}
			}
#endif
		}
    }
    if (level) bit_sum += t;
    bit_tm += t;

    return pkt_ack;
}

/*
 * FX25 receive processing
 */
static int fx25_rx(uint32_t rxd, uint32_t rxd0)
{
    static int bit_tm = 0;
    static int bit_sum = 0;
    uint32_t t = rxd - rxd0;
    int level = rxd & 1; // 0: positive edge, 1: negative edge
    int pkt_ack = 0;
    //static int bit_time_cnt = 0;

    if (t > BIT_LEN_MAX * BIT_TIME) {

		//printf("fx25_rx: bit sync, bit_tm = %u, bit_sum = %u\n", bit_tm, bit_sum);

		// bit sync
		bit_tm = 0;
		bit_sum = 0;
		fx25_decode_bit(-1, NULL, 0);

		return 0;
    }

    while (bit_tm + t >= BIT_TIME) {
    //while (bit_tm + t >= bit_time[bit_time_cnt]) {
		static uint8_t fx25_buf[FX25_BUF_SIZE];
		uint32_t addt = BIT_TIME - bit_tm;
		//uint32_t addt = bit_time[bit_time_cnt] - bit_tm;
			//bit_time_cnt = (bit_time_cnt + 1) % 3;

		if (level) bit_sum += addt;
		t -= addt;

		int lv = bit_sum >= BIT_TIME2;

		// statistics
		fx25_bit_sum[lv] += bit_sum;
		fx25_bit_cnt[lv]++;

			//if ((bit_sum > BIT_TIME2/2) && (bit_sum < BIT_TIME2*3/2)) printf("fx25_rx(): bit_tm = %u, bit_sum = %u\n", bit_tm, bit_sum);

		bit_sum = 0;
		bit_tm = 0;

		int tag_no = fx25_decode_bit(lv, fx25_buf, FX25_BUF_SIZE); 

		if (tag_no == -2) { // flag found

			//printf("fx25_rx: bit sync, t = %u\n", t);

			// bit sync
			t = ((t + BIT_TIME2) / BIT_TIME) * BIT_TIME;
		}

		if (tag_no > 0) { // correlation tag is detected
			static uint8_t ax25_buf[AX25_BUF_SIZE];
			int rs_code_size = tags[tag_no].rs_code;
			int ax25_len = bitstuff_decode(ax25_buf, AX25_BUF_SIZE, &fx25_buf[1], rs_code_size - 1); // buf[0] is AX.25 flag (7E)

#ifdef CONFIG_TNC_DEMO_MODE
			info_len = snprintf(info_buf, INFO_BUF_SIZE, "\tFX25 info: detect tag, Tag_%02X, RS(%d, %d)\n", tag_no, tags[tag_no].rs_code, tags[tag_no].rs_info);
			packet_output((uint8_t *)info_buf, info_len);
#endif

#if 0
			if (ax25_len <= 2) {
				printf("fx25_rx(): ax25_len = %d\n", ax25_len);
				//print_buf(fx25_buf, rs_code_size);
			}
#endif

#if CONFIG_TNC_STATISTICS
			// statistics info
			fx25_stat.count++;
			fx25_stat.rs_code = tags[tag_no].rs_code;
			fx25_stat.rs_info = tags[tag_no].rs_info;
			fx25_stat.error_bytes = 0;
#endif

			if ((ax25_len > 2) && ax25_fcs_check(ax25_buf, ax25_len)) {
#ifdef CONFIG_TNC_DEMO_MODE
				output_fx25_info(tag_no, ax25_buf, ax25_len, fx25_buf, NULL, 0);
#else
#ifdef CONFIG_TNC_STATISTICS
				if (!fx25_stat.ax25_ok) packet_output(ax25_buf, ax25_len - 2);
				memcpy(fx25_stat.from, &ax25_buf[7], 7); // copy source address
				fx25_stat.len = ax25_len;
				fx25_output_statistics(&fx25_stat);
#else
				packet_output(ax25_buf, ax25_len - 2);
#endif
#endif
				pkt_ack++;
				fx25_bits++;

			} else {

#ifdef CONFIG_TNC_STATISTICS
				fx25_stat.fcs_error++;
#endif

				//print_diff(test_fx25_buf, fx25_buf, test_fx25_len);

				fx25_rs_try++;

				static uint8_t err_buf[FX25_BUF_SIZE];
				memcpy(err_buf, fx25_buf, rs_code_size);

				int rs_result = fx25_rsdecode(fx25_buf, tag_no); // RS error correction


				if (rs_result >= 0) {

					ESP_LOGD(TAG, "fx25_rx: RS error correction: %d symbols", rs_result);
					//print_diff(err_buf, fx25_buf, rs_code_size);
					
					ax25_len = bitstuff_decode(ax25_buf, AX25_BUF_SIZE, &fx25_buf[1], rs_code_size - 1);

					if ((ax25_len > 2) && ax25_fcs_check(ax25_buf, ax25_len)) {
#ifdef CONFIG_TNC_DEMO_MODE
						output_fx25_info(tag_no, ax25_buf, ax25_len, fx25_buf, err_buf, rs_result);
#else
#ifdef CONFIG_TNC_STATISTICS
						if (!fx25_stat.ax25_ok) packet_output(ax25_buf, ax25_len - 2);
						memcpy(fx25_stat.from, &ax25_buf[7], 7); // copy source address
						fx25_stat.len = ax25_len;
						fx25_stat.rs_decode++;
						fx25_stat.error_bytes = rs_result;
						fx25_output_statistics(&fx25_stat);
#else
						packet_output(ax25_buf, ax25_len - 2);
#endif
#endif
						fx25_bits++;
						fx25_rs_ok++;
						pkt_ack++;

#ifdef CHECK_MISS
						if (memcmp(test_packet, ax25_buf, test_packet_len) != 0) fx25_rs_miss++;
#endif
					} else {
#ifdef CONFIG_TNC_DEMO_MODE
						printf("\tFX25 info: FX.25 FCS error\n");
						fx25_fcs_err++;
#endif

						ESP_LOGD(TAG, "fx25_rx: FCS error");
					}

				} else {
#ifdef CONFIG_TNC_DEMO_MODE
					printf("\tFX25 info: RS decode error\n");
					rs_decode_err++;
#endif	
					ESP_LOGD(TAG, "fx25_rx: RS decode error: %d", rs_result);
#ifdef CHECK_MISS
					//print_diff(test_packet, ax25_buf, test_packet_len);
#endif
				}
			}
		}
    }
    if (level) bit_sum += t;
    bit_tm += t;

    return pkt_ack;
}

/*
 * TNC receive task
 */
void rx_task(void *p)
{
    QueueHandle_t capqueue = *(QueueHandle_t *)p;
    uint32_t rxd0 = 0, /*rxd1 = 0,*/ rxd;    
    int rc;
    //static uint8_t ax25_buf[AX25_BUF_SIZE];
    //static uint8_t ax25_nrzi_buf[AX25_NRZI_SIZE];
    //static uint8_t ax25_buf2[AX25_BUF_SIZE];
    //static uint8_t fx25_buf[FX25_BUF_SIZE];
    //int ax25_cnt = 0;
    //int ec_cnt = 0;
    //int len;
    //int nrzi_cnt = 0;
    //int st = 0;
    //int bit_sum = 0;
    //int bit_tm = 0;
    int pkt_ack = 0;

    while (1) {
		rxd0 = rxd;
		rc = xQueueReceive(capqueue, &rxd, 1000); // wait 1000 ticks
		if (rc != pdTRUE) continue;

		if (cap_queue_err) {
			printf("mcpwm: capture error: %d\n", cap_queue_err);
			cap_queue_err = 0;
		}

		// check RXD edge polarity
		if (((rxd0 ^ rxd) & 1) == 0) {
			// may be lost interrupt
			uint32_t tmp = (rxd - rxd0) >> 3;
			ESP_LOGI(TAG, "wrong RXD edge polarity: time diff %u.%u us", tmp / 10, tmp % 10);
		}


		// AX.25 packet decode
		pkt_ack = ax25_rx(rxd, rxd0);

		//uint32_t t = rxd - rxd0;
		//int level = rxd & 1; // positive edge 0, negative edge 1
		//int bits = (t + BIT_TIME2) / BIT_TIME; // number of bit length between edges
#ifdef AX25_DECODE

#if 0
		if (bits == 0) {
			printf("rx_task: t = %u, rxd = %u, rxd0 = %u\n", t, rxd, rxd0);
			//continue;
		}
#endif

#if 0
		if ((rxd - rxd1) < 5000) {
			rxd1 = rxd;
			continue;
		}
#endif

		//	if (bits <= 0) continue; // too short
#if 0
		rxd0 = rxd;
		rxd1 = rxd;
#endif
		//printf("%d, ", bits);

		len =  ax25_nrzi(bits, ax25_nrzi_buf, AX25_NRZI_SIZE); // store NRZI mesg to buffer
		if (len > 0) printf("ax25_nrzi(): len = %d\n", len);

		// len > 0, if AX.25 packet received (len is bit length)
		if (len >= 18 * 8) { // minimum AX.25 packet length
			printf("ax25_nrzi() found flag: bit len = %d\n", len);

			int bits = nrzi_decode(ax25_buf2, AX25_BUF_SIZE, ax25_nrzi_buf, len);
			printf("nrzi_decode(): bit len = %d\n", bits);

#if 0
			printf("ax25_nrzi_buf: len = %d\n", len);
#if 1
			for (int i = 0; i < len / 8; i++) {
				printf("%02x ", ax25_nrzi_buf[i]);
			}
#else
			for (int i = 0; i < len; i++) {
				printf("%d", (ax25_nrzi_buf[i / 8] >> (i % 8)) & 1);
				if (i % 8 == 7) printf(" ");
			}
#endif
			printf("\n");

			printf("ax25_buf2: bits = %d\n", bits);
#if 1
			for (int i = 0; i < bits / 8; i++) {
				printf("%02x ", ax25_buf2[i]);
			}
#else
			for (int i = 0; i < bits; i++) {
				printf("%d", (ax25_buf2[i / 8] >> (i % 8)) & 1);
				if (i % 8 == 7) printf(" ");
			}
#endif
			printf("\n");

			printf("ax25_buf2: bits = %d\n", bits);
			for (int i = 0; i < bits/8; i++) {
				printf("%02x ", ax25_buf2[i]);
			}
			printf("\n");
#endif
			int fcs_ok = 0;
			if ((bits % 8 == 0) && (bits >= 24)) { // if NRZI decode packet has byte length
				int byte = bits / 8;
				uint16_t fcs = ax25_fcs(ax25_buf2, byte - 2);
				uint16_t fcs2 = ax25_buf2[byte - 2] | (ax25_buf2[byte - 1] << 8);
#if 0
				printf("ax25_nrzi: bits = %d, byte = %d\n", bits, byte);
				printf("ax25_nrzi: fcs = %04x, fcs2 = %04x\n", fcs, fcs2);
				printf("ax25_buf2: byte = %d\n", byte);
#endif
#if 0
				for (int i = 0; i < byte; i++) {
					printf("%02x ", ax25_buf2[i]);
				}
				printf("\n");
#endif
				if (fcs == fcs2) {
					printf("ax25_nrzi: FCS is correct: %04x\n", fcs);
					fcs_ok = 1;
					ax25_nrzi_pkts++;
#ifdef CHECK_MISS
					if ((byte != test_packet_len) || memcmp(ax25_buf2, test_packet, test_packet_len)) ax25_nrzi_ec_miss++;
					ESP_LOGI(TAG, "ax25_nrzi: byte = %d, len = %d", byte, test_packet_len);
#if 0
					for (int i = 0; i < test_packet_len; i++) {
						printf("%02x:%02x ", ax25_buf2[i], test_packet[i]);
					}
					printf("\n");
#endif
#endif
#if 0
					for (int i = 0; i < byte - 2; i++) {
						int c = ax25_buf2[i];

						if (i < 14) c >>= 1; // address field
						printf("%c", isprint(c) ? c : '.');
					}
					printf("\n");
#endif

				}
			}

#if 0
			if (!fcs_ok && (len < 512)) { // avoid large packet
				struct timeval tv0, tv1;
				gettimeofday(&tv0, NULL);

				ax25_nrzi_ec_try++;
				// error correction with BEC
				for (int off = 0; off < len; off++) {
					ax25_nrzi_buf[off / 8] ^= 1 << (off % 8); // invert bit

					bits = nrzi_decode(ax25_buf2, AX25_BUF_SIZE, ax25_nrzi_buf, len);
					if ((bits % 8 == 0) && (bits >= 24)) {
						int byte = bits / 8;

						uint16_t fcs = ax25_fcs(ax25_buf2, byte - 2);
						uint16_t fcs2 = ax25_buf2[byte - 2] | (ax25_buf2[byte - 1] << 8);

						if (fcs == fcs2) {
							gettimeofday(&tv1, NULL);
							tv1.tv_sec -= tv0.tv_sec;
							tv1.tv_usec -= tv0.tv_usec;
							if (tv1.tv_usec < 0) {tv1.tv_usec += 1000000; --tv1.tv_sec; }

							fcs_ok = 1;
							ax25_nrzi_pkts++;
							ax25_nrzi_ec_ok++;
#if 0 
		  					printf("ax25_nrzi: error correction: err_bit = %d, len = %d, cnt = %d, time: %ld.%06ld\n",
				  					  off, byte, ++nrzi_cnt, tv1.tv_sec, tv1.tv_usec);

							printf("correct NRZI data, error pos: %d:%d, error data: %02x, correct data: %02x\n",
									  off / 8, off % 8, ax25_nrzi_buf[off/8] ^ (1 << (off%8)), ax25_nrzi_buf[off/8]);
#endif
#if 0
							int i;
							for (i = 0; i < (len + 7) / 8; i++) {
								printf("%02x ", ax25_nrzi_buf[i]);
							}
		  					printf("\n");
#endif
							printf("ax25 decoded packet, len = %d\n", byte);
#if 0
							for (i = 0; i < byte; i++) {
								printf("%02x ", ax25_buf2[i]);
							}
							printf("\n");

							for (i = 0; i < byte - 2; i++) {
								int c = ax25_buf2[i];

		    					if (i < 14) c >>= 1;
		    					printf("%c", isprint(c) ? c : '.');
							}
							printf("\n");
#endif
							break;
						}

	    			}
		
	    			ax25_nrzi_buf[off / 8] ^= 1 << (off % 8); // restore bit
	    		}

	    		if (!fcs_ok) {
					gettimeofday(&tv1, NULL);
					tv1.tv_sec -= tv0.tv_sec;
					tv1.tv_usec -= tv0.tv_usec;
					if (tv1.tv_usec < 0) {tv1.tv_usec += 1000000; --tv1.tv_sec; }
					printf("error correction fail: %ld.%06lds\n", tv1.tv_sec, tv1.tv_usec);
	   			}

			}
#endif
		}

		//len = ax25_decode(bits, ax25_buf, AX25_BUF_SIZE);


		int bit = 0;
		int cnt = bits;

		//printf("bits = %d, level = %d\n", bits, level);

		//if ((bits <= 0) || (bits > BIT_LEN_MAX)) {
		if (t > BIT_LEN_MAX * BIT_TIME) { // between packets
			st = rxd + BIT_TIME2;
			bit_tm = 0;
			bit_sum = 0;

			// reset decoder
			ax25_decode_bit(-1, NULL, 0);
			ax25_nrzi_bit(-1, NULL, 0);
			fx25_decode_bit(-1, NULL, 0);
		} else {
	
#if 0

#define BIT_SYNC_LEN 7
#define BIT_SYNC_MIN ((BIT_TIME * BIT_SYNC_LEN * 15) / 16)
#define BIT_SYNC_MAX ((BIT_TIME * BIT_SYNC_LEN * 17) / 16)

			// sampling time adjust
			static int last_adjust = 0;

			if ((t > BIT_SYNC_MIN) && (t < BIT_SYNC_MAX)) { // pulse width 7 bits mean 7E
				printf("sampling time adjust: between %d bits\n", (rxd - last_adjust + BIT_TIME2) / BIT_TIME);
				last_adjust = rxd;

				bit_tm = 0;
				bit_sum = 0;
			}
#endif

			int bit = 0;

			//while (cnt-- > 0) {
			while (bit_tm + t >= BIT_TIME) {
				int addt = BIT_TIME - bit_tm;

				if (level) bit_sum += addt;
				t -= addt;

				int lv = (bit_sum >= BIT_TIME2); // 0 or 1
#define DIV 16
				//if ((bit_sum > BIT_TIME / DIV) && (bit_sum < BIT_TIME * (DIV-1) / DIV)) printf("bit_sum = %6d, lv = %d\n", bit_sum, lv);

				bit_sum = 0;
				bit_tm = 0;

				st += BIT_TIME; // next sampling time

				static uint8_t ax25_nrzi_buf2[AX25_NRZI_SIZE];
				int nrzi_len = ax25_nrzi_bit(lv, ax25_nrzi_buf2, AX25_NRZI_SIZE);

				if (nrzi_len > 18 * 8) {
#if 0
					printf("ax25_nrzi_bit(): nrzi_len = %d\n", nrzi_len);
					for (int i = 0; i < (nrzi_len + 7) / 8; i++) {
						printf("%02x ", ax25_nrzi_buf2[i]);
					}
					printf("\n");
#endif
					int bits = nrzi_decode(ax25_buf2, AX25_BUF_SIZE, ax25_nrzi_buf2, nrzi_len);
#if 0
					printf("nrzi_decode(): bits = %d\n", bits);
#endif
					if ((bits > 2) && (bits % 8 == 0)) {
						int len = bits / 8;
						int fcs = ax25_fcs(ax25_buf2, len - 2);
						int fcs2 = ax25_buf2[len - 2] | (ax25_buf2[len - 1] << 8);
#if 0
						printf("nrzi_decode(): fcs = %04x, fcs2 = %04x\n", fcs, fcs2);
						if (fcs == fcs2) ax25_nrzi_bits++;

						for (int i = 0; i < bits / 8; i++) {
							printf("%02x ", ax25_buf2[i]);
						}
						printf("\n");
#endif
					}
				}

				static uint8_t fx25_buf2[FX25_BUF_SIZE];
				int tag_no = fx25_decode_bit(lv, fx25_buf2, FX25_BUF_SIZE);

				// if (tag_no) printf("fx25_decode_bit(): tag_no = %d\n", tag_no);
#if 0
				if (tag_no) printf("fx25_decode_bit(): tag_no = %02x\n", tag_no);
#endif
				if (tag_no > 0) {
					int rs_code_size = tags[tag_no].rs_code;
					static uint8_t ax25_buf[AX25_BUF_SIZE];
					int ax25_len;
#if 0
					printf("fx25_decode_bit(): tag_no = %02x, code_size = %d\n", tag_no, rs_code_size);
					for (int i = 0; i < rs_code_size; i++) {
						printf("%02x ", fx25_buf2[i]);
					}
					printf("\n");
#endif
					ax25_len = bitstuff_decode(ax25_buf, AX25_BUF_SIZE, &fx25_buf2[1], rs_code_size - 1);
#if 0
					printf("bitstuff_decode(): ax25_len = %d\n", ax25_len);
#endif

					if (ax25_len > 0) {
#if 0
						for (int i = 0; i < ax25_len; i++) {
							printf("%02x ", ax25_buf[i]);
						}
						printf("\n");
#endif
						int fcs = ax25_fcs(ax25_buf, ax25_len - 2);
						int fcs2 = ax25_buf[ax25_len - 2] | (ax25_buf[ax25_len - 1] << 8);

						if (fcs == fcs2) fx25_bits++;
#if 0
						printf("bitstuff_decode(): fcs = %04x, fcs2 = %04x\n", fcs, fcs2);
#endif
					}
				}

				len = ax25_decode_bit(bit, ax25_buf, AX25_BUF_SIZE);
				//if (len) printf("ax25_decode_bit(): %d\n", len);
				bit = 1;

				if ((len > 2) && (len < AX25_BUF_SIZE)) {
					uint16_t fcs = ax25_fcs(ax25_buf, len - 2);
					uint16_t fcs2 = ax25_buf[len - 2] | (ax25_buf[len - 1] << 8);

#if 0
					printf("ax25_decode_bit(): len = %d, fcs = %04x, fcs2 = %04x\n", len, fcs, fcs2);
					for (int i = 0; i < len; i++) {
						printf("%02x ", ax25_buf[i]);
					}
					printf("\n");
#endif

#if 0
					if (fcs != fcs2 && len >= 32) { // do error correction
						uint16_t bits = fcs ^ fcs2;

						// error correction
						if ((bits & (bits - 1)) == 0) { // FCS may have 1 bit error...
							printf("AX25 error correction: FCS: %04x:%04x:%04x\n", fcs, fcs2, bits);
							fcs2 = fcs;
							ec_cnt++;

						} else {

							struct timeval tv0;
							gettimeofday(&tv0, NULL);

							for (int bit = 0; bit < (len - 2) * 8; bit++) {
								ax25_buf[bit / 8] ^= 1 << (bit % 8); // invert bit

								fcs = ax25_fcs(ax25_buf, len - 2);

								if (fcs == fcs2) {
									struct timeval tv1;
									gettimeofday(&tv1, NULL);
									int sec = tv1.tv_sec - tv0.tv_sec;
									int usec = tv1.tv_usec - tv0.tv_usec;
									if (usec < 0) { usec += 1000000; --sec; }

									printf("AX25 error correction: bit = %d:%d, %d.%06ds\n", bit/8, bit%8, sec, usec);
									ec_cnt++;
									break;
								}

							ax25_buf[bit / 8] ^= 1 << (bit % 8); // restore bit
							}
						}
					}
#endif

					if (fcs == fcs2) {
						ax25_pkts++;
						printf("AX25 packet: len = %d, cnt = %d, ecc = %d, nrzi = %d\n", len, ++ax25_cnt, ec_cnt, nrzi_cnt);
#if 0
						for (int i = 0; i < len - 2; i++) {
							int c = ax25_buf[i];
							int d = c >> 1;

							if (i < 14) {
								printf("%c", isprint(d) ? d : '.');
							} else {
								printf("%c", isprint(c) ? c : '.');
							}
						}
						printf("\n");
#endif
					}
				}
	  		} // while
	  		if (level) bit_sum += t;
	  		bit_tm += t;
		}
#endif // AX25_DECODE

		// decode FX.25 packet
		pkt_ack += fx25_rx(rxd, rxd0);

#ifdef FX25_DECODE
		int rs_status = 0;
		len = fx25_decode(bits, ax25_buf, AX25_BUF_SIZE, &rs_status);
		if (len > 0) { // packet received
			fx25_pkts++; 

			printf("fx25_decode: ax25_buf: len = %d\n", len);
#if 0
			for (int i = 0; i < len; i++) {
				printf("%02x ", ax25_buf[i]);
			}
			printf("\n");
#endif
			if (rs_status > 0) {
				// check packet identity
#ifdef CHECK_MISS
				if ((len != test_packet_len) || memcmp(ax25_buf, test_packet, test_packet_len)) fx25_rs_miss++;
#endif
			}
		}
		if (len != 0) {
			if (rs_status) {
				fx25_rs_try++;
				if (rs_status > 0) {
					fx25_rs_ok++;
				}
			}
		}
#endif // FX25_DECODE

#ifdef USE_NOTIFICATION
#ifdef USE_ACK
		if (pkt_ack) xQueueSend(ack_queue, "A", portMAX_DELAY);
#else
	  	// notify tx_task
		if (pkt_ack) xTaskNotify(tx_task_handle, 0, eNoAction);
#endif
#endif
    }
}

#define PKT_LEN 1024
#define BITS_BUF_LEN 1195

//static uint8_t pkt[256];

#ifdef CONFIG_TNC_BEACON

#define SSID 0x60
#define UI_CONTROL 0x03
#define UI_PID 0xf0

static const uint8_t mycall[] = CONFIG_TNC_BEACON_MYCALL;

#ifndef CONFIG_TNC_DEMO_MODE

// sending a packet periodically for test purpose
void tx_task(void *p)
{
    int len;
    int i, j;
    static uint8_t ax25_data[PKT_LEN];
    static uint8_t dst_addr[7] = { 'B' << 1, 'E' << 1, 'A' << 1, 'C' << 1, 'O' << 1, 'N' << 1, SSID };
    static uint8_t src_addr[7] = { 'N' << 1, 'O' << 1, 'C' << 1, 'A' << 1, 'L' << 1, 'L' << 1, SSID | 0x01 };;
    uint8_t *s;
    struct timeval tv;
    int seq = 1;
    int tnc_mode = get_tnc_mode(); // get default TNC mode

    // callsign to AX.25 addr
    s = (uint8_t *)ax25_call_to_addr((char *)mycall);
    if (s) {
	memcpy(src_addr, s, 7);
	src_addr[6] |= 0x01;
    }

#if 0
    // callsign
    for (i = 0; i < 6; i++) {
	c = toupper(mycall[i]);
	if (!isalnum(c)) c = ' ';
	src_addr[i] = c;
    }
#endif

    i = 0;
    memcpy(&ax25_data[i], dst_addr, 7); i += 7;
    memcpy(&ax25_data[i], src_addr, 7); i += 7;
    ax25_data[i++] = UI_CONTROL; // Control 0x03, UI frame
    ax25_data[i++] = UI_PID;     // PID 0xf0, no layer 3 protocol

    while (1) {
	vTaskDelay(CONFIG_TNC_BEACON_INTERVAL * 1000 / portTICK_PERIOD_MS);
#if 0
	//len = rand() % PKT_LEN + 1;
	len = PKT_LEN; // longest packet
	for (i = 0; i < len; i++) ax25_data[i] = rand() & 0xff;
#else
	i = 16;

	gettimeofday(&tv, NULL);

	i += sprintf((char *)&ax25_data[i],
			"seq=%d, time=%lds, proto=%s%s, FX.25 KISS TNC.",
			seq++, tv.tv_sec,
			(tnc_mode == AX25_MODE) ? "AX.25" : "FX.25/",
			(tnc_mode == AX25_MODE) ? ""
			: (tnc_mode == FX25_PARITY_16) ? "16"
			: (tnc_mode == FX25_PARITY_32) ? "32" : "64");

	// padding with randam data
	while (i < CONFIG_TNC_BEACON_LEN) {
	    ax25_data[i] = ' ' + i % ('~' - ' ' + 1);
	    i++;
	}

	len = CONFIG_TNC_BEACON_LEN;
#endif
	fx25_send_packet(ax25_data, len, 0, tnc_mode); // 0:do not wait for Queuing, default TNC mode
    }
}

#else // CONFIG_TNC_DEMO_MODE

#define PKT_BUFSIZE 239

// sending packet for demo purpose
void tx_task(void *p)
{
    int idx, len;
    int tnc_mode = get_tnc_mode();
    static unsigned int seed = 1;
    static uint8_t pkt_buf[PKT_BUFSIZE];
    int seq= 0;

    while (1) {
	idx = 0;
		while ((len = packet_table[idx++]) > 0) {
			memcpy(pkt_buf, &packet_table[idx], len - 2); // copy packet to rewrite source address

			// use my callsign as source address
			for (int i = 0; i < 6; i++) {
				uint8_t c = toupper(mycall[i]);
				if (!isalnum(c)) c = ' ';
				pkt_buf[i+7] = c << 1; // source address
			}
			// SSID, store sequnce number
			pkt_buf[13] &= 0x01; // clear except address extension bit
			pkt_buf[13] |= ++seq << 1;

			vTaskDelay((9 * 1000 + rand_r(&seed) % 3000) / portTICK_PERIOD_MS);
			fx25_send_packet(pkt_buf, len - 2, 0, tnc_mode); // -2: delete FCS
			idx += len;
		}
    }
}
#endif // CONFIG_TNC_DEMO_MODE

#endif // CONFIG_TNC_BEACON

void app_main()
{

    static QueueHandle_t capqueue;
    BaseType_t rc;
    TaskHandle_t xHandle;

    // set log level
#ifdef CONFIG_TNC_LOG_LEVEL
    esp_log_level_set("*", CONFIG_TNC_LOG_LEVEL);
#else
    esp_log_level_set("*", ESP_LOG_NONE);
#endif

#ifdef CONFIG_RS_DIREWOLF_GP
    ESP_LOGI(TAG, "RS code Direwolf compatible mode");
#else
    ESP_LOGI(TAG, "RS code old FX.25 KISS TNC compatible mode");
#endif

    capqueue = xQueueCreate(RXD_QUEUE_LEN, sizeof(uint32_t));

    mcpwm_initialize(capqueue); // capture RXD state

    gpio_init(); // LED reflect GPIO state (connect CDT or RXD)

    rmt_tx_init(); // send pulse train to modem (TXD)
 
#ifdef CONFIG_ESP_DAC
    dac_init();	// RXB and CDL
#endif

#ifdef CONFIG_ESP_LEDC
    ledc_init(); // clock for TCM3105
#endif

#ifdef USE_WIFI
    wifi_start();	// start WiFi
#endif

    uart_init(); // set TX and RX buffer, invoke service task

    if (rs_init() != RS_OK) {; // init Reed Solomon routine
		printf("rs_init() error\n");
		return;
    }

    fx25tag_init(); // generate FX.25 PN tags 

    // receiver task
    rc = xTaskCreatePinnedToCore(rx_task, "rx_task", 1024*4, &capqueue, tskIDLE_PRIORITY+1, &xHandle, tskNO_AFFINITY); 
    if (rc != pdPASS) {
		return;
    }

#ifdef CONFIG_TNC_BEACON 
    // transmitter task
    rc = xTaskCreatePinnedToCore(tx_task, "tx_task", 1024*4, NULL, tskIDLE_PRIORITY+1, &tx_task_handle, tskNO_AFFINITY); 
    if (rc != pdPASS) {
		return;
    }
#endif


#ifdef CONFIG_BME280_EXISTS
    //  BME280 APRS transmit test
    ESP_ERROR_CHECK(i2c_master_init()); /* I2C interface initialize */
    BME280_setup(); /* BME280 setup */
    vTaskDelay(2 * 1000 / portTICK_PERIOD_MS); /* wait more than BME280 measurment period */
    xTaskCreate(BME280_aprs_task, "BME280_aprs_task_0", 1024 * 2, (void *)0, 10, NULL);
#endif


#ifdef CONFIG_TNC_DEMO_MODE
    printf("***Warning: FX.25 KISS TNC enter demonstration mode***\n");
#endif

}


/*
 * fx25 decode routines
 */
#include "config.h"

#ifdef FX25_ENABLE

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "esp32/rom/crc.h"

#include "fx25.h"
#include "rs8.h"
#include "tnc.h"
#include "kiss.h"
#include "fx25_decode.h"
#include "send.h"

#define AX25_FLAG 0x7e
#define FX25_TAG_MATCH 8

#define TAG "fx25_decode"


static int fx25_decode_address(uint8_t *addr, uint8_t buf[])
{
    int ssid = (addr[CALLSIGN_LEN - 1] >> 1) & 0x0f;
    uint8_t *p = buf;

    for (int i = 0; i < CALLSIGN_LEN - 1; i++) {
        int c = addr[i] >> 1;

        if (c == ' ') break;
        *p++ = c;
    }

    if (ssid > 0) {
        p += snprintf((char *)p, 4, "-%d", ssid);
    }

    return p - buf;
}

#define FX25_STATUS_PACKET_SIZE 128

static void fx25_send_status_packet(tcb_t *tp, uint8_t ax25buf[])
{
    static const uint8_t status_header[] = {
    	'A' << 1, 'P' << 1, 'Z' << 1, 'F' << 1, '2' << 1, '5' << 1, // APZX25 experimental
    	0xe0, // command(V2), ssid
	    'N' << 1, 'O' << 1, 'C' << 1, 'A' << 1, 'L' << 1, 'L' << 1, // source address
	    0x61, // command(V2), ssid, end of address field
	    0x03, // control
	    0xf0, // PID
        ':', // APRS Message Data Type
        'F', 'X', '2', '5', 'I', 'N', 'F', 'O', ' ',
        ':',
    };
    static uint8_t src_addr[CALLSIGN_LEN];
    uint8_t *packet;
    uint8_t *p;

    packet = malloc(FX25_STATUS_PACKET_SIZE);
    if (packet == NULL) {
        ESP_LOGW(TAG, "malloc() fail");
        return;
    }

    if (src_addr[0] == 0) {
        make_address((char *)src_addr, FX25_STAT_CALLSIGN);
    }

    p = packet;

    memcpy(p, status_header, sizeof(status_header));
    p += sizeof(status_header);

    memcpy(&packet[CALLSIGN_LEN], src_addr, CALLSIGN_LEN);

    p += fx25_decode_address(&ax25buf[CALLSIGN_LEN], p);

    p += snprintf((char *)p, FX25_STATUS_PACKET_SIZE - (p - packet), " TAG=%u FX25=%u AX25=%u FCS_ERR=%u RS_DEC=%u",
            tp->fx25_cnt_tag, tp->fx25_cnt_fx25, tp->fx25_cnt_tag - tp->fx25_cnt_fcs_err, tp->fx25_cnt_fcs_err, tp->fx25_cnt_fx25 - (tp->fx25_cnt_tag - tp->fx25_cnt_fcs_err));

    send_packet(tp, packet, p - packet, SEND_PACKET_AX25);

#ifdef DEBUG
    ESP_LOGI(TAG, "fx25_send_status_packet(): size=%d, port=%d", p - packet, tp->port);
#endif

    free(packet);
}

static inline int bit_count(uint64_t bits)
{
#ifdef __XTENSA__
    uint32_t t0 = (uint32_t)bits;
    uint32_t t1 = (uint32_t)(bits >> 32);
    uint32_t t2, t3;
    uint32_t c55 = 0x55555555;
    uint32_t c33 = 0x33333333;
    uint32_t c0f = 0x0f0f0f0f;

    asm volatile(
	    // x = x - ((x >> 1) & 0x55555555);
	    // low 32bit
	    "srli	%[t2], %[t0], 1\n\t"	    // x >> 1
	    "and	%[t2], %[t2], %[c55]\n\t"   // (x >> 1) & 0x55555555
	    "sub	%[t0], %[t0], %[t2]\n\t"	// x - ((x >> 1) & 0x55555555)

	    // high 32bit
	    "srli	%[t2], %[t1], 1\n\t"        // x >> 1
        "and	%[t2], %[t2], %[c55]\n\t"   // (x >> 1) & 0x55555555
	    "sub	%[t1], %[t1], %[t2]\n\t"	// x - ((x >> 1) & 0x55555555)

	    // x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
	    // low 32bit
	    "and	%[t2], %[t0], %[c33]\n\t"   // (x & 0x33333333)
	    "srli	%[t3], %[t0], 2\n\t"		// x >> 2
	    "and	%[t3], %[t3], %[c33]\n\t"	// (x >> 2) & 0x33333333;
	    "add.n	%[t0], %[t2], %[t3]\n\t"	// (x & 0x33333333) + ((x >> 2) & 0x33333333)

	    // high 32bit
	    "and	%[t2], %[t1], %[c33]\n\t"	// (x & 0x33333333)
	    "srli	%[t3], %[t1], 2\n\t"		// x >> 2
	    "and	%[t3], %[t3], %[c33]\n\t"	// (x >> 2) & 0x33333333;
	    "add.n	%[t1], %[t2], %[t3]\n\t"	// (x & 0x33333333) + ((x >> 2) & 0x33333333)

	    // x = (x + (x >> 4)) & 0x0f0f0f0f
	    // low 32bit
	    "srli	%[t2], %[t0], 4\n\t"		// x >> 4
	    "add.n	%[t0], %[t0], %[t2]\n\t"	// x + (x >> 4)
	    "and	%[t0], %[t0], %[c0f]\n\t"	// (x + (x >> 4)) & 0x0f0f0f0f

	    // high 32bit
	    "srli	%[t2], %[t1], 4\n\t"		// x >> 4
	    "add.n	%[t1], %[t1], %[t2]\n\t"	// x + (x >> 4)
	    "and	%[t1], %[t1], %[c0f]\n\t"	// (x + (x >> 4)) & 0x0f0f0f0f

	    // x = x + (x >> 32)
	    "add.n	%[t0], %[t0], %[t1]\n\t"	// low 32bit + high 32bit

	    // x = x + (x >> 8)
	    "srli	%[t2], %[t0], 8\n\t"		// x >> 8
	    "add.n	%[t0], %[t0], %[t2]\n\t"	// x + (x >> 8)

	    // x = x + (x >> 16)
	    "srli	%[t2], %[t0], 16\n\t"		// x >> 16
	    "add.n	%[t0], %[t0], %[t2]\n\t"	// x + (x >> 16)

	    // x & 0x7f
	    "extui	%[t0], %[t0], 0, 7\n\t"		// x = x & 0x7f

	    :
	    [t0] "+r" (t0),			// 0
	    [t1] "+r" (t1),			// 1
	    [t2] "=&r" (t2),		// 2
	    [t3] "=&r" (t3)			// 3
	    :
	    [c55] "r" (c55),		// 4
	    [c33] "r" (c33),		// 5
	    [c0f] "r" (c0f)			// 6
       );

    return t0;
#else
    uint64_t b64_0 = bits, b64_1;
    uint32_t b32_0, b32_1;
    uint16_t b16_0, b16_1;
    uint8_t b8_0, b8_1;

    b64_1 = (b64_0 >> 1) & 0x5555555555555555LLU;
    b64_0 &=               0x5555555555555555LLU;

    b64_0 += b64_1; /* 01 + 01 = 10 */

    b64_1 = (b64_0 >> 2) & 0x3333333333333333LLU;
    b64_0 &=               0x3333333333333333LLU;

    b64_0 += b64_1; /* 10 + 10 = 0100 */

    b32_1 = (b64_0 >> 32); /* higher 32bit */
    b32_0 = b64_0;         /* lower 32bit */

    b32_0 += b32_1; /* 0100 + 0100 = 1000 */

    b32_1 = (b32_0 >> 4) & 0x0f0f0f0f; /* separate each 4bit */
    b32_0 &=               0x0f0f0f0f;

    b32_0 += b32_1; /* 0x8 + 0x8 = 0x10 */

    b16_1 = (b32_0 >> 16); /* higher 16bit */
    b16_0 = b32_0; 	   /* lower 16bit */

    b16_0 += b16_1; /* 0x10 + 0x10 = 0x20 */

    b8_1 = (b16_0 >> 8); /* higher 8bit */
    b8_0 = b16_0;	 /* lower 8bit */

    b8_0 += b8_1; /* 0x20 + 0x20 = 0x40 */

    return b8_0;
#endif
}

static fx25tag_t const *fx25_search_tag(tcb_t *tp)
{
    for (int i = TAG_01; i < TAG_0C; i++) {
        int bits = bit_count(tp->fx25_tag ^ fx25tag[i].tagval);

        if (bits <= FX25_TAG_MATCH) {
#ifdef DEBUG
            ESP_LOGI(TAG, "bits = %d", bits);
            ESP_LOGI(TAG, "tag0 = %016llx", tp->fx25_tag);
            ESP_LOGI(TAG, "tag1 = %016llx", fx25tag[i].tagval);
#endif
            return &fx25tag[i];
        }
    }

     return NULL;
}

static int fx25_unbit_stuffing(tcb_t *tp, uint8_t *buf)
{
    int data_cnt = 1; // AX.25 packet start at fx25_data[1]
    bool delete_zero = false;
    int count_ones = 0;
    uint8_t data = 0;
    int data_bits = 0;
    int buf_index = 0;

    while (data_cnt < tp->fx25_tagp->rs_code) {
    
        uint8_t bitq = tp->fx25_data[data_cnt++];
        int bitq_bits = 8;

        while (bitq_bits-- > 0) {

            uint8_t bit = bitq & 1;
            bitq >>= 1;

            if (delete_zero) {
                if (bit == 0) { // bit stuffing bit
                    delete_zero = false;
                    continue;
                }

                if (data_bits == 6) {  // may be end flag (7e)
                    return buf_index;
                }

                return -1; // error         
            }

            if (bit) {

#define BIT_STUFFING_BITS 5

                if (++count_ones >= BIT_STUFFING_BITS) {
                    delete_zero = true;
                    count_ones = 0;
                }

            } else {

                count_ones = 0;
            
            }

            data |= bit << data_bits;

            if (++data_bits >= 8) {
                buf[buf_index++] = data;

                if (buf_index >= tp->fx25_tagp->rs_info) {
                    return buf_index;
                }

                data = 0;
                data_bits = 0;
                
            }
        }

    }

	return -1; // AX.25 end flag not found
}

#define GOOD_CRC 0x0f47

static int fx25_packet_decode(tcb_t *tp)
{
    uint8_t *kiss_buf = malloc(tp->fx25_tagp->rs_info + 1);
    if (kiss_buf == NULL) return -1;

    uint8_t *buf = &kiss_buf[1]; // room for kiss type

#ifdef DEBUG
    // add error
    //tp->fx25_data[tp->fx25_tagp->rs_code / 2] ^= 1;
    int rs8_ret = rs8_decode(tp->fx25_data, tp->fx25_tagp->rs_code, tp->fx25_tagp->rs_code - tp->fx25_tagp->rs_info);
    if (rs8_ret == RS8_ERR) {
        ESP_LOGI(TAG, "RS decode: decode error, port=%d", tp->port);
    } else {
        ESP_LOGI(TAG, "RS decode: %d errors corrected, RS(%d, %d), port=%d", rs8_ret, tp->fx25_tagp->rs_code, tp->fx25_tagp->rs_info, tp->port);
        if (rs8_ret > 0) {
            for (int i = 0; i < tp->fx25_tagp->rs_code; i++) {
                printf("%02x, ", tp->fx25_data[i]);
                if (i % 16 == 15) printf("\n");
            }
            printf("\n");
        }
        int buf_len = fx25_unbit_stuffing(tp, buf);
        if (buf_len > 0 && crc16_le(0, buf, buf_len) == GOOD_CRC) {
            ESP_LOGI(TAG, "FCS is correct, port=%d", tp->port);
        }
    }
#endif

    int buf_len = fx25_unbit_stuffing(tp, buf);

    if (buf_len <= 0 || crc16_le(0, buf, buf_len) != GOOD_CRC) { // FCS error

#ifdef FX25_STAT
        tp->fx25_cnt_fcs_err++;
#endif

#ifdef DEBUG
        ESP_LOGI(TAG, "FCS error detected, port=%d", tp->port);
        printf("buf[%d] =\n", buf_len);
        for (int i = 0; i < buf_len; i++) {
            printf("%02x, ", buf[i]);
            if (i % 16 == 15) printf("\n");
        }
        printf("\nfx25_data[%d] =\n", tp->fx25_tagp->rs_code);
        for (int i = 0; i < tp->fx25_tagp->rs_code; i++) {
            printf("%02x, ", tp->fx25_data[i]);
            if (i % 16 == 15) printf("\n");
        }
        printf("\n");
#endif
        tp->fx25_data[0] = AX25_FLAG; // the byte must be 0x7e
        int rs8_ret = rs8_decode(tp->fx25_data, tp->fx25_tagp->rs_code, tp->fx25_tagp->rs_code - tp->fx25_tagp->rs_info);
        if (rs8_ret == RS8_ERR) {
            free(kiss_buf);
            return -1;
        }
#ifdef DEBUG
        ESP_LOGI(TAG, "RS decode: %d errors corrected, RS(%d, %d), port=%d", rs8_ret, tp->fx25_tagp->rs_code, tp->fx25_tagp->rs_info, tp->port);
#endif
        buf_len = fx25_unbit_stuffing(tp, buf);
    }

    if (buf_len <= 0 || crc16_le(0, buf, buf_len) != GOOD_CRC) { // FCS error
        free(kiss_buf);
        return -1;
    }
    
	kiss_buf[0] = tp->port << 4;

#if defined(DEBUG) || defined(FX25_STAT)
    if (xTaskGetTickCount() - tp->decode_time > 1000 / portTICK_PERIOD_MS) { // AX.25 packet decode fail
    	kiss_packet_send(kiss_buf, buf_len + 1 - 2); // add kiss_type, delete FCS
    }
#ifdef FX25_STAT
    tp->fx25_cnt_fx25++;
    fx25_send_status_packet(tp, &kiss_buf[1]); // skip kiss type byte
#endif
#else
	kiss_packet_send(kiss_buf, buf_len + 1 - 2); // add kiss_type, delete FCS
#endif

    free(kiss_buf);

    return 0;   
}

void fx25_decode_bit(tcb_t *tp, uint8_t bit)
{
    tp->fx25_tag >>= 1;
    tp->fx25_tag |= (uint64_t)bit << 63;

    switch (tp->fx25_state) {
        case FX25_FINDTAG:
            tp->fx25_tagp = fx25_search_tag(tp);
            
            if (tp->fx25_tagp == NULL) return;
            
#ifdef DEBUG
            ESP_LOGI(TAG, "found fx25tag: TAG_%02d", tp->fx25_tagp - fx25tag);
#endif
            tp->fx25_state = FX25_DATA;
            tp->fx25_data_cnt = 0;
            tp->fx25_data_bit_cnt = 0;
#ifdef FX25_STAT
            tp->fx25_cnt_tag++;
#endif
            break;

        case FX25_DATA:
            tp->fx25_data_byte >>= 1;
            tp->fx25_data_byte |= bit << 7;
            tp->fx25_data_bit_cnt++;
        
            if (tp->fx25_data_bit_cnt >= 8) {

                tp->fx25_data[tp->fx25_data_cnt++] = tp->fx25_data_byte;
                tp->fx25_data_bit_cnt = 0;

                if (tp->fx25_data_cnt >= tp->fx25_tagp->rs_code) { // all data received

#if defined(DEBUG) || defined(FX25_STAT)
					fx25_packet_decode(tp);
#ifdef FX25_STAT
                    ESP_LOGI(TAG, "FX.25 STAT(%d): TAG=%d FX25=%d AX25=%d FCS_ERR=%d RS_DEC=%d",
                        tp->port, tp->fx25_cnt_tag, tp->fx25_cnt_fx25, tp->fx25_cnt_tag - tp->fx25_cnt_fcs_err, tp->fx25_cnt_fcs_err,
                        tp->fx25_cnt_fx25 - (tp->fx25_cnt_tag - tp->fx25_cnt_fcs_err));
#endif
#else
			    	if (xTaskGetTickCount() - tp->decode_time > 1000 / portTICK_PERIOD_MS) { // AX.25 packet decode fail
                   		fx25_packet_decode(tp);
					}
#endif
                    tp->fx25_state = FX25_FINDTAG;
                }
            }
            break;

    }

}

#endif // FX25_ENABLE

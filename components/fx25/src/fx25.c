
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp32/rom/crc.h"
#include "fx25.h"

/*
 * FX.25 correlation tag
 *
 * .cotag: correlation tag value, uint64_t
 * .rs_code: length of code, int
 * .rs_info: length of information, int
 */
const fx25tag_t fx25tag[] = {
    { 0x566ED2717946107EULL, 0, 0, },		// Tag_00
    { 0xB74DB7DF8A532F3EULL, 255, 239, },	// Tag_01
    { 0x26FF60A600CC8FDEULL, 144, 128, },	// Tag_02
    { 0xC7DC0508F3D9B09EULL, 80, 64, },		// Tag_03
    { 0x8F056EB4369660EEULL, 48, 32, },		// Tag_04
    { 0x8F056EB4369660EEULL, 255, 223, },	// Tag_05
    { 0xFF94DC634F1CFF4EULL, 160, 128, },	// Tag_06
    { 0x1EB7B9CDBC09C00EULL, 96, 64, },		// Tag_07
    { 0xDBF869BD2DBB1776ULL, 64, 32, },		// Tag_08
    { 0x3ADB0C13DEAE2836ULL, 255, 191, },	// Tag_09
    { 0xAB69DB6A543188D6ULL, 192, 128, },	// Tag_0A
    { 0x4A4ABEC4A724B796ULL, 128, 64, },	// Tag_0B
    { 0x0293D578626B67E6ULL, 0, 0, },		// Tag_0C
    { 0xE3B0B0D6917E58A6ULL, 0, 0, },		// Tag_0D
    { 0x720267AF1BE1F846ULL, 0, 0, },		// Tag_0E
    { 0x93210201E8F4C706ULL, 0, 0, },		// Tag_0F
};

// find FX.25 tag by data size and parity
fx25tag_t *fx25_get_tagno(int size, int parity)
{
    fx25tag_t *p = NULL;

    for (int i = TAG_01; i < TAG_0C; i++) {
		if (size <= fx25tag[i].rs_info && parity == (fx25tag[i].rs_code - fx25tag[i].rs_info)) {
	    	p = &fx25tag[i];
		}
    }

    return p;
}

/*
 * make AX.25 packet with bit stuffing
 * *buf[2]: input data
 * size[2]: input data length
 * buff: output buffer
 * buff_len: buffer length
 * return: bit length of AX.25 packet
*/
int fx25_bit_stuffing(uint8_t *buf[2], size_t size[2], uint8_t buff[], int buff_len)
{
    int bit_len = 0;
	int buff_index = 0;
    uint8_t data = 0; // data to modem
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

    if (buff_index < buff_len) buff[buff_index++] = flag;
    bit_len += 8;

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

		// bit stuffing
		while (bitq_bits-- > 0) {
	    	int bit;

	    	if (insert_zero) {

				bit = 0;
				insert_zero = false;

	    	} else {

				bit = bitq & 1;
				bitq >>= 1;

		   		if (bit) { // "one"

#define BIT_STUFFING_BITS 5

			   		if (++count_ones >= BIT_STUFFING_BITS) { // insert zero
				   		insert_zero = true;
			   			bitq_bits++;
			   			count_ones = 0;
			   		}
	   			} else { // "zero"
			    		count_ones = 0;
	   			}


	    	}

	    	data |= bit << data_bits;

	    	if (++data_bits >= 8) { // filled all 8 bits
				if (buff_index < buff_len) buff[buff_index++] = data;

				bit_len += data_bits;
				data = 0;
				data_bits = 0;

	    	}
		} // while (bitq_bits-- > 0)

    } // for (i = 0; ..

    if (data_bits > 0) { // there is data to be sent

		if (buff_index < buff_len) buff[buff_index++] = data | (AX25_FLAG << data_bits);

		bit_len += data_bits;

    }

	return bit_len;
}

// send FX.25 packet
// parity: 0: AX.25, 2..64: FX.25
int fx25_send_packet(tcb_t *tp, uint8_t data[], size_t data_len, int parity)
{
	fx25tag_t tagp;
	int tag_no;
	int bit_len;
	int byte_len;
	uint8_t *buf;
	size_t buf_len;

	if (data_len > FX25_DATA_LEN_MAX) return -1; // data too large

	buf_len = ((data_len + 2) * 6 + 5) / 5; // * 6/5, calculate maximum byte length
	
	buf = malloc(buf_len);
	if (buf == NULL) return -1; // malloc error

	// get bit stuffed packet length
	bit_len = fx25_bit_stuffing(data, data_len, buf, buf_len);
	byte_len = (bit_len + 7) / 8;

	// find suitable tag_no
	tagp = fx25_get_tagno(byte_len, parity);
	if (tagp == NULL) {
		free(buf);		
		return -1; // no suitable tag_no
	}

	buf = realloc(buf, tagp->rs_code);
	if (buf == NULL) return -1;

	// fill pad
	for (int i = byte_len; i < tagp->rs_code; i++) {
		int flag = (AX25_FLAG << 8)| AX25_FLAG;
		int shift = bit_len % 8;

		buf[i] = flag >> (8 - shift);
	}

	// add parity
	if (rs8_encode(buf, tagp->rs_info, &buf[tagp->rs_info], parity) != RS8_OK) {
		free(buf);
		return -1;
	}

	// send
	//  preamble + correlation tag + AX.25 + RS parity + postamble
	send_bytes(tp, &tagp->tagval, sizeof(tagp->tagval)); // correlation tag
	send_bytes(tp, buf, tagp->rs_code); // AX.25 packet + FX.25 parity
	send_bytes(tp, &flag, sizeof(flag)); // FX.25 postamble

	free(buf);

	return 0;
}

#include <stdio.h>
#include <freertos/FReeRTOS.h>
#include <freertos/ringbuf.h>

#include "esp_log.h"

#include "kiss.h"
#include "tnc.h"
#include "uart.h"

static char TAG[] = "kiss";

void kiss_packet_send(uint8_t *data, size_t data_len)
{
	xRingbufferSend(uart_rb, data, data_len, 0);
}

// process kiss frame
void kiss_process_frame(kcb_t *kp)
{
    int cmd = kp->data_buf[0];

    if (cmd == CMD_EXIT_KISS) {
		ESP_LOGD(TAG, "exit KISS");
		// set packet monitor mode
		kp->monitor_mode = true;
		ESP_LOGI(TAG, "monitor mode");

		return;
    }

    int port = cmd >> 4;
    cmd &= 0x0f;

    if (port >= TNC_PORTS) return;

    tcb_t *tp = &tcb[port];
    uint8_t ch = 0;
    
    if (kp->data_size >= 2) ch = kp->data_buf[1];

    switch (cmd) {
	case CMD_DATA:
	    // send data to the port
		if (kp->data_size <= 1) break;
#ifdef FX25_ENABLE
		kp->data_buf[0] = tp->fx25_parity; // number of FX.25 parity
#else
		kp->data_buf[0] = 0; // AX.25 packet
#endif
	    if (xRingbufferSend(tp->input_rb, kp->data_buf, kp->data_size, kp->wait) != pdTRUE) {
			ESP_LOGW(TAG, "xRingbufferSend() fail, size = %d, port = %d", kp->data_size - 1, port);
	    }
	    break;

	    // process kiss command
	case CMD_TXDELAY:
	    if (kp->data_size <= 1) break;
	    ESP_LOGD(TAG, "set TXDELAY = %d, port = %d", ch, port);
	    tp->TXDELAY = ch;
	    break;

	case CMD_P:
	    if (kp->data_size <= 1) break;
	    ESP_LOGD(TAG, "set P = %d, port = %d", ch, port);
	    tp->persistence_P = ch;
	    break;

	case CMD_SLOTTIME:
	    if (kp->data_size <= 1) break;
	    ESP_LOGD(TAG, "set SlotTime = %d, port = %d", ch, port);
	    tp->SlotTime = ch;
	    break;

	case CMD_FULLDUPLEX:
	    if (kp->data_size <= 1) break;
	    ESP_LOGD(TAG, "set FullDuplex = %d, port = %d", ch, port);
	    tp->fullDuplex = ch;
	    break;

	case CMD_SETHARDWARE:
	    ESP_LOGD(TAG, "SetHardware = %d, port = %d", ch, port);
	    // do something in the future
	    break;
    }
}

void kiss_process_char(kcb_t *kp, uint8_t ch)
{
    switch (kp->data_state) {
		case DATA_IDLE:
	    	ESP_LOGD(TAG, "DATA_IDLE");
	    	if (ch == KISS_FEND) {
				kp->data_state = DATA_INFRAME;
				kp->data_size = 0;
				break;
	    	}

	    	// process console command
	    	switch (ch) {
				case 'K': // detect 'K' of "KISS ON" command
				case 'k':
		    		kp->monitor_mode = false;
		    		ESP_LOGI(TAG, "kiss mode");
		    		break;

				case 'm': // monitor mode on/off
			    	kp->monitor_mode = !kp->monitor_mode;
		    		if (kp->monitor_mode) {
						ESP_LOGI(TAG, "monitor mode");
		    		} else {
						ESP_LOGI(TAG, "kiss mode");
			    	}
			    	break;

				case '?':
			    	ESP_LOGI(TAG, "console command:");
		    		ESP_LOGI(TAG, "m: toggle monitor mode");
		    		ESP_LOGI(TAG, "K,k: turn on kiss mode");
		    		break;
	    	}

	    	break;

		case DATA_INFRAME:
	    	//ESP_LOGD(TAG, "DATA_INFRAME");
	    	switch (ch) {
				case KISS_FEND: // end of frame
		    		if (kp->data_size == 0) break;

		    		kiss_process_frame(kp);
		    		kp->data_state = DATA_IDLE;
		    		break;

				case KISS_FESC:
				    kp->data_state = DATA_FESC;
		    		break;

				default:
		    		if (kp->data_size >= DATA_BUF_SIZE) {
						ESP_LOGW(TAG, "input buffer overflow");
						kp->data_state = DATA_FEND; // discard the frame
						break;
					}

		    		kp->data_buf[kp->data_size++] = ch;
	    	}
	    	break;

		case DATA_FESC:
	    	ESP_LOGD(TAG, "DATA_FESC");
	    	switch (ch) {
				case KISS_TFEND:
		    		ch = KISS_FEND;
		    		break;

				case KISS_TFESC:
		    		ch = KISS_FESC;
		    		break;

				default:
		    		ESP_LOGW(TAG, "no TFEND or TFESC after FESC, ch = %02x", ch);
	    	}

	    	if (kp->data_size >= DATA_BUF_SIZE) {
				ESP_LOGW(TAG, "input buffer overflow");
				kp->data_state = DATA_FEND;
				break;
	    	}

	    	kp->data_buf[kp->data_size++] = ch;
	    	kp->data_state = DATA_INFRAME;
	    	break;

		case DATA_FEND: // stay in this state until FEND
	    	ESP_LOGD(TAG, "DATA_FEND");
			if (ch == KISS_FEND) {
				kp->data_state = DATA_IDLE;
	    	}
	}
}

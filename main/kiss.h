#pragma once
#ifndef __KISS_H__
#define __KISS_H__ 1

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DATA_BUF_SIZE 1500

#define KISS_FEND 0xc0
#define KISS_FESC 0xdb
#define KISS_TFEND 0xdc
#define KISS_TFESC 0xdd

typedef struct KCB { // Kiss Control Block
	uint8_t monitor_mode;	// output packet in text format if true
	TickType_t wait; // xTicksToDelay of xRingbufferSend
	uint16_t data_size;
    	uint8_t data_state;
	//uint8_t kiss_port;
	//uint8_t kiss_cmd;
	uint8_t data_buf[DATA_BUF_SIZE];
} kcb_t;

enum DATA_STATE {
    DATA_IDLE = 0,
    DATA_TYPE,
    //DATA_COMMAND,
    DATA_INFRAME,
    DATA_FESC,
    DATA_FEND,
    DATA_ERROR,
};

enum KISS_COMMAND {
    CMD_DATA = 0,
    CMD_TXDELAY = 1,
    CMD_P = 2,
    CMD_SLOTTIME = 3,
    CMD_TXTAIL = 4,
    CMD_FULLDUPLEX = 5,
    CMD_SETHARDWARE = 6,
    CMD_EXIT_KISS = 255,
};

void kiss_process_char(kcb_t *kp, uint8_t ch);
void kiss_process_frame(kcb_t *kp);
#endif // __KISS_H__

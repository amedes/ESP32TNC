#pragma once
/*
 * tnc.h
 * 
 * TNC control block
 */
#ifndef _TCB_H_

#define _TCB_H_ 1

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/ringbuf.h>

#include "config.h"
#include "filter.h"

//#define FX25TNCR2 1
//#define FX25TNCR3 1
//#define M5ATOM 1
//#define M5STICKC 1

#ifdef M5ATOM
#include "m5atom.h"
#endif

#ifdef FX25TNCR2
#define TNC_PORTS 2 // number of ports
#elif defined(FX25TNCR3)
#define TNC_PORTS FX25TNCR3_PORTS // number of ports 1..6
#elif defined(M5ATOM)
#define TNC_PORTS 1
#elif defined(M5STICKC)
#define TNC_PORTS 1
#else
#define TNC_PORTS 6 // number of ports 1..6
#endif

#define BAUD_RATE 1200
#define DECODE_DELAY 4.458981479161393e-4 // sample delay
#define DELAY_DIVIDEND 325
#define DELAY_DIVISOR 728866

#define SAMPLING_RATE 20184

#define DELAYED_N ((DELAY_DIVIDEND * SAMPLING_RATE + DELAY_DIVISOR/2) / DELAY_DIVISOR)

#if SAMPLING_RATE < 8000

#define FIR_BPF_N (5 * 4 - 1)
#define FIR_LPF_N (3 * 4 - 1)

#else

#define FIR_BPF_N (8 * 4 - 1)
#define FIR_LPF_N (8 * 4 - 1) // must be multiple of 4 minus 1

#endif


//#define DELAYED_N (int)(SAMPLING_RATE * DECODE_DELAY + 0.5)

enum STATE {
	FLAG = 0,
	DATA
};

#define AX25_FLAG 0x7e
#define DATA_LEN 1500

#define TCB_QUEUE_LENGTH (1024 * 2)
#define TCB_QUEUE_ITEM_SIZE sizeof(uint16_t)

typedef struct TCB { // TNC Control Block
    //QueueHandle_t queue; // send data to modem
    RingbufHandle_t queue; // send data to modem
    RingbufHandle_t ringbuf; // receive data from uart/tcp
    TaskHandle_t task;
    
    uint8_t port; // port NO. 0 - 5
    uint16_t pkts;

    uint8_t state;
    uint8_t flag;
    uint8_t kiss_type; // indicate port number in upper nibble
    uint8_t data[DATA_LEN];
    int data_cnt;
    uint8_t data_byte;
    uint8_t data_bit_cnt;

    int pval;
    int edge;

// audio signal processing
    uint16_t avg;
    int avg_sum;

#define TCB_AVG_N 23

    uint16_t avg_buf[TCB_AVG_N];
    uint8_t avg_idx;
    int cdt_lvl;
    uint8_t cdt;
    int8_t cdt_led_pin;
    uint8_t cdt_led_on;
    SemaphoreHandle_t cdt_sem;

    // FSK decode
    uint8_t bit;

    // transmitter control
    uint8_t ptt;
    uint8_t ptt_pin;
    //QueueHandle_t mqueue; // modem queue

#ifdef FX25TNCR2 // only rev.2 has STA LED
    uint8_t sta_led_pin;
#endif

    // kiss parameter
    uint8_t SlotTime; // 10ms uint
    uint8_t TXDELAY; // 10ms uint
    uint8_t persistence_P; // P = p * 256 - 1
    uint8_t fullDuplex;

    // bell202 demodulator
    int delayed[DELAYED_N];
    int delay_idx;
    int x[FIR_LPF_N];
    int x_idx;

    // filter
    filter_t *bpf;
    filter_t *lpf;
    filter_t *avgf;
} tcb_t;

extern const uint8_t TNC_ADC_CH[];
extern tcb_t tcb[];

void tnc_init(tcb_t *tcb, int ports);

#endif // _TCB_H_

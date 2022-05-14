#pragma once
/*
    bk4802 control program
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"

#define BK4802_REG_MAX 24

typedef struct {
    // I2C
    i2c_port_t i2c_num;
    int sda_io_num;
    int scl_io_num;
    SemaphoreHandle_t i2c_sem;

    // BK4802
    int freq;
    int trx_pin;
    int dcd_led;
    volatile bool ptt;
    volatile bool squelch;
    SemaphoreHandle_t cdt_sem;
    uint16_t rxreg[BK4802_REG_MAX];
    uint16_t txreg[BK4802_REG_MAX];

    // queue
    QueueHandle_t queue;

} bk4802_t;

void bk4802_set_tx(bk4802_t *bkp);
void bk4802_ptt(bk4802_t *bkp, int on);
void bk4802_ptt_isr(bk4802_t *bkp, int on);
void bk4802_freq(bk4802_t *bkp, int freq);
void bk4802_init(bk4802_t *bkp);

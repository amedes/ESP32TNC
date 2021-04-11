#pragma once
/*
 * decode.h
 * 
 * decode routine
 */
#include <stdint.h>
#include "tnc.h"

enum STATE {
	FLAG = 0,
	DATA
};

void demodulator(tcb_t *tp, uint16_t adc);
void decode_bit(tcb_t *tp, uint8_t bit);
void decode(tcb_t *tp, int val);

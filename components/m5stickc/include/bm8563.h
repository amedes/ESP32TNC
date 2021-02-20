#pragma once
/*
 * BM8563: time read/write routine
 */

struct BM8563 {
    int timezone; // time difference between UTC and local time in second, JST-9 = -32400
};

extern struct BM8563 bm8563;

void bm8563_init(void);
void bm8563_read_time(uint8_t *buf);
void bm8563_write_time(uint8_t *buf);

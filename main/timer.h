/*
 * timer.h
 */
#ifndef _TIMER_H_
#define _TIMER_H_ 1

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "tnc.h"

#define SAMPLING_N 11

extern uint32_t timer_ccount;
extern uint32_t mod_ccount;
extern uint32_t timer_cnt_low;
extern uint32_t timer_exec_time;
extern int task_woken;
extern int timer_queue_empty;
extern int timer_interrupt;
extern int send_queue_empty;

void timer_initialize(tcb_t *tcbp, int num_queue);
#endif // _TIMER_H_

#pragma once
/*
    functions for tcm3105
*/
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

extern QueueHandle_t cap_queue;

void tcm3105_init(void);

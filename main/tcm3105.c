#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/timer.h"

#include "config.h"
#include "tnc.h"

#ifndef TCM3105_PORT
#define TCM3105_PORT 0
#endif

#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29)  //Capture 2 interrupt bit

// RXD input PIN
#define GPIO_CAP0_IN   GPIO_RXD_PIN   // capture TCM3105 RXD

#define BUAD_RATE 1200 // 1200 bps
#define TIME_HALF_BIT (80*1000*1000 / BUAD_RATE / 2)
#define CAP_QUEUE_SIZE 1024

#define TAG "tcm3105"

xQueueHandle cap_queue = NULL;

static mcpwm_dev_t * const MCPWM[2] = {&MCPWM0, &MCPWM1};

static void IRAM_ATTR mcpwm_isr_handler(void *arg)
{
    tcb_t *tp = (tcb_t *)arg;
    uint32_t mcpwm_intr_status;
    uint32_t ts0;
    uint32_t edge;
    BaseType_t taskWoken = pdFALSE;

    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status

    if (mcpwm_intr_status & CAP0_INT_EN) {
        ts0 = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); // timestamp
#if 1
        edge = mcpwm_capture_signal_get_edge(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); // edge polarity
        ts0 >>= 1; // clear LSB for indicate positive or negative edge
        ts0 <<= 1;
        ts0 |= edge >> 1; // edge: 1 - positive edge, 2 - negative edge
#endif
#if 1
        if (tp->cdt) xQueueSendFromISR(cap_queue, &ts0, &taskWoken);
#else
        xQueueSendFromISR(cap_queue, &ts0, &taskWoken);
#endif
    }

    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;

#if 1
    if (taskWoken) {
        portYIELD_FROM_ISR();
    }
#endif
}

static void isr_register_task(void *arg)
{
    // register ISR handler
    ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, mcpwm_isr_handler, arg, ESP_INTR_FLAG_IRAM, NULL));

    vTaskDelete(NULL);
}

void mcpwm_initialize(void)
{
    if (cap_queue == NULL) {
        // create capture queue
        cap_queue = xQueueCreate(CAP_QUEUE_SIZE, sizeof(uint32_t));
        assert(cap_queue != NULL);
    }

    // assign GPIO pin for mcpwm capture function
#ifdef DEBUG
    ESP_LOGI(TAG, "GPIO_CAP0_IN = %d", GPIO_CAP0_IN);
#endif
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_IN));

#define CAP_PRESCALE (0) // prescale value is 1

    // enable capture module
    ESP_ERROR_CHECK(mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, CAP_PRESCALE));
 
    // detect both positive and negative edge
    MCPWM[MCPWM_UNIT_0]->cap_cfg_ch[0].mode = BIT(0) | BIT(1); 

    // register ISR handler
    //ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, mcpwm_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL));

    // register ISR handler to CPU 1
    assert(xTaskCreatePinnedToCore(isr_register_task, "isr_register_task", 2048, &tcb[TCM3105_PORT], tskIDLE_PRIORITY, NULL, 1) == pdPASS);

    // enable interrupt
    MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN;
}

// CDT interrupt service routine
static void IRAM_ATTR gpio_isr_cdt(void *arg)
{
    tcb_t *tp = (tcb_t *)arg;
    int level = gpio_get_level(GPIO_CDT_PIN);
    BaseType_t taskWoken = pdFALSE;

    // change CDT semaphore status
    if (level) {
        // channel busy
        xSemaphoreTakeFromISR(tp->cdt_sem, &taskWoken);
    } else {
        // channel clear
        xSemaphoreGiveFromISR(tp->cdt_sem, &taskWoken);
    }

    // change CDT LED state
    gpio_set_level(tp->cdt_led_pin, level);
    tp->cdt = level;

    if (taskWoken) {
        portYIELD_FROM_ISR();
    }
}

#define ESP_INTR_FLAG_DEFAULT (0)

static void gpio_initialize(void)
{
#ifdef DEBUG
    ESP_LOGI(TAG, "GPIO_CDT_PIN = %d", GPIO_CDT_PIN);
#endif
    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_CDT_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_CDT_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_intr_type(GPIO_CDT_PIN, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_CDT_PIN, gpio_isr_cdt, &tcb[TCM3105_PORT]));
    ESP_ERROR_CHECK(gpio_intr_enable(GPIO_CDT_PIN));

#if 0
#ifdef DEBUG
    ESP_LOGI(TAG, "GPIO_TXD_PIN = %d", GPIO_TXD_PIN);
#endif
    ESP_ERROR_CHECK(gpio_reset_pin(GPIO_TXD_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_TXD_PIN, GPIO_MODE_OUTPUT));
#endif
}

#if 0
// timer initialization

#define TCM3105_TIMER_GROUP TIMER_GROUP_0
#define TCM3105_TIMER_INDEX TIMER_1
#define TCM3105_TIMER_DIVIDER 2
#define TCM3105_TIMER_ALARM ((TIMER_BASE_CLK / TCM3105_TIMER_DIVIDER + BAUD_RATE/2) / BAUD_RATE)

static void IRAM_ATTR tcm3105_isr(void *arg)
{
    static uint32_t bitq;
    static int bitq_cnt = 0;
    static int pttoff_timer = 0;
    static uint8_t nrzi = 0;
    tcb_t *tp = (tcb_t *)arg;
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    size_t size;
    uint8_t *rbuf;

    // clear the interrupt status bit
    TIMERG0.int_clr_timers.t1 = 1;

    // enable timer interrupt again
    TIMERG0.hw_timer[TCM3105_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;

    if (bitq_cnt > 0) {

        // do NRZI conversion
        if ((bitq & 1) == 0) {
            nrzi = !nrzi;
        }

        gpio_set_level(GPIO_TXD_PIN, nrzi); // TCM3105 TXD
        bitq >>= 1;
        bitq_cnt--;

    } else if (pttoff_timer > 0) {
        
        if (--pttoff_timer == 0) {
            gpio_set_level(tp->ptt_pin, 0); // PTT off
            tp->ptt = false;
        }
    }

    if (32 - bitq_cnt >= 8) { // bitq has room for one byte
        rbuf = xRingbufferReceiveUpToFromISR(tp->queue, &size, 1); // get one byte

        if (rbuf != NULL) {
            bitq |= rbuf[0] << bitq_cnt;
            bitq_cnt += 8;
            pttoff_timer = 1;

            vRingbufferReturnItemFromISR(tp->queue, rbuf, NULL);
        }

    }

}

static void timer_initialize(void)
{
    const timer_config_t timer_conf = {
        .alarm_en = true,
        .counter_en = true,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = true,
        .divider = TCM3105_TIMER_DIVIDER,
    };
    
    ESP_ERROR_CHECK(timer_init(TCM3105_TIMER_GROUP, TCM3105_TIMER_INDEX, &timer_conf));
    ESP_ERROR_CHECK(timer_set_alarm_value(TCM3105_TIMER_GROUP, TCM3105_TIMER_INDEX, TCM3105_TIMER_ALARM));
#ifdef DEBUG
    ESP_LOGI(TAG, "TCM3105_TIMER_ALARM = %d", TCM3105_TIMER_ALARM);
#endif
    ESP_ERROR_CHECK(timer_set_counter_value(TCM3105_TIMER_GROUP, TCM3105_TIMER_INDEX, 0));
    ESP_ERROR_CHECK(timer_isr_register(TCM3105_TIMER_GROUP, TCM3105_TIMER_INDEX, tcm3105_isr, &tcb[TCM3105_PORT], ESP_INTR_FLAG_DEFAULT, NULL));
    ESP_ERROR_CHECK(timer_enable_intr(TCM3105_TIMER_GROUP, TCM3105_TIMER_INDEX));
}
#endif

void tcm3105_init(void)
{
#ifndef TCM3105_ADC
    mcpwm_initialize();
#endif
    gpio_initialize();
#if 0
    timer_initialize();
#endif
}

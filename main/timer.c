/*
 * AFSK modulator
 */
#include <stdint.h>
#include "esp_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/sigmadelta.h"
#include "soc/sens_periph.h"

#include "config.h"
#include "timer.h"
#include "tnc.h"

#ifdef M5ATOM
#include "m5atom.h"
#endif

#define FACTOR 1	// sampling rate, 1: 13200Hz, 2: 26400Hz

#define TIMER_GROUP_NUM 0
#define TIMER_NUM 0
//#define TIMER_DIVIDER 2
#define TIMER_DIVIDER 2
#define TIMER_CLOCK (TIMER_BASE_CLK / TIMER_DIVIDER)
#define BAUD_RATE 1200
#define RATE_FACTOR (11 * FACTOR)
#ifdef SAMPLING_RATE
#undef SAMPLING_RATE
#endif
#define SAMPLING_RATE (BAUD_RATE * RATE_FACTOR)
//#define SAMPLING_N 8
#define TIMER_ALARM_VALUE ((TIMER_CLOCK + SAMPLING_RATE/2) / SAMPLING_RATE)
//#define TIMER_ALARM_VALUE 80000 // 1ms
#define ESP_INTR_FLAG_DEFAULT 0

#define SIGMADELTA_CHANNEL SIGMADELTA_CHANNEL_0
#define SIGMADELTA_GPIO_PIN GPIO_NUM_25

static const uint8_t sigmadelta_gpio_pins[] = {
#if defined(FX25TNCR1)
	GPIO_TXD_PIN,
#elif defined(FX25TNCR2)
    25, 26,
#elif defined(M5ATOM)
    22, 25,
#elif defined(M5STICKC)
#ifdef M5STICKC_AUDIO
    2, // piezo speaker
#else
    26,
#endif
#else
    25, 26, 27, 14, 12, 13,
    //23, 22, 21, 19, 18, 5, 4, 15,	// noise away?
#endif
};

//#define TIMER_BUSY_PIN GPIO_NUM_25
//#define MOD_BUSY_PIN GPIO_NUM_14

#define TAG "timer"

static  tcb_t *tp;

#define SIN_CYCLES (6 * 11 * FACTOR)
//#define SIN_CYCLES (256)
#define MARK_INC 6	// 1200Hz
#define SPACE_INC 11	// 2200Hz
#define MARK_DIV 11	// 1200Hz
#define SPACE_DIV 6	// 2200Hz
#define MARK_MOD (SIN_CYCLES * MARK_DIV)	// 1200Hz
#define SPACE_MOD (SIN_CYCLES * SPACE_DIV)	// 2200Hz

static const uint8_t sin_tab[SIN_CYCLES] = {
	// cos table for sigma-delta DAC
	90,90,88,86,84,80,76,71,65,59,52,
	45,37,29,21,13,4,-4,-13,-21,-29,-37,
	-45,-52,-59,-65,-71,-76,-80,-84,-86,-88,-90,
	-90,-90,-88,-86,-84,-80,-76,-71,-65,-59,-52,
	-45,-37,-29,-21,-13,-4,4,13,21,29,37,
	45,52,59,65,71,76,80,84,86,88,90,
};

typedef struct {
    RingbufHandle_t queue; // byte data queue
    uint32_t bitq;	// bit queue
    uint8_t bitq_cnt;	// number of bits in bitq

    uint8_t sd_ch;	// sigmadelta dac channel
    uint8_t phase;	// phase of SIN, 0..65 or 0..131

    uint8_t pttoff_timer;	// ptt off timer
    uint8_t level;	// nrzi state
    uint8_t gpio_pin;	// dac output pin
    uint8_t sd_func;	// GPIO matrix function number
} mod_t;

#define MODULATORS 8

static mod_t modulator[MODULATORS];

int timer_queue_empty = 0;
int task_woken = 0;
int timer_interrupt = 0;
int send_queue_empty = 0;
uint32_t timer_exec_time = 0;

uint32_t timer_ccount = 0;
uint32_t mod_ccount = 0;
uint32_t timer_cnt_low = 0;

static inline uint32_t ccount(void)
{
    uint32_t ccount;

    asm volatile(
	    "rsr.ccount	%[ccount]\n\t"
	    : [ccount]"=r"(ccount)
	    );

    return ccount;
}


static RingbufHandle_t rb_isr;
static uint8_t num_q;

#define TIMER_GROUP_NUM 0

void IRAM_ATTR timer_isr(void *arg)
{
    static uint32_t current_alarm_value = TIMER_ALARM_VALUE;

    TIMERG0.int_clr_timers.t0 = 1; // clear interrupt
    TIMERG0.hw_timer[TIMER_NUM].config.alarm_en = TIMER_ALARM_EN; // enable next alarm

    static int cycle = 0;
    static uint8_t *buf = NULL, *bp;

    // for sampling frequency adjustment by DDA
    // 40 MHz / 13200 Hz = 100000 / 33
#define SAMPLING_FREQ (13200 * FACTOR)
#define DDA_GCD (400 * FACTOR)
#define DDA_DIVISOR (SAMPLING_FREQ / DDA_GCD)
#define DDA_DIVIDEND (TIMER_CLOCK / DDA_GCD)

    static uint8_t remainder = 0; // for DDA

    size_t size;

    BaseType_t taskWoken = pdFALSE;

    // output audio data
    if (cycle > 0) {

		// write audio data to DAC
	
#ifdef __XTENSA__ 

		uint32_t val;
		asm volatile (
			"loopgtz	%[count], %=f\n\t"

			"l8ui	%[val], %[bp], 0\n\t"	// load audio sample
			"addi	%[bp], %[bp], 1\n\t"	// next sample
			"addi	%[dac], %[dac], 4\n\t"	// next DAC
			"s32i	%[val], %[dac], 0\n\t"	// write to DAC

			"%=:\n\t"
			:
			[val] "=&r" (val),
			[bp] "+r" (bp)
			:
			[count] "r" (num_q),
			[dac] "r" (&SIGMADELTA.channel[0].val - 1) // offset -4 for adding 4 before storing the value to DAC
		);
#else

		for (int port = 0; port < num_q; port++) {
		    SIGMADELTA.channel[port].val = *bp++; // write to DAC
		}

#endif

		cycle--;

    }

    if (cycle <= 0) {

		// give buffer
		if (buf) vRingbufferReturnItemFromISR(rb_isr, buf, &taskWoken);

		// take buffer
		buf = xRingbufferReceiveFromISR(rb_isr, &size);

		if (buf != NULL) {
			bp = buf;
			cycle = RATE_FACTOR;
		}

    }


    timer_interrupt++;

    // adjust sampling frequency to 13200Hz by DDA
	// 40MHz / 13200Hz = 3030 + 10/33
    uint32_t alarm = 3030;

    remainder += 10;
    if (remainder >= 33) {
		remainder -= 33;
		alarm++;
    }
    
    current_alarm_value += alarm;
    TIMERG0.hw_timer[TIMER_NUM].alarm_low = current_alarm_value; // set next alarm value
    if (current_alarm_value < alarm) { // overflow
		TIMERG0.hw_timer[TIMER_NUM].alarm_high++;
    }

    if (taskWoken) {
		task_woken++;
		portYIELD_FROM_ISR();
    }

}

#if 1
//static QueueHandle_t queues[MODULATORS];

// modulator task
void mod_task(void *arg)
{
    int inc;
    //uint32_t cc0 = 0;
    int start = 0;

    while (1) {

#define BUSY_PORT 0

#define AFSK_CYCLE_BUFSIZE (11 * 6 * FACTOR)

	uint8_t *buf;

	// take buffer
	if (xRingbufferSendAcquire(rb_isr, (void **)&buf, AFSK_CYCLE_BUFSIZE * num_q, portMAX_DELAY) != pdTRUE) {
	    ESP_LOGI(TAG, "xRingbufferSendAcquire() fail");
	    continue;
	}

	uint32_t exec_time = esp_timer_get_time();

	//gpio_set_level(MOD_BUSY_PIN, 1); // busy

	int queue_read_done = false; // restrict queue read once in a loop

	//for (int port = 0; port < num_q; port++) { // port number
	if (++start >= num_q) start -= num_q;
	for (int i = 0; i < num_q; i++) {
	    int port = start + i;
	    if (port >= num_q) port -= num_q;

	    mod_t *mp = &modulator[port];
	    tcb_t *tp = &tcb[port];

	    
	    if (!queue_read_done && (mp->bitq_cnt < 8)) { // bit queueu near empty
		uint8_t *rbuf;

		queue_read_done = true;

//#define USEQUEUE 1

#ifdef USEQUEUE
		uint8_t byte;
	    	if (xQueueReceive(mp->queue, &byte, 0) == pdTRUE) { // no wait
#else
		size_t size;
		size_t read_size = (32 - mp->bitq_cnt) / 8; // free space in byte
	    	//if ((rbuf = xRingbufferReceiveUpTo(mp->queue, &size, 0, sizeof(byte))) == NULL) { // no wait
	    	if ((rbuf = xRingbufferReceiveUpTo(mp->queue, &size, 0, read_size)) != NULL) { // no wait
#endif
			    if (mp->pttoff_timer == 0) { // if DAC is disabled

					GPIO.func_out_sel_cfg[mp->gpio_pin].func_sel = mp->sd_func; // enable

    				//if (port == 0) gpio_set_level(TIMER_BUSY_PIN, 1); // PTT on
			    }

#ifdef USEQUEUE
		    	mp->bitq |= byte << mp->bitq_cnt;
		    	mp->bitq_cnt += 8;
		    	//mp->bitq = byte | 0x100; // sentinel 
		    	//mp->bitq = *rbuf | 0x100; // sentinel 
#else 
		 
#if 1
		    	//mp->bitq = rbuf[0];
		    	for (int i = 0; i < size; i++) {
					mp->bitq |= rbuf[i] << mp->bitq_cnt;
					mp->bitq_cnt += 8;
			    }
#else
			    for (int i = 0; i < size; i++) {
					*((uint8_t *)&mp->bitq + i) = rbuf[i];
		    	}
#endif
		    	vRingbufferReturnItem(mp->queue, rbuf);

		    	//mp->bitq_cnt = size * 8; // bit count in bitq
#endif

#define WBUF_SIZE 3 // wave buffer size

		    	//mp->pttoff_timer = WBUF_SIZE + 3; // PTT off (val - 3) * (1/1200) sec after sending data
		    	mp->pttoff_timer = mp->bitq_cnt + 8; // PTT off timer * (1/1200) sec after sending data

			} else { // no data

			    //gpio_set_level(TIMER_BUSY_PIN, 1); // busy
		    	if (mp->bitq_cnt <= 0) send_queue_empty++;

		    	if (port == BUSY_PORT) {
		    	}


		    	//mp->bitq_cnt += 8; // read queue every 8 bits

		    	//ESP_LOGI(TAG, "xQueueReceive() fail");
		    	//mp->bitq = 0x100; // bit queue is empty
		    	//mp->bitq = 0; // bit queue is empty
		    	//mp->ptt_state = PTT_OFF; // send all data

			}

	    }

	    if (mp->bitq_cnt > 0) { // queue has data

	    	// NRZI processing
#if 0
	    	mp->level ^= !(mp->bitq & 1); // invert level if bit == 0
#else
	    	if (!(mp->bitq & 1)) mp->level = !mp->level;
#endif
	    	//mp->bitq >>= 1;
#ifdef ENABLE_TCM3105
			if (tp->enable_tcm3105) {
				// set level for TCM3105
				int sd_level;

				if (mp->level) {
					sd_level = -128; // mark (1200Hz)
				} else {
					sd_level = 127; // space (2200Hz)
				}

				for (int j = 0; j < RATE_FACTOR; j++) {
					buf[port + num_q * j] = sd_level;
				}

			} else
#endif // ENABLE_TCM3105
			{

		    	if (mp->level) {
			    	inc = MARK_INC;
				} else {
			    	inc = SPACE_INC;
				}

				// store one bit of audio data (11 samples)
				for (int j = 0; j < RATE_FACTOR; j++) {
				    mp->phase += inc;
		    		//mp->phase %= SIN_CYCLES;
		    		if (mp->phase >= SIN_CYCLES) mp->phase -= SIN_CYCLES;
		    		buf[port + num_q * j] = sin_tab[mp->phase];
		    		//buf[port + num_q * j] = table[mp->phase];
				}

			}

			mp->bitq >>= 1;
			mp->bitq_cnt--;

	    } else if (mp->pttoff_timer > 0) { // queue is empty

			if (--mp->pttoff_timer == 0) {
		    	if (port == 0) {
					//gpio_set_level(TIMER_BUSY_PIN, 0); // PTT off
		    	}

		    	// PTT off
#ifndef M5STICKC_AUDIO
		    	gpio_set_level(tp->ptt_pin, 0);
#endif
#ifdef M5ATOM
		    	m5atom_led_set_level(M5ATOM_LED_RED, 0);
#endif
		    	//ESP_LOGI(TAG, "ptt off: pin = %d, port = %d", tp->ptt_pin, tp->port);

		    	GPIO.func_out_sel_cfg[mp->gpio_pin].func_sel = SIG_GPIO_OUT_IDX; // disable DAC output

		    	tp->ptt = false;
#ifdef FX25TNCR2
		    	gpio_set_level(tp->sta_led_pin, 0); // STA LED off
#endif
			}

	    }

	    //if (port == BUSY_PORT) gpio_set_level(TIMER_BUSY_PIN, 0); // free
	    //gpio_set_level(TIMER_BUSY_PIN, 0); // free

	}

	//gpio_set_level(MOD_BUSY_PIN, 0); // free

	// give buffer
	if (xRingbufferSendComplete(rb_isr, buf) != pdTRUE) {
	    ESP_LOGI(TAG, "xRingbufferSendComplete() fail");
	}

	exec_time = esp_timer_get_time() - exec_time;
	if (exec_time > timer_exec_time) timer_exec_time = exec_time;

	//mod_ccount = ccount() - cc0;
    }
}
#endif

void timer_isr_register_task(void *p)
{
    ESP_ERROR_CHECK(timer_isr_register(TIMER_GROUP_NUM, TIMER_NUM, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL));
    //vTaskDelay(1000 / portTICK_PERIOD_MS);

    //vTaskResume((TaskHandle_t)p);
    vTaskDelete(NULL);
}

void timer_initialize(tcb_t *p, int num_queue)
{
    const timer_config_t tc = {
		.alarm_en = TIMER_ALARM_EN,
		.counter_en = TIMER_PAUSE,
		.intr_type = TIMER_INTR_LEVEL,
		.counter_dir = TIMER_COUNT_UP,
		.auto_reload = TIMER_AUTORELOAD_DIS,
		.divider = TIMER_DIVIDER,
    };
    //timer_isr_handle_t handle;
    sigmadelta_config_t sd_conf = {
		.channel = SIGMADELTA_CHANNEL,
		.sigmadelta_duty = 0,
		.sigmadelta_prescale = 0,
		.sigmadelta_gpio = SIGMADELTA_GPIO_PIN,
    };

    tp = p;

    //ESP_ERROR_CHECK(gpio_set_direction(SIGMADELTA_GPIO_PIN, GPIO_MODE_OUTPUT));

    if ((num_queue <= 0)  || (num_queue > MODULATORS)) {
		ESP_LOGE(TAG, "number of queue invalid, num_queue = %d", num_queue);
		assert(false);
    }

    for (int i = 0; i < num_queue; i++) {

		sd_conf.channel = i;
		sd_conf.sigmadelta_gpio = sigmadelta_gpio_pins[i];

		ESP_LOGI(TAG, "port[%d] output pin GPIO%d", i, sigmadelta_gpio_pins[i]);

		ESP_ERROR_CHECK(sigmadelta_config(&sd_conf));

		mod_t *mp = &modulator[i];

		mp->sd_ch = i;
		mp->queue = tp[i].queue; // data receiving queue
		mp->phase = 0;
		//mp->inc = MARK_INC;
		mp->bitq = 0;
		//mp->cycle = 0;
		mp->level = 0;

		mp->gpio_pin = sigmadelta_gpio_pins[i];
		mp->sd_func = GPIO.func_out_sel_cfg[mp->gpio_pin].func_sel; // save function number for SD

		ESP_LOGI(TAG, "pad[%d].func_sel: %d, oen_sel: %d", mp->gpio_pin, GPIO.func_out_sel_cfg[mp->gpio_pin].func_sel, GPIO.func_out_sel_cfg[mp->gpio_pin].oen_sel);

		GPIO.func_out_sel_cfg[mp->gpio_pin].func_sel = SIG_GPIO_OUT_IDX; // disable DAC output

		//queues[i] = queue[i];
    }
    num_q = num_queue;

    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_NUM, TIMER_NUM, &tc));
    ESP_LOGI(TAG, "timer_init");
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_NUM, TIMER_NUM, TIMER_ALARM_VALUE));
    ESP_LOGI(TAG, "timer_set_alarm_value: %d", TIMER_ALARM_VALUE);

    // register timer_isr
#if 0
    ESP_ERROR_CHECK(timer_isr_register(TIMER_GROUP_NUM, TIMER_NUM, timer_isr, (void *)num_queue, ESP_INTR_FLAG_IRAM, &handle));
#else
    // register timer_isr to CPU 1
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    assert(xTaskCreatePinnedToCore(timer_isr_register_task, "isr register", 4096, (void *)task, tskIDLE_PRIORITY, NULL, 1) == pdPASS); // CPU ID 1
    //vTaskSuspend(task); // wait for register task execution
#endif 

    ESP_LOGI(TAG, "timer_isr_register");
    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_NUM, TIMER_NUM, 0)); // auto reload value
    ESP_LOGI(TAG, "timer_set_counter");

    // ringbuffer for DAC audio data, number of DACs, roundup 4 byte align, 8 bytes overhead, double buffer
#define ISR_RINGBUF_SIZE (((((AFSK_CYCLE_BUFSIZE * MODULATORS + 3) / 4) * 4) + 8) * WBUF_SIZE)

    rb_isr = xRingbufferCreate(ISR_RINGBUF_SIZE, RINGBUF_TYPE_NOSPLIT);
    assert(rb_isr != NULL);

    //assert(xTaskCreatePinnedToCore(mod_task, "mod task", 4096, modulator, configMAX_PRIORITIES - 1, NULL, tskNO_AFFINITY) == pdPASS);
    //assert(xTaskCreatePinnedToCore(mod_task, "mod task", 4096, modulator, tskIDLE_PRIORITY + 10, NULL, tskNO_AFFINITY) == pdPASS);
    //
    assert(xTaskCreatePinnedToCore(mod_task, "mod task", 4096, modulator, tskIDLE_PRIORITY + 24, NULL, 0) == pdPASS);

    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_NUM, TIMER_NUM));
    ESP_LOGI(TAG, "timer_start");

    ESP_LOGI(TAG, "divider: %u", TIMERG0.hw_timer[0].config.divider);
    ESP_LOGI(TAG, "cnt_low: %u", TIMERG0.hw_timer[0].cnt_low);
    ESP_LOGI(TAG, "cnt_high: %u", TIMERG0.hw_timer[0].cnt_high);
    ESP_LOGI(TAG, "alarm_low: %u", TIMERG0.hw_timer[0].alarm_low);
    //TIMERG0.hw_timer[0].alarm_low += 500;
    ESP_LOGI(TAG, "alarm_low: %u", TIMERG0.hw_timer[0].alarm_low);
    ESP_LOGI(TAG, "alarm_high: %u", TIMERG0.hw_timer[0].alarm_high);
    ESP_LOGI(TAG, "load_low: %u", TIMERG0.hw_timer[0].load_low);
    ESP_LOGI(TAG, "load_high: %u", TIMERG0.hw_timer[0].load_high);

    //gpio_reset_pin(TIMER_BUSY_PIN);
    //gpio_set_direction(TIMER_BUSY_PIN, GPIO_MODE_OUTPUT);

    //gpio_reset_pin(MOD_BUSY_PIN);
    //gpio_set_direction(MOD_BUSY_PIN, GPIO_MODE_OUTPUT);
}

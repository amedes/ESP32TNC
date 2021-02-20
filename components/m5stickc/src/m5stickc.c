/* M5StickC Plus support routine

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"

//#include "decode_image.h"
//#include "pretty_effect.h"

#include "config.h"

#include "m5stickc.h"

#include "axp192.h"
#include "font.h"
//#include "packet_table.h"
#include "spi.h"
#include "lcd.h"
#include "bm8563.h"
#include "mpu6886.h"
#include "beep.h"
#include "led.h"

/*
 This code displays some fancy graphics on the 320x240 LCD on an ESP-WROVER_KIT board.
 This example demonstrates the use of both spi_device_transmit as well as
 spi_device_queue_trans/spi_device_get_trans_result and pre-transmit callbacks.

 Some info about the ILI9341/ST7789V: It has an C/D line, which is connected to a GPIO here. It expects this
 line to be low for a command and high for data. We use a pre-transmit callback here to control that
 line: every transaction has as the user-definable argument the needed state of the D/C line and just
 before the transaction is sent, the callback will set this line to the correct state.
*/

#define M5STICKC_PLUS 1
#define LCD_SPI_NUM SPI2_HOST

#ifdef M5STICKC_PLUS
#define PIN_NUM_MISO -1	// not used
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK  13
#define PIN_NUM_CS   5

#define PIN_NUM_DC   23
#define PIN_NUM_RST  18
//#define PIN_NUM_BCKL 0 // not supported

#define TFT_WIDTH (320)
//#define TFT_HEIGHT 320
#define TFT_HEIGHT (240)
#else
#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   22

#define PIN_NUM_DC   21
#define PIN_NUM_RST  18
#define PIN_NUM_BCKL 5
#endif

//#define LCD_OFF_TIME_BATT 2
//#define LCD_OFF_TIME_VBUS 10

#define BEEP_HIGH	4096	// Hz
#define BEEP_LOW	2048
#define BEEP_SHORT	100	// ms
#define BEEP_LONG	500

#define M5STICKC_SYS_INT GPIO_NUM_35
#define M5STICKC_LED GPIO_NUM_10
#define EVENT_SYSINT 0x0192 // magic number 
#define EVENT_M5BUTTONA 0x0037 // M5 button A
#define EVENT_M5BUTTONB 0x0039 // M5 button B
#define M5BUTTONA GPIO_NUM_37
#define M5BUTTONB GPIO_NUM_39

#define LCD_COLOR_DEFAULT 0x1f	// foreground white, background blue
#define LCD_COLOR_LOG 0x1a	// bright green for log message

static QueueHandle_t int_queue;
static uint8_t lcd_disp = true;

static void IRAM_ATTR sys_int_isr(void *arg)
{
    uint32_t item = (uint32_t)arg;
    BaseType_t tskWoken = pdFALSE;

    xQueueSendFromISR(int_queue, &item, &tskWoken);

    if (tskWoken) {
	portYIELD_FROM_ISR();
    }
}

static void sys_int_init(QueueHandle_t queue)
{
#if 0
    gpio_config_t conf = {
	.pin_bit_mask = 1ULL << M5STICKC_SYS_INT,
	.mode = GPIO_MODE_INPUT,
	.intr_type = GPIO_INTR_NEGEDGE,
	//.intr_type = GPIO_INTR_LOW_LEVEL,
    };
#endif

    int_queue = queue;

    ESP_ERROR_CHECK(gpio_reset_pin(M5STICKC_SYS_INT));
    ESP_ERROR_CHECK(gpio_set_direction(M5STICKC_SYS_INT, GPIO_MODE_INPUT));

    //ESP_ERROR_CHECK(gpio_set_pull_mode(M5STICKC_SYS_INT, GPIO_PULLUP_ONLY));
    //ESP_ERROR_CHECK(gpio_config(&conf));
    //ESP_ERROR_CHECK(gpio_isr_register(sys_int_isr, NULL, ESP_INTR_FLAG_IRAM, NULL));

#define ESP_INTR_FLAG_DEFAULT (0)
 
    ESP_ERROR_CHECK(gpio_set_intr_type(M5STICKC_SYS_INT, GPIO_INTR_NEGEDGE));
    //ESP_ERROR_CHECK(gpio_set_intr_type(M5STICKC_SYS_INT, GPIO_INTR_LOW_LEVEL));

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    ESP_ERROR_CHECK(gpio_isr_handler_add(M5STICKC_SYS_INT, sys_int_isr, (void *)EVENT_SYSINT));
    ESP_ERROR_CHECK(gpio_intr_enable(M5STICKC_SYS_INT));

    // LED enable
    //ESP_ERROR_CHECK(gpio_reset_pin(M5STICKC_LED));
    //ESP_ERROR_CHECK(gpio_set_direction(M5STICKC_LED, GPIO_MODE_OUTPUT_OD));
    //ESP_ERROR_CHECK(gpio_set_level(M5STICKC_LED, 1)); // off, negative logic

    // M5 button A (GPIO_NUM_37)
    ESP_ERROR_CHECK(gpio_reset_pin(M5BUTTONA));
    ESP_ERROR_CHECK(gpio_set_direction(M5BUTTONA, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_intr_type(M5BUTTONA, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(M5BUTTONA, sys_int_isr, (void *)EVENT_M5BUTTONA));
    ESP_ERROR_CHECK(gpio_intr_enable(M5BUTTONA));

    // M5 button B (GPIO_NUM_39)
    ESP_ERROR_CHECK(gpio_reset_pin(M5BUTTONB));
    ESP_ERROR_CHECK(gpio_set_direction(M5BUTTONB, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_intr_type(M5BUTTONB, GPIO_INTR_NEGEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(M5BUTTONB, sys_int_isr, (void *)EVENT_M5BUTTONB));
    ESP_ERROR_CHECK(gpio_intr_enable(M5BUTTONB));

    return;

    // clear irq
    for (int reg = 0x44; reg < 0x4e; reg++) {
	if (reg == 0x48) reg = 0x4d;
	uint8_t val = axp192_readReg(reg);

	if (val) {
	    printf("irq[%02x] = %02x\n", reg, val);
	    axp192_writeReg(reg, val);
	}
    }

    // set timer
    printf("timer: %02x\n", axp192_readReg(0x8a));
    axp192_writeReg(0x8a, 0x80); // timer irq clear
    axp192_writeReg(0x8a, 1); // 1 minute, timer irq clear
    axp192_writeReg(0x4a, axp192_readReg(0x4a) | 0x80); // timer irq enable
}

#define MAGIC_BATTERY 192 // 1 byte

#define LCD_PRINT 1

static void print_battery_info(bool lcd)
{
    static uint8_t buf[128];
    char *s = (char *)buf;
    uint32_t val;
    int c;
    struct tm *tp;

    // time
    tp = localtime(&axp192.time);
    if (tp->tm_year + 1900 > 2020) { // time is valid
	int timezone = bm8563.timezone;
	char sign = '-';

	if (timezone <= 0) {
	    timezone = -timezone;
	    sign = '+';
	}

	c = sprintf(s, "Time: %04d-%02d-%02dT%02d:%02d:%02d%c%02d\n",
		tp->tm_year + 1900, tp->tm_mon + 1, tp->tm_mday, // year, month, day
		tp->tm_hour, tp->tm_min, tp->tm_sec, // hour, min, sec
		sign, timezone / 3600);

	if (lcd) tty_write(buf, c);
	fputs(s, stdout);
    }

    // Battery voltage, FP3.13
    val = axp192.v_batt; // FP3.13 V
    c = sprintf(s, "Battery voltage: %d.%03d V\n", val >> 13, ((val & 0x1fff) * 1000) >> 13);
    if (lcd) tty_write(buf, c);
    fputs(s, stdout);
	    
    // Battery current in
    val = axp192.i_in_batt; // FP13.3 mA
    if (val) {
	c = sprintf(s, "Battery in: %d.%d mA\n", val >> 3, ((val & 7) * 10) >> 3);
	if (lcd) tty_write(buf, c);
	fputs(s, stdout);
    }

    // Battery current out
    val = axp192.i_out_batt; // FP13.3 mA
    if (val) {
	c = sprintf(s, "Battery out: %d.%d mA\n", val >> 3, (val & 7) * 10 >> 3);
	if (lcd) tty_write(buf, c);
	fputs(s, stdout);
    }

    // Battery power
    val = axp192.p_batt; // FP13.3 mW
    if (val) {
	c = sprintf(s, "Battery power: %d.%d mW\n", val >> 3, ((val & 7) * 10) >> 3);
	if (lcd) tty_write(buf, c);
	fputs(s, stdout);
    }

    // VBUS voltage
    val = axp192.v_vbus; // FP3.13 V
    c = sprintf(s, "VBUS: %d.%03d V, ", val >> 13, ((val & 0x1fff) * 1000) >> 13);
    if (lcd) tty_write(buf, c);
    fputs(s, stdout);

    // VBUS current
    val = axp192.i_vbus; // FP13.3 mA
    c = sprintf(s, "%d.%d mA\n", val >> 3, ((val & 7) * 10) >> 3);
    if (lcd) tty_write(buf, c);
    fputs(s, stdout);

    // APS voltage
    val = axp192.v_aps; // FP3.13 V
    c = sprintf(s, "APS: %d.%03d V\n", val >> 13, ((val & 0x1fff) * 1000) >> 13);
    if (lcd) tty_write(buf, c);
    fputs(s, stdout);

    // AXP192 internal temperature
    val = axp192.t_axp192; // 0.1 C
    c = sprintf(s, "AXP192 temp: %d.%d C\n", val / 10, val % 10);
    if (lcd) tty_write(buf, c);
    fputs(s, stdout);

    // power input status
    val = axp192.power_status;
    c = sprintf(s, "Power source: %s%s%s\n",
	    (val & 0x40) ? "ACIN"
	    : ((val & 0x10) ? "VBUS" : "BATT"),
	    ((val & 0x50) && (val & 0x04)) ? ",charging" : "",
	    (val & 0x01) ? ",start VBUS" : "");
    if (lcd) tty_write(buf, c);
    fputs(s, stdout);

    // battery mode
    val = axp192.charge_status;
    c = sprintf(s, "Power mode: %s%s%s%s%s%s\n",
	    (val & 0x80) ? "over temp," : "",
	    (val & 0x40) ? "charging," : "",
	    (val & 0x20) ? "" : "no battery,",
	    (val & 0x08) ? "battery activation mode," : "",
	    (val & 0x04) ? "charging current too low," : "",
	    (val & 0x02) ? "mode B" : "mode A"
	    );
    fputs(s, stdout);
    if (lcd) tty_write(buf, c);

#if 0
    // USB OTG VBUS status
    val = axp192.usb_otg_status;
    c = sprintf(s, "USB OTG status: %s%s%s\n",
	    (val & 0x04) ? "VBUS," : "",
	    (val & 0x02) ? "Session A/B," : "",
	    (val & 0x01) ? "Session end" : "");
    fputs(s, stdout);
#ifdef LCD_PRINT
    tty_write(buf, c);
#endif
#endif

    // Coulomb counter
    // Battery charge / discharge
    // C = val * 65536 * (1/2) / 3600 / 200Hz [mAh]
    c = sprintf(s, "Charge cycle in/out: %u/%u\n",
	    axp192.c_in_batt  * 32 / 84375,  // charge / 120mAh
	    axp192.c_out_batt * 32 / 84375); // discharge / 120mAh
    fputs(s, stdout);
    if (lcd) tty_write(buf, c);
	    
    if (axp192.magic == MAGIC_BATTERY) {

	c = sprintf(s, "Battery capacity: %u.%u mAh\n", axp192.batt_capacity / 10, axp192.batt_capacity % 10);
	fputs(s, stdout);
	if (lcd) tty_write(buf, c);
    }

    if (axp192.discharge_time > 0 && axp192.ai_batt > 0) { // in battery operation mode
	
	c = sprintf(s, "Average current: %d.%d mA\n", axp192.ai_batt / 10, axp192.ai_batt % 10);
	fputs(s, stdout);
	if (lcd) tty_write(buf, c);

	c = sprintf(s, "Activity time: %d min\n", axp192.batt_time / 60);
	fputs(s, stdout);
	if (lcd) tty_write(buf, c);

	c = sprintf(s, "Activity limit: %d min\n", axp192.time_left / 60);
	fputs(s, stdout);
	if (lcd) tty_write(buf, c);
    }
}

#define SYSINT_ACIN_OVER 0x4480
#define SYSINT_ACIN_CONN 0x4440
#define SYSINT_ACIN_DISC 0x4420

#define SYSINT_VBUS_OVER 0x4410
#define SYSINT_VBUS_CONN 0x4408
#define SYSINT_VBUS_DISC 0x4404
#define SYSINT_VBUS_VHOLD 0x4402

#define SYSINT_BAT_CONN 0x4580
#define SYSINT_BAT_DISC 0x4540
#define SYSINT_BAT_MODE_ON 0x4520
#define SYSINT_BAT_MODE_OFF 0x4510
#define SYSINT_BAT_CHG_START 0x4508
#define SYSINT_BAT_CHG_END 0x4504

#define SYSINT_AXP_OVER 0x4680
#define SYSINT_AXP_CHGC_LOW 0x4640
#define SYSINT_AXP_DC1_LOW 0x4620
#define SYSINT_AXP_PKEY_SHORT 0x4602
#define SYSINT_AXP_PKEY_LONG 0x4601

#define SYSINT_N_OE_BOOT 0x4780
#define SYSINT_N_OE_SHUT 0x4740
#define SYSINT_VBUS_ACTIVE 0x4720
#define SYSINT_VBUS_INACTIVE 0x4710
#define SYSINT_VBUS_SES_AB 0x4708
#define SYSINT_VBUS_SES_END 0x4704
#define SYSINT_APS_LOW 0x4701

#define SYSINT_TIMER 0x4d80

#define N_VBUSEN_PIN GPIO_NUM_27

#if 0
static void update_axp192_info(void)
{
    uint32_t x;

    axp192.magic = axp192_readReg(0x06);	// Magic number
    axp192.c_diff = axp192_readRegs(0x07, 2);	// Coulomb difference

    x = axp192_readRegs(0x00, 3); // power status etc.
    axp192.power_status = x >> 16;
    axp192.charge_status = x >> 8;
    axp192.usb_otg_status = x;

    if ((x >> 16) & 0x50) { // power by ACIN/VBUS
	axp192.lcd_timer_default = LCD_OFF_TIME_VBUS;
    } else {
	axp192.lcd_timer_default = LCD_OFF_TIME_BATT;
    }

    // adjust LCD off timer
    if (axp192.lcd_timer > 0 && axp192.lcd_timer > axp192.lcd_timer_default) {
	axp192.lcd_timer = axp192.lcd_timer_default;
    }

#define M_ACIN 912681 // 1.7e-3 * 2^29

    x = axp192_readRegs(0x56, 4); // ACIN voltage, current
    x = ((x & 0xff00ff00) >> 4) | (x & 0x000f000f); // calculate two 12bit value
    axp192.v_acin = ((x >> 16) * M_ACIN) >> 16;	// ACIN voltage, FP3.13 V
    axp192.i_acin = (uint16_t)x * 5;		// ACIN current, FP13.3 mA, 0.625 * 2^3 = 5

#define M_VBUS 912681 // 1.7e-3 * 2^29

    x = axp192_readRegs(0x5a, 4); // VBUS voltage, current
    x = ((x & 0xff00ff00) >> 4) | (x & 0x000f000f); // calculate two 12bit value
    axp192.v_vbus = ((x >> 16) * M_VBUS) >> 16;	// VBUS voltage, FP3.13 V
    axp192.i_vbus = (uint16_t)x * 3;		// VBUS current, FP13.3 mA, 0.375 * 2^3 = 3;

#define TEMP_OFFSET (-1447)

    x = axp192_readRegs(0x5e, 2); // AXP192 temp
    x = ((x & 0xff00) >> 4) | (x & 0x000f); // calculate 12bit value
    axp192.t_axp192 = x + TEMP_OFFSET;	// AXP192 temp, unit 0.1 C

#if 0
    x = axp192_readRegs(0x62, 2); // Battery temp
    x = ((x & 0xff00) >> 4) | (x & 0x000f); // calculate 12bit value
    axp192.t_batt = x;	// Battery temp

    x = axp192_readRegs(0x64, 4); // GPIO0 voltage, GPIO1 voltage
    x = ((x & 0xff00ff00) >> 4) | (x & 0x000f000f); // calculate two 12bit value
    axp192.v_gpio0 = x >> 16;	// GPIO0 voltage
    axp192.v_gpio1 = (uint16_t)x; // GPIO1 voltage

    x = axp192_readRegs(0x68, 4); // GPIO2 voltage, GPIO3 voltage
    x = ((x & 0xff00ff00) >> 4) | (x & 0x000f000f); // calculate two 12bit value
    axp192.v_gpio2 = x >> 16;	// GPIO2 voltage
    axp192.v_gpio3 = (uint16_t)x; // GPIO3 voltage
#endif

#define M_POWER 288 // 1.1 * 0.5 / 1000 * 2^19

    x = axp192_readRegs(0x70, 3); // Battery power
    axp192.p_batt = (axp192_readRegs(0x70, 3) * M_POWER) >> 16; // Battery power, FP13.3 mW

#define M_BATT 590558 // 1.1mV * 2^29

    x = axp192_readRegs(0x78, 2); // Battery voltage
    x = ((x & 0xff00) >> 4) | (x & 0x000f); // calculate 12bit value
    axp192.v_batt = (x * M_BATT) >> 16; // Battery voltage, FP3.13

    x = axp192_readRegs(0x7a, 4); // Battery in/out current
    x = ((x & 0xff00ff00) >> 3) | (x & 0x001f001f); // calculate two 13bit value
    axp192.i_in_batt = (x >> 16) * 4;	// Battery charge current, FP13.3 mA
    axp192.i_out_batt = (uint16_t)x * 4; // Battery discharge current, FP13.3 mA

#define M_APS 751619 // 1.4mV * 2^29

    x = axp192_readRegs(0x7e, 2); // APS voltage
    x = ((x & 0xff00) >> 4) | (x & 0x000f); // calculate 12bit value
    axp192.v_aps = (x * M_APS) >> 16;	// APS voltage, FP3.13 mV

    axp192.c_in_batt = axp192_readRegs(0xb0, 4); // Battery charge Coulomb
    axp192.c_out_batt = axp192_readRegs(0xb4, 4); // Battery discharge Coulomb

#if 0
    // stop charging if battery voltage > 4.0 V
    if (axp192_readReg(0x01) & 0x40) { // charging
	uint16_t val = axp192_readRegs(0x78, 2);
	val = ((val >> 4) & ~0x0f) | (val & 0x0f);

#define VOLT_4V (40000 / 11) // 4.0V in 1.1mV unit

	//printf("battery voltage ADC: %d\n", val);

	if (val > VOLT_4V) {

	    //printf("charging stop\n");
	    //static const char s[] = "Stop charging, exceed 4.0 V";
	    //ESP_LOGI("BATT", "%s", s);
	    //lcd_print((uint8_t *)s, strlen(s));
	    ESP_LOGI("BATT", "Stop charging, exceed 4.0 V");

	    // stop charging
	    uint8_t reg33 = axp192_readReg(0x33);
	    axp192_writeReg(0x33, reg33 & 0x7f); // clear charge bit (bit7)
	    axp192_writeReg(0x33, reg33); // restore original value
	}

    }
#endif

    // update Battery capacity estimation
    int32_t diff = axp192.c_in_batt	// Charge Coulomb counter
	         - axp192.c_out_batt;	// Discharge Coulomb counter

    // check magic number
    if (axp192.magic == MAGIC_BATTERY) {

	if (diff >= axp192.c_diff) return; // no need to update

    } else {

	// initialize Coulomb valriables
	axp192_writeReg(0x06, MAGIC_BATTERY); // set magic number
    }

    if (diff > 32767 || diff < -32768) { // Coulomb counter value is invalid

	axp192_writeReg(0xb8, 0x20); // counter clear;
	axp192_writeReg(0xb8, 0x80); // counter start;

	diff = 0; // now, coulomb_diff is zero
    }

    // update coulomb_diff variable in axp192
    axp192_writeReg(0x07, diff >> 8);
    axp192_writeReg(0x08, diff & 0xff);
}
#endif

#define LCD_BRIGHTNESS_VBUS 12
#define LCD_BRIGHTNESS_BATT 9

static char const *sysint_process(int sysint_id)
{
    char *str = NULL;

    switch (sysint_id) {
	case SYSINT_ACIN_OVER:
	    str = "ACIN over voltage";
	    break;

	case SYSINT_ACIN_CONN:
	    str = "ACIN connected";
	    break;

	case SYSINT_ACIN_DISC:
	    str = "ACIN disconnected";
	    break;

	case SYSINT_VBUS_OVER:
	    str = "VBUS over voltage";
	    break;

	case SYSINT_VBUS_CONN:
	    str = "VBUS connected";
	    beep(BEEP_HIGH, BEEP_SHORT, false);
	    update_axp192_info();
	    axp192.discharge_time = 0;
	    axp192_brightness(LCD_BRIGHTNESS_VBUS);
	    break;

	case SYSINT_VBUS_DISC: // start battery operation
	    str = "VBUS disconnected";
	    beep(BEEP_LOW, BEEP_SHORT, false);
	    update_axp192_info();
	    axp192.c_out_batt_origin = axp192.c_out_batt; // record Coulomb discharge value
	    axp192.discharge_time = xTaskGetTickCount(); // record discharge start time
	    axp192_brightness(LCD_BRIGHTNESS_BATT);
	    break;

	case SYSINT_VBUS_VHOLD:
	    str = "VBUS under V_hold";
	    break;

	case SYSINT_BAT_CONN:
	    str = "Battery connected";
	    break;
	case SYSINT_BAT_DISC:
	    str = "Battery disconnected";
	    break;
	case SYSINT_BAT_MODE_ON:
	    str = "Battery mode on";
	    break;
	case SYSINT_BAT_MODE_OFF:
	    str = "Battery mode off";
	    break;
	case SYSINT_BAT_CHG_START:
	    beep(BEEP_HIGH, BEEP_SHORT, false);
	    str = "Battery charging start";
	    update_axp192_info();
	    break;
	case SYSINT_BAT_CHG_END:
	    beep(BEEP_HIGH, BEEP_SHORT, false);
	    str = "Battery charging finish";
	    update_axp192_info();
	    break;

	case SYSINT_AXP_OVER:
	    str = "Over temperature";
	    break;

	case SYSINT_AXP_CHGC_LOW:
	    str = "Charging current too low";
	    break;

	case SYSINT_AXP_DC1_LOW:
	    str = "DC-DC1 voltage too low";
	    break;

	case SYSINT_AXP_PKEY_SHORT:
	    {
		int batt = !gpio_get_level(N_VBUSEN_PIN);
		beep(1 << (12 - batt), BEEP_SHORT, false); // BATT: 2048Hz, VBUS: 4096Hz
		gpio_set_level(N_VBUSEN_PIN, batt); // 0: VBUS, 1: BATT
	    }
	    str = "Power key short press";
	    break;

	case SYSINT_AXP_PKEY_LONG:
	    beep(BEEP_HIGH, BEEP_SHORT, true);
	    axp192_shutdown(); // power off
	    esp_deep_sleep_start(); // does not return
	    str = "Power key long press";
	    break;

	case SYSINT_N_OE_BOOT:
	    str = "N_OE boot";
	    break;

	case SYSINT_N_OE_SHUT:
	    str = "N_OE shutdown";
	    break;

	case SYSINT_VBUS_ACTIVE:
	    str = "VBUS active";
	    break;

	case SYSINT_VBUS_INACTIVE:
	    str = "VBUS inactive";
	    break;

	case SYSINT_VBUS_SES_AB:
	    str = "VBUS Session A/B";
	    break;

	case SYSINT_VBUS_SES_END:
	    str = "VBUS Session End";
	    break;

	case SYSINT_APS_LOW:
	    str = "APS voltage under Warning Level2";
	    // shutdown to avoid hang up
	    axp192_shutdown(); // power off
	    esp_deep_sleep_start(); // does not return
	    break;

	case SYSINT_TIMER:
	    //str = "Timer expired";
	    str = NULL;

	    update_axp192_info();
#ifdef BATTERY_INFO
	    print_battery_info(false); // output serial only
#endif

	    // LCD off timer
	    if (axp192.lcd_timer > 0 && --axp192.lcd_timer == 0) {
		//axp192_backlight(false); // BL off
		//tty_lcd_sleep(1); // enter LCD sleep mode
		tty_lcd_sleep(true);
		lcd_disp = false;
	    }

	    axp192_writeReg(0x8a, 0x81); // re-enable timer, 1 min
    }

#if 0
    if (sysint_id != SYSINT_TIMER) {
	lcd_sleep(false); // lcd on
	axp192_backlight(true); // BL on
    }
#endif

    return str;
}

static const char TAG[] = "axp192";

#define WDT_TIMEOUT (70 * 1000 / portTICK_PERIOD_MS) // 70sec in tick
#define SYSINT_TIMEOUT (70 * 1000 / portTICK_PERIOD_MS) // 1min
#define EVENT_TIMEOUT 0x55aa


static void sysint_task(void *arg)
{
    QueueHandle_t queue = (QueueHandle_t)arg;
    uint32_t item;
    static uint32_t m5a_time = 0;
    static uint32_t m5b_time = 0;

    while (1) {
	if (xQueueReceive(queue, &item, SYSINT_TIMEOUT) != pdTRUE) {
	    //ESP_LOGW(TAG, "xQueueReceive() fail");
	    //continue;
	    item = EVENT_TIMEOUT;
	}

	switch (item) {
	    case EVENT_TIMEOUT:
		ESP_LOGW(TAG, "timeout occur");
		/* FALLTHROUGH */

	    case EVENT_SYSINT:
		for (int reg = 0x44; reg <= 0x4d; reg++) { // scan irq status reg.
		    if (reg == 0x48) reg = 0x4d;
		    uint8_t val = axp192_readReg(reg);  // get irq status

		    if (!val) continue;

		    axp192_writeReg(reg, val); // clear irq status

		    for (uint8_t m = 1; m; m <<= 1) { // check bits
			if (val & m) {
			    char const *s = sysint_process((reg << 8) | m); // parameter is event ID

			    // output message to log
			    if (s) {
				ESP_LOGI(TAG, "%s", s);
				//printf("%s\n", s);

				// output to LCD
				//uint8_t color = lcd_get_color();
				//lcd_set_color((color & 0xf0) | 0x0a); // set green
				tty_lcd_color(LCD_COLOR_LOG);
				tty_write((uint8_t *)s, strlen(s));
				tty_write("\n", 1);
				tty_lcd_color(LCD_COLOR_DEFAULT);
			    }
			}
		    }
		}
		break;

	    case EVENT_M5BUTTONA:
		{
		    uint32_t t = xTaskGetTickCount();

		    if (t - m5a_time > 500 / portTICK_PERIOD_MS) {
			beep(BEEP_HIGH, BEEP_SHORT, false);
			tty_lcd_sleep(lcd_disp);
			lcd_disp = !lcd_disp;
			//tty_lcd_sleep(false); // lcd on
			//axp192_backlight(true); // BL on
			//print_battery_info();
			//axp192_writeReg(0x8a, 0x81); // reset BL off timer
			if (lcd_disp) {
			    axp192.lcd_timer = axp192.lcd_timer_default;
			} else {
			    axp192.lcd_timer = 0;
			}
		    }
		    m5a_time = t;
		}
		break;

	    case EVENT_M5BUTTONB:
		{
		    uint32_t t = xTaskGetTickCount();

		    // to avoid chattering
		    if (t - m5b_time > 500 / portTICK_PERIOD_MS) { // more than 500 ms
			//lcd_sleep(false); // lcd on
			//axp192_backlight(1); // BL on
			beep(BEEP_HIGH, BEEP_SHORT, false);
			update_axp192_info();
			print_battery_info(true);
		    }
		    m5b_time = t;
		}
		break;
	
	    default:
		ESP_LOGW(TAG, "unexpected Event ID: %x", item);
	}
    }
}

void show_reset_reason(void)
{
    esp_reset_reason_t reason = esp_reset_reason();

    switch (reason) {
	case ESP_RST_UNKNOWN:
	    ESP_LOGI(TAG, "ESP_RST_UNKNOWN");
	    break;

	case ESP_RST_POWERON:
	    ESP_LOGI(TAG, "ESP_RST_POWERON");
	    break;

	case ESP_RST_EXT:
	    ESP_LOGI(TAG, "ESP_RST_EXT");
	    break;

	case ESP_RST_SW:
	    ESP_LOGI(TAG, "ESP_RST_SW");
	    break;

	case ESP_RST_PANIC:
	    ESP_LOGI(TAG, "ESP_RST_PANIC");
	    break;

	case ESP_RST_INT_WDT:
	    ESP_LOGI(TAG, "ESP_RST_INT_WD");
	    break;

	case ESP_RST_TASK_WDT:
	    ESP_LOGI(TAG, "ESP_RST_TASK_WD");
	    break;

	case ESP_RST_WDT:
	    ESP_LOGI(TAG, "ESP_RST_WD");
	    break;

	case ESP_RST_DEEPSLEEP:
	    ESP_LOGI(TAG, "ESP_RST_DEEPSLEEP");
	    break;

	case ESP_RST_BROWNOUT:
	    ESP_LOGI(TAG, "ESP_RST_BROWNOUT");
	    break;

	case ESP_RST_SDIO:
	    ESP_LOGI(TAG, "ESP_RST_SDIO");
	    break;

	default:
	    ESP_LOGI(TAG, "reset reason: %d", reason);
    }
}

void m5stickc_init(void)
{
    spi_device_handle_t spi;

    // reset reason
    show_reset_reason();

    // RTC BM8563
    bm8563_init();

    // Motion sensor, set sleep mode
    mpu6886_init();

    // initialize the SPI bus and add the LCD to the bus
    spi_init(&spi, LCD_SPI_NUM, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS, PIN_NUM_DC);

    //Initialize the LCD
    lcd_init(spi, PIN_NUM_RST);

    // beep initialization
    beep_init();

    // LED initialization
    led_init();


    // AXP192 initialization (turn on LCD)
    axp192_init();

    // read AXP192 registers
    update_axp192_info();
    
    // LCD off timer set
    axp192.lcd_timer = axp192.lcd_timer_default;

    // Coulomb counter initialize, if battery mode
    if ((axp192.power_status & 0x50) == 0x00) {
	axp192.c_out_batt_origin = axp192.c_out_batt; // record Coulomb counter value
	axp192.discharge_time = xTaskGetTickCount(); // record start time
    } else {
	axp192_brightness(LCD_BRIGHTNESS_VBUS);
    }

    //vTaskDelay(1000 / portTICK_PERIOD_MS);

    //display_font(spi);

    lcd_text_init(spi);

    // initialize tty interface
    tty_init();

#if 0
    // splash screen
    vTaskDelay(100 / portTICK_PERIOD_MS);
    lcd_madctl(0x68); // MX=MV=RGB=1
    int ret=decode_image(spi);
    ESP_ERROR_CHECK(ret);
    lcd_madctl(0x48); // MX=RGB=1
#endif

    //vTaskDelay(5000 / portTICK_PERIOD_MS);

#define SYSINT_QUEUE_LEN 8

    QueueHandle_t queue = xQueueCreate(SYSINT_QUEUE_LEN, sizeof(uint32_t));
    assert(queue != NULL);

    assert(xTaskCreatePinnedToCore(sysint_task, "sysint_task", 2048, queue, tskIDLE_PRIORITY, NULL, tskNO_AFFINITY) == pdPASS);

    // enable axp192 interrupt
    sys_int_init(queue);

#if 0
    vTaskDelete(NULL);

    while (1) {
	uint8_t const *pkt = packet_table;
	int n;
	uint8_t buf[128];
	char *s = (char *)buf;
	while ((n = *pkt++) > 0) {
	    uint32_t t0 = esp_timer_get_time();
	    //lcd_print(pkt, n - 2);
	    //lcd_print((uint8_t *)"\n", 1);
	    pkt += n;
	    t0 = esp_timer_get_time() - t0;

	    //int c = sprintf(s, "%d us\n", t0);
	    //lcd_print(buf, c);

	    // print bm8563 time registers
	    //bm8563_print();

	    vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
    }
#endif
}

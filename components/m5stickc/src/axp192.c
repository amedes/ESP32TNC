/*
 * AXP192 control routine for M5StickC Plus
 */
#include <sys/time.h>
#include "driver/i2c.h"
#include "esp_log.h"

#include "config.h"
#include "m5stickc.h"
#include "i2c.h"
#include "lcd.h"

#define M5_I2C_NUM I2C_NUM_0
#define ACK_CHECK_EN 1
#define ACK_CHECK_DIS 0
#define ACK_VAL 0
#define NACK_VAL 1
#define AXP192_ADDR 0x34

static const char TAG[] = "axp192";

// battery management information
struct AXP192 axp192;

uint8_t axp192_readReg(uint8_t reg)
{
    return i2c_read_1byte(AXP192_ADDR, reg);
}

uint32_t axp192_readRegs(uint8_t reg, int len)
{
    uint8_t data[4], *p;
    uint32_t val;

    if (len > 4) return -1;

    i2c_read_bytes(AXP192_ADDR, reg, data, len);

    p = data;
    val = *p++;
    while (--len > 0) {
	val <<= 8;
	val |= *p++;
    }

    return val;
}

void axp192_writeReg(uint8_t reg, uint8_t val)
{
    i2c_write_1byte(AXP192_ADDR, reg, val);
}

void axp192_init(void)
{
    // initialize I2C driver
    i2c_init();

    // Disable LDO2 (LCD Back Light)
    //axp192_writeReg(0x12, axp192_readReg(0x12) & ~0x04);
    // Enable DC-DC1 only
    axp192_writeReg(0x12, 0x01);

    // Set LDO2 (LCD LED) 2.6V & LDO3(ST7789v 2.75V) 2.3V
    axp192_writeReg(0x28, 0x95);

    // Set Voff voltage, 3.3V
    axp192_writeReg(0x31, 0x07);

    // Set warning level1 voltage, 3.4496V
    axp192_writeReg(0x3a, 0x68);
 
    // Set warning level2 voltage, 3.3992V
    axp192_writeReg(0x3b, 0x5f);

    // battery charge setting
#ifdef AXP192_CHG_VOLTAGE_V41
#define AXP192_CHG_VOLTAGE (0 << 5)
#elif defined(AXP192_CHG_VOLTAGE_V415)
#define AXP192_CHG_VOLTAGE (1 << 5)
#elif defined(AXP192_CHG_VOLTAGE_V42)
#define AXP192_CHG_VOLTAGE (2 << 5)
#else
#define AXP192_CHG_VOLTAGE (0 << 5)
#endif

#ifdef AXP192_CHG_10PERCENT
#define AXP192_CHG_CURRENT (0 << 4)
#elif defined(AXP192_CHG_15PERCENT)
#define AXP192_CHG_CURRENT (1 << 4)
#else
#define AXP192_CHG_CURRENT (0 << 4)
#endif

    // Bat charge voltage to 4.1V, Current 100mA, stop at 15%
    //axp192_writeReg(0x33, 0x90);
    // Bat charge voltage to 4.1V, Current 100mA, stop at 10%
    //axp192_writeReg(0x33, 0x80);
    axp192_writeReg(0x33, 0x80 | AXP192_CHG_VOLTAGE | AXP192_CHG_CURRENT);
    ESP_LOGI(TAG, "reg[0x33] = %02x", 0x80 | AXP192_CHG_VOLTAGE | AXP192_CHG_CURRENT);

    // Enable Bat, ACIN, VBUS, APS adc, disable TS
    axp192_writeReg(0x82, 0xfe);

    // Disable GPIO[0-3] ADC
    axp192_writeReg(0x83, 0x80);

    // Enable EXTEN (5V), LDO3 (ST7789V), LDO2 (LCD Back Light), DC-DC1 (3.3V)
    axp192_writeReg(0x12, axp192_readReg(0x12) | 0x4d);
    // Enable LDO3 (ST7789V), LDO2 (LCD Back Light), DC-DC1 (3.3V)
    //axp192_writeReg(0x12, axp192_readReg(0x12) | 0x0d);

    // PEK short 128ms, long 1s, shutdown 4s
    axp192_writeReg(0x36, 0x0c);

#define AXP192_MIC_ON 1

#ifdef AXP192_MIC_ON
    // Set GPIO0 to LDO (MIC power on)
    axp192_writeReg(0x90, 0x02);

    // Set MIC voltage to 3.3V
    axp192_writeReg(0x91, 0xf0);
#else
    // Set GPIO0 to floating (MIC power off)
    axp192_writeReg(0x90, 0x07);
#endif

    // Set GPIO1 to floating
    axp192_writeReg(0x92, 0x07);

    // Set GPIO2 to floating
    axp192_writeReg(0x93, 0x07);

    // Disable GPIO3, GPIO4
    axp192_writeReg(0x95, 0x00);

#if 1
#define N_VBUSEN GPIO_NUM_27

    ESP_ERROR_CHECK(gpio_reset_pin(N_VBUSEN));
    ESP_ERROR_CHECK(gpio_set_direction(N_VBUSEN, GPIO_MODE_INPUT_OUTPUT));
    //ESP_ERROR_CHECK(gpio_set_level(N_VBUSEN, 1)); // do not use VBUS
    ESP_ERROR_CHECK(gpio_set_level(N_VBUSEN, 0)); // use VBUS
#endif

    // Disable vbus hold limit
    //axp192_writeReg(0x30, 0xe2);
    // Eable vbus hold limit, enable N_VBUSEN pin
    axp192_writeReg(0x30, 0x62);

    // Set temperature protection
    axp192_writeReg(0x39, 0xfc);

    // Enable RTC BAT charge, 3.0V, 200uA
    axp192_writeReg(0x35, 0xa2);

    // Enable bat detection
    axp192_writeReg(0x32, 0x46);

    // Enable IRQs
    // ACIN, VBUS 
    axp192_writeReg(0x40, 0xfe);
    // Battery 
    axp192_writeReg(0x41, 0xfc);
    // axp192 too hot, charge current, DCDC1, short press, long press
    axp192_writeReg(0x42, 0xe3);
    // N_OE boot/shut down, VBUS enable/disable, VBUS Session, APS voltage
    axp192_writeReg(0x43, 0xfd);

    // Enable timer
    axp192_writeReg(0x8a, 0x81); // 1 minute

    // timer, GPIO 2/1/0
    axp192_writeReg(0x4a, 0x80); // enable timer irq

    // Enable Coulomb counter
    axp192_writeReg(0xb8, 0x80);

    // Set ADC sampling rate to 200Hz, TS disable
    axp192_writeReg(0x84, 0xc2);

    // Sense resister setting?, default value 0x00
    axp192_writeReg(0x8b, 0x00);

    // Enable over temp shutdown
    axp192_writeReg(0x8f, 0x04);

#if 1
    // Clear all IRQ status
    for (int reg = 0x44; reg <= 0x4d; reg++) {
	if (reg == 0x48) reg = 0x4d;

	uint8_t val = axp192_readReg(reg);
	if (val) {
	    printf("AXP192: REG[%02x]: %02x\n", reg, val);
	    axp192_writeReg(reg, val); // irq clear
	}
    }
#endif

#if 0
    printf("AXP192 registers\n");
    for (int reg = 0; reg <= 0xb8; reg++) {
	printf("AXP192 REG[%02x]: %02x\n", reg, axp192_readReg(reg));
    }

    printf("AXP192 interrupt enable\n");
    for (int reg = 0x40; reg <= 0x4a; reg++) {
	if (reg == 0x44) reg = 0x4a;

	printf("AXP192 REG[%02x]: %02x\n", reg, axp192_readReg(reg));

	axp192_writeReg(reg, 0x00); // disable all irq
    }

    printf("AXP192 interrupt status\n");
    for (int reg = 0x44; reg <= 0x4d; reg++) {
	uint8_t val;
	if (reg == 0x48) reg = 0x4d;

	val = axp192_readReg(reg);
	printf("AXP192 REG[%02x]: %02x\n", reg, axp192_readReg(reg));
	if (val) axp192_writeReg(reg, val); // irq clear
    }

    // disable timer
    axp192_writeReg(0x8a, 0x80);
#endif
}

void axp192_backlight(uint8_t on)
{
    uint8_t val = axp192_readReg(0x12);
    
    if (on) {
	val |= 0x04; // BL on
    } else {
	val &= ~0x04; // BL off
    }

    axp192_writeReg(0x12, val);
}

void axp192_brightness(uint8_t b)
{
    if (b > 12) b = 12;

    uint8_t v = axp192_readReg(0x28);
    axp192_writeReg(0x28, (v & 0x0f) | (b << 4));
}

int axp192_getBatVoltage(void)
{
    uint16_t val;

    val = axp192_readRegs(0x78, 2);
    return (val >> 4) | (val & 0x0f);
}

int axp192_getBatCurrentIn(void)
{
    uint16_t val;

    val = axp192_readRegs(0x7a, 2);
    return ((val >> 3) & ~0x1f) | (val & 0x1f);
}

int axp192_getBatCurrentOut(void)
{
    uint16_t val;

    val = axp192_readRegs(0x7c, 2);
    return ((val >> 3) & ~0x1f) | (val & 0x1f);
}

int axp192_getBatPower(void)
{
    uint32_t val;

    val = axp192_readRegs(0x70, 3);
    return val;
}

void axp192_shutdown(void)
{
    axp192_writeReg(0x31, axp192_readReg(0x31) | 0x08); // power key short press enable under sleep
    axp192_writeReg(0x32, axp192_readReg(0x32) | 0x83); // AXP192 power off
}

void axp192_sleep(void)
{
    axp192_writeReg(0x31, axp192_readReg(0x31) | 0x08); // enter sleep mode
    axp192_writeReg(0x12, 0x01); // power off EXTEN, LDO3, LDO2
}

void update_axp192_info(void)
{
    uint32_t x;
    struct timeval tv;

    // get time
    gettimeofday(&tv, NULL);
    axp192.time = tv.tv_sec;

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

    if (diff > 32767 || diff < -32768) { // Coulomb counter value is invalid

	axp192_writeReg(0xb8, 0x20); // counter clear;
	axp192_writeReg(0xb8, 0x80); // counter start;

	diff = 0; // now, coulomb_diff is zero
    }

    // check magic number
    if (axp192.magic != MAGIC_BATTERY) {

	// initialize Coulomb variables
	axp192_writeReg(0x06, MAGIC_BATTERY); // set magic number

    	// update coulomb_diff variable in axp192
	axp192_writeReg(0x07, diff >> 8);
	axp192_writeReg(0x08, diff & 0xff);

	axp192.c_diff = diff;

    } else {

	if (diff < axp192.c_diff) {

	    // update coulomb_diff variable in axp192
	    axp192_writeReg(0x07, diff >> 8);
	    axp192_writeReg(0x08, diff & 0xff);

	}
    }

#define ADC_SAMPLING_RATE	200 // Hz

    // Battery capacity estimation
    // C = 65536 * 0.5mA / 3600s / 200Hz = 256/5625
    x = axp192.c_in_batt - axp192.c_out_batt - axp192.c_diff;

    if ((int32_t)x < 0) x = 0;
    int capacity = x;

    axp192.batt_capacity = x * 256 * 10 / 5625; // 0.1 mAh / LSB

    if (axp192.discharge_time) { // in battery operation
	int c_raw = axp192.c_out_batt - axp192.c_out_batt_origin;

	x = c_raw * 32768; // 65536 * 0.5 mA

	uint32_t t = (xTaskGetTickCount() - axp192.discharge_time) * ADC_SAMPLING_RATE * portTICK_PERIOD_MS / 1000; // 200Hz

	axp192.batt_time = t / ADC_SAMPLING_RATE; // sec

	if (c_raw == 0) { // to avoid division by zero
	    axp192.ai_batt = 0; // 0.1 mA / LSB
	    axp192.time_left = 0; // sec
	} else {
	    axp192.ai_batt = x * 10 / t; // 0.1 mA / LSB
	    axp192.time_left = capacity * t / (c_raw * ADC_SAMPLING_RATE); // sec
	}
    }
}

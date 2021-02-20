#pragma once
/*
 * support routines for axp192
 */

#define MAGIC_BATTERY	192

struct AXP192 {
    uint8_t lcd_timer;	// LCD off timer
    uint8_t lcd_timer_default;	// LCD off timer default value
    time_t time;	// timestamp, UNIX time
    uint8_t power_status; // input power status
    uint8_t charge_status; // battery charging status
    uint8_t usb_otg_status; // USB OTG VBUS status
    uint8_t magic;	// Magic number for Coulomb difference value validity
    int16_t c_diff;	// Coulomb value for battery capacity calculation
    uint16_t v_acin;	// ACIN voltage, FP3.13 V
    uint16_t i_acin;	// ACIN current, FP13.3 mA
    uint16_t v_vbus;	// VBUS voltage, FP3.13 V
    uint16_t i_vbus;	// VBUS current, FP13.3 mA
    uint16_t t_axp192;	// axp192 temp, 0.1 C
#if 0
    uint16_t t_batt;	// Battery temp, 12bit
    uint16_t v_gpio0;	// GPIO0 voltage, 12bit
    uint16_t v_gpio1;	// GPIO1 voltage, 12bit
    uint16_t v_gpio2;	// GPIO2 voltage, 12bit
    uint16_t v_gpio3;	// GPIO3 voltage, 12bit
#endif
    uint16_t p_batt;	// Battery power, FP13.3 mW
    uint16_t v_batt;	// Battery voltage, FP3.13 V
    uint16_t i_in_batt; // Battery charge current, FP13.3 mA
    uint16_t i_out_batt; // Battery discharge current, FP13.3 mA
    uint16_t v_aps;	// APS voltage, FP3.13 V
    uint32_t c_in_batt; // Battery charge Coulomb, 32bit
    uint32_t c_out_batt; // Battery discharge Coulomb, 32bit
    uint32_t c_out_batt_origin; // Battery discharge Coulomb at starting of discharge
    uint16_t batt_capacity; // estimated battery capacity, 0.1 mAh / LSB
    uint16_t ai_batt;	// battery average current, 0.1 mA
    uint16_t time_left; // activity limit in battery operation
    uint32_t discharge_time; // Discharge start time in FreeRTOS ticks
    uint16_t batt_time; // battery operation time
};

extern struct AXP192 axp192;

void axp192_init(void);
void axp192_brightness(uint8_t b);
int axp192_getBatVoltage(void);
int axp192_getBatCurrentIn(void);
int axp192_getBatCurrentOut(void);
int axp192_getBatPower(void);
uint8_t axp192_readReg(uint8_t reg);
uint32_t axp192_readRegs(uint8_t reg, int len);
void axp192_writeReg(uint8_t reg, uint8_t val);
void axp192_backlight(uint8_t on);
void axp192_shutdown(void);
void axp192_sleep(void);
void update_axp192_info(void);

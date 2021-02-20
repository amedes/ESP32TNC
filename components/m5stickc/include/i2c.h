#pragma once
/*
 * I2C for M5StickC Plus
 */
#include <stdint.h>

void i2c_write_1byte(uint8_t i2c_addr, uint8_t addr, uint8_t data);
uint8_t i2c_read_1byte(uint8_t i2c_addr, uint8_t addr);
void i2c_read_bytes(uint8_t i2c_addr, uint8_t addr, uint8_t *data, int len);
void i2c_init(void);

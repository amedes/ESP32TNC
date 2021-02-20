#include <stdio.h>
#include "i2c.h"

#define MPU6886_ADDR 0x68

void mpu6886_init(void)
{
    i2c_init();

    // int pin configuration
    i2c_write_1byte(MPU6886_ADDR, 0x37, 0xfc); // active low, open drain
}

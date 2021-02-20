/*
 * I2C control routine for M5StickC Plus
 */
#include "driver/i2c.h"

#include "m5stickc.h"

#define M5_I2C_NUM I2C_NUM_0
#define ACK_CHECK_EN 1
#define ACK_CHECK_DIS 0
#define ACK_VAL 0
#define NACK_VAL 1

void i2c_write_1byte(uint8_t i2c_addr, uint8_t addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(M5_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
}

uint8_t i2c_read_1byte(uint8_t i2c_addr, uint8_t addr)
{
    uint8_t data;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

    i2c_master_read_byte(cmd, &data, NACK_VAL);
    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(M5_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    return data;
}

void i2c_read_bytes(uint8_t i2c_addr, uint8_t addr, uint8_t *data, int len)
{
    //uint8_t data;

    if (len <= 0) return;
    if (data == NULL) return;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);

    if (len > 1) {
	i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }

    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    ESP_ERROR_CHECK(i2c_master_cmd_begin(M5_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    //return data;
}

void i2c_init(void)
{
    static const i2c_config_t conf = {
	.mode = I2C_MODE_MASTER,
	.sda_io_num = M5_SYS_SDA,
	.scl_io_num = M5_SYS_SCL,
	.master.clk_speed = M5_CLK_SPEED,
    };
    static uint8_t initialized = 0;


    if (initialized) return;
    
    ESP_ERROR_CHECK(i2c_param_config(M5_I2C_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(M5_I2C_NUM, conf.mode, 0, 0, 0));
    initialized = 1;

    //printf("i2c_init()\n");
}

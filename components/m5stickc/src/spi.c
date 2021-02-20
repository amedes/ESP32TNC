#include <strings.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define TRANSACTION_N 6

static int dc_pin;

static spi_transaction_t trans[TRANSACTION_N];
static int trans_ptr = 0;
static int trans_num = TRANSACTION_N;

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
IRAM_ATTR static void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(dc_pin, dc);
}

void spi_init(spi_device_handle_t *spi, int spi_num, int mosi, int sclk, int spics, int spidc)
{
    spi_bus_config_t buscfg={
        .miso_io_num=-1,
        .mosi_io_num=mosi,
        .sclk_io_num=sclk,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz= 0, // default size (4092)
	.intr_flags = ESP_INTR_FLAG_IRAM,
    };
    spi_device_interface_config_t devcfg={
#ifdef CONFIG_LCD_OVERCLOCK
        .clock_speed_hz=26*1000*1000,           //Clock out at 26 MHz
#else
        .clock_speed_hz=10*1000*1000,           //Clock out at 10 MHz
#endif
        .mode=0,                                //SPI mode 0
        .spics_io_num=spics,               //CS pin
        .queue_size=TRANSACTION_N,                          //We want to be able to queue 7 transactions at a time
        .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

    //Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(spi_num, &buscfg, 1));

    //Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(spi_num, &devcfg, spi));

    // initialize the gpio connected to LCD command/data pin
    ESP_ERROR_CHECK(gpio_set_direction(spidc, GPIO_MODE_OUTPUT));

    dc_pin = spidc;
}

// return the pointer of unused transaction structure
spi_transaction_t *spi_get_trans(spi_device_handle_t spi)
{
    spi_transaction_t *tp = &trans[trans_ptr++];

    if (trans_ptr >= TRANSACTION_N) trans_ptr = 0;

    if (trans_num > 0) {
	trans_num--;
    } else {
	spi_transaction_t *tpp;
	
	// wait for execution of queued transaction
	spi_device_get_trans_result(spi, &tpp, portMAX_DELAY);
    }

    bzero(tp, sizeof(*tp));
    // default value
    tp->length = 8;
    tp->flags = SPI_TRANS_USE_TXDATA;

    return tp;
}

// wait for all queued transactions 
void spi_wait_transaction(spi_device_handle_t spi)
{
    while (trans_num < TRANSACTION_N) {
	spi_transaction_t *tpp;

	spi_device_get_trans_result(spi, &tpp, portMAX_DELAY);
	trans_num++;
    }
}

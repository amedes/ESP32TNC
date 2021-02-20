#pragma once
#include "driver/spi_master.h"

void spi_init(spi_device_handle_t *spi, int spi_num, int miso, int sclk, int spics, int spidc);
spi_transaction_t *spi_get_trans(spi_device_handle_t spi);
void spi_wait_transaction(spi_device_handle_t spi);

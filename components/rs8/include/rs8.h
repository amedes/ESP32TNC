#pragma once

#include <stdint.h>

#include "gf8.h"
#include "poly8.h"

#define RS8_OK (0)
#define RS8_ERR (-1)

#define RS8_PARITY_MIN 2
#define RS8_PARITY_MAX 64

int rs8_encode(uint8_t data[], int data_len, uint8_t parity[], int parity_len);
int rs8_decode(uint8_t code[], int code_len, int parity_len);

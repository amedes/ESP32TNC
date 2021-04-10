// fx25_decode.h

#pragma once

#include <stdint.h>
#include "tnc.h"

enum FX25_STATE {
    FX25_FINDTAG  = 0,
    FX25_DATA,
};

void fx25_decode_bit(tcb_t *tp, uint8_t bit);

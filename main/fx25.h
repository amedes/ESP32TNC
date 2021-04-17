#pragma once

#include <stdint.h>
#include "tnc.h"

#define FX25_CODE_MAX   255
#define FX25_PARITY_MIN 16
#define FX25_DATA_LEN_MAX   (FX25_CODE_MAX - FX25_PARITY_MIN)

// struct for FX.25 tag info.
typedef struct FX25TAG {
    uint64_t tagval;
    int rs_code;
    int rs_info;
} fx25tag_t;

// variable for tag info.
extern const fx25tag_t fx25tag[];

// tag number
enum FX25TAG_NO {
    TAG_00 = 0,
    TAG_01,
    TAG_02,
    TAG_03,
    TAG_04,
    TAG_05,
    TAG_06,
    TAG_07,
    TAG_08,
    TAG_09,
    TAG_0A,
    TAG_0B,
    TAG_0C,
    TAG_0D,
    TAG_0E,
    TAG_0F,
};

typedef struct TCB tcb_t;

int fx25_send_packet(tcb_t *tp, void *data, size_t data_len, int parity);
//int fx25_send_packet(tcb_t *tp, uint8_t *data[2], size_t data_len[2], int parity);

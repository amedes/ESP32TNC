#pragma once

#include <stdint.h>

typedef struct FIR_FILTER {
    int16_t *an;
    int16_t *x;
    int size;
    int index;
} filter_t;

typedef struct FILTER_PARAM {
    int size;
    int sampling_freq;
    int pass_freq;
    int cutoff_freq;
} filter_param_t;

int filter(filter_t *fp, int16_t value);
void filter_init(filter_t *fp, int16_t an[], int size);
int16_t *filter_coeff(filter_param_t const *f);

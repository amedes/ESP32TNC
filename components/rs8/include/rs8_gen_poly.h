/*
 * generator polynomial table for Reed-Solomon fast encoder
 */
#pragma once

#include "gf8.h"

extern const gf8_t rs8_gen_poly[];

const gf8_t *rs8_gen_poly_table(int parity);

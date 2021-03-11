#pragma once

#include <stdio.h>
#include <assert.h>

#define GF8_ELEMENTS 256
#define GF8_MOD_MAX (GF8_ELEMENTS - 1)

#define GF8_INDEX_MAX GF8_ELEMENTS
#define GF8_VECTOR_MAX (GF8_ELEMENTS * 2)

#define GF8_A0 255 // special value "zero" for index form

typedef uint8_t gf8_t;

extern const gf8_t gf8_index[GF8_INDEX_MAX];
extern const gf8_t gf8_vector[GF8_VECTOR_MAX];

static inline gf8_t gf8_add(gf8_t x, gf8_t y)
{
    return x ^ y;
}

static inline gf8_t gf8_sub(gf8_t x, gf8_t y)
{
    return x ^ y;
}

static inline gf8_t gf8_mul(gf8_t x, gf8_t y)
{
    if (!x || !y) return 0;

    return gf8_vector[(int)gf8_index[x] + (int)gf8_index[y]];
}

static inline gf8_t gf8_div(gf8_t x, gf8_t y)
{
    assert(y != 0); // division by zero error

    return gf8_vector[(int)gf8_index[x] + GF8_MOD_MAX - (int)gf8_index[y]];
}

static inline gf8_t gf8_recip(gf8_t x)
{
    assert(x != 0); // division by zero error

    return gf8_vector[GF8_MOD_MAX - (int)gf8_index[x]];
}

static inline gf8_t gf8_alpha(int x)
{
    int r = x % GF8_MOD_MAX;

    if (r < 0) r += GF8_MOD_MAX;

    return gf8_vector[r];
}

static inline gf8_t gf8_power(gf8_t x, int n)
{
    if (x == 0) return 0;

    return gf8_vector[(gf8_index[x] * n) % GF8_MOD_MAX];
}

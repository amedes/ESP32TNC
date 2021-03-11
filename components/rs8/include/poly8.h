#pragma once
/*
 * polynomial manupilation routines
 */
#include <stdio.h>

#include "gf8.h"

#define POLY8_OK 0
#define POLY8_ERR (-1)

typedef union {
    struct {
	uint8_t items;		// number of coefficient
	uint8_t coeff[255];	// value of coefficient
    };
    uint8_t item[256];
} poly8_t;

static inline int poly8_degree(poly8_t *p)
{
    return p->items - 1;
}

void poly8_copy(poly8_t *dst, poly8_t *src);
int poly8_add(poly8_t *x, poly8_t *y, poly8_t *r);
int poly8_mul(poly8_t *p1, poly8_t *p2, poly8_t *result);
int poly8_div(poly8_t *dividend, poly8_t *divisor, poly8_t *quotient, poly8_t *remainder);
gf8_t poly8_subst(poly8_t *poly, gf8_t x);
int poly8_euclid(poly8_t *x, poly8_t *y, poly8_t *a, poly8_t *b, poly8_t *c, int t);
int poly8_iszero(poly8_t *p);
void poly8_clear(poly8_t *p);
void poly8_normalize(poly8_t *p);
void poly8_print(const char s[], poly8_t *p);
int poly8_diff(poly8_t *f, poly8_t *df);

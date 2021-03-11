/*
 * polynomial routines for GF(2^8)
 */
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "poly8.h"
#include "gf8.h"

#if 0
static uint8_t work1[256] = { 255 };
static uint8_t work2[256] = { 255 };

static poly8_t *w1 = (poly8_t *)work1;
static poly8_t *w2 = (poly8_t *)work2;
#endif

//#define CYCLE 1

int poly8_mul(poly8_t *p1, poly8_t *p2, poly8_t *r)
{
#ifdef CYCLE
    int cc = 0;
#endif

#if 0
    printf("p1(%d), p2(%d), r(%d)\n", p1->items, p2->items, r->items);
    poly8_print("p1", p1);
    poly8_print("p2", p2);
#endif

    if (poly8_degree(p1) + poly8_degree(p2) > poly8_degree(r)) {
	return POLY8_ERR; // result is too small to store the product
    }

    // zero clear the result
    memset(r->coeff, 0, r->items);
 
    for (int i = 0; i < p2->items; i++) {
	gf8_t vec2 = p2->coeff[i];

	if (vec2 == 0) continue; // skip if the coefficient is "0"

	gf8_t ind2 = gf8_index[vec2]; // index of the coefficient

	const gf8_t *vector = &gf8_vector[ind2]; // offset of the vector table

#ifdef __XTENSA__

	uint32_t reg0, reg1, reg2;
	uint32_t coeffp = (int)p1->coeff;
	uint32_t resultp = (int)&r->coeff[i] - 1;
#ifdef CYCLE
	uint32_t cc0, cc1;
#endif

	asm volatile(
#ifdef CYCLE
		"rsr.ccount	%[cc0]\n\t"
#endif
		"loopgtz	%[loop], %=f\n\t"

		"l8ui		%[reg0], %[coeffp], 0\n\t"	// a = *coeff, index form
		"l8ui		%[reg1], %[resultp], 1\n\t"	// sum = result[1]

		"addi		%[coeffp], %[coeffp], 1\n\t"	// coeff++

		"add		%[reg2], %[index], %[reg0]\n\t"	// convert to index form
		"l8ui		%[reg2], %[reg2], 0\n\t"

		"addi		%[resultp], %[resultp], 1\n\t"	// result++

		"add		%[reg2], %[vec], %[reg2]\n\t"	// c = a * b
		"l8ui		%[reg2], %[reg2], 0\n\t"	// convert to vector form

		"moveqz		%[reg2], %[reg0], %[reg0]\n\t"	// c = 0 if a == 0

		"xor		%[reg1], %[reg1], %[reg2]\n\t"	// sum = c + sum
		"s8i		%[reg1], %[resultp], 0\n\t"	// *result = sum

		"%=:\n\t"
#ifdef CYCLE
		"rsr.ccount	%[cc1]\n\t"
#endif
		:
#ifdef CYCLE
		[cc0] "=&r" (cc0),
	        [cc1] "=&r" (cc1),
#endif
		[reg0] "=&r" (reg0),
		[reg1] "=&r" (reg1),
		[reg2] "=&r" (reg2),
		[coeffp] "+r" (coeffp),
		[resultp] "+r" (resultp)
		:
		[loop] "r" (p1->items),
		[vec] "r" (vector),
		[index] "r" (gf8_index)
		);
#ifdef CYCLE
	cc = cc1 - cc0 - 1;
#endif

#else // __XTENSA__

	for (int j = 0; j < w->items; j++) {
	    gf8_t ind1 = w->coeff[j]; // index form

	    if (ind1 == GF8_A0) continue; // if p1->coeff[j] is zero

	    r->coeff[i + j] ^= vector[ind1];
	}

#endif // __XTENSA__

    }

#ifdef CYCLE
    printf("poly8_mul(): %d cycle\n", cc);
#endif

    return POLY8_OK;
}

/*
 * polynomial division
 */
int poly8_div(
	poly8_t *dividend,
	poly8_t *divisor,
	poly8_t *quotient,
	poly8_t *remainder
	)
{
#ifdef CYCLE
    int cc = 0;
#endif

    if (poly8_degree(quotient) < poly8_degree(dividend) - poly8_degree(divisor)) {
	// quotient is too small
	return POLY8_ERR;
    }

    if (poly8_degree(remainder) < poly8_degree(divisor) - 1) {
	// remainder is too small
	return POLY8_ERR;
    }

    if (dividend->items == 0 || divisor->items == 0) {
	return POLY8_ERR;
    }

    poly8_t *work = malloc(divisor->items + 1);
    assert(work != NULL);

    // convert the divisor from vector form to index form
    work->items = divisor->items;
    for (int i = 0; i < work->items; i++) {
	work->coeff[i] = gf8_index[divisor->coeff[i]];
    }

    //int divd_dgree = poly8_degree(dividend);
    int top = poly8_degree(dividend);

    // most significant coefficient
    gf8_t msc = dividend->coeff[top--];

    int divs_degree = poly8_degree(divisor);

    while (divs_degree >= 0) {
	if (divisor->coeff[divs_degree]) break;
	divs_degree--;
    }

    int rem_degree = poly8_degree(divisor) - 1;

    int copy_len = rem_degree + 1;

    poly8_t *rem = remainder;

    // copy dividend to work area (remainder)
    for (int i = 0; i < copy_len; i++) {
	if (top - i >= 0) {
	    rem->coeff[rem_degree - i] = dividend->coeff[top - i];
	} else {
	    rem->coeff[rem_degree - i] = 0;
	}
    }

#if 0
    printf("work(%d) = \n", w->items);
    for (int i = 0; i < w->items; i++) {
	printf("%02x, ", w->coeff[i]);
	if (i % 16 == 15) printf("\n");
    }
    printf("\nmsc = %02x\n", msc);
#endif

    // most significant coefficient of the divisor polynomial
    int divs_top = divisor->coeff[divs_degree];

    for (int i = top; i + 1 - divs_degree >= 0; i--) {
	gf8_t lsc;

	if (i - divs_degree >= 0) {
	    lsc = dividend->coeff[i - divs_degree];
	} else {
	    lsc = 0;
	}

	//printf("%02x, ", lsc);
#if 0	
	printf("msc = %02x, lsc = %02x, work[%i] = \n", msc, lsc, i);
	for (int j = 0; j < w->items; j++) {
	    printf("%02x, ", w->coeff[j]);
	    //if (j % 16 == 15) printf("\n");
	}
	printf("\n");
#endif
	if (msc != 0) {
	    int quo = gf8_div(msc, divs_top); // calculate one digit of quotient

	    quotient->coeff[i - divs_degree + 1] = quo;
	    //printf("quo[%d] = %02x (%02x, %02x), ", i - divs_degree + 1, quo, msc, divs_top);
#if 1
	    gf8_t ind = gf8_index[quo]; // index of quotinent
	    const gf8_t *vector = &gf8_vector[ind]; // offset the vector table

#ifdef __XTENSA__

	    uint32_t reg0, reg1, reg2;
	    uint32_t divsp = (int)work->coeff;
	    uint32_t remp = (int)rem->coeff - 1;
#ifdef CYCLE
	    uint32_t cc0, cc1;
#endif

	    asm volatile(
#ifdef CYCLE
		    "rsr.ccount	%[cc0]\n\t"
#endif
		    "loopgtz	%[loop], %=0f\n\t"

		    "l8ui	%[reg0], %[divsp], 0\n\t"	// divisor, index form
		    "l8ui	%[reg1], %[remp], 1\n\t"	// remainder

		    "movi	%[reg2], 0\n\t"
		    "beq	%[reg0], %[A0], %=1f\n\t"	// if divisor is zero 

		    "add	%[reg0], %[vec], %[reg0]\n\t"	// quotinent * divisor
		    "l8ui	%[reg2], %[reg0], 0\n\t"	// vector form

		    "%=1:\n\t"

		    "addi	%[divsp], %[divsp], 1\n\t"	// advance next divisor
		    "addi	%[remp], %[remp], 1\n\t"	// advance next remainder

		    "xor	%[reg1], %[reg1], %[reg2]\n\t"	// "subtract" dividend by divisor
		    "s8i	%[lsc], %[remp], 0\n\t"		// store previous remainder

		    "mov	%[lsc], %[reg1]\n\t"		// shift one digit

		    "%=0:\n\t"
#ifdef CYCLE
		    "rsr.ccount	%[cc1]\n\t"
#endif
		    :
#ifdef CYCLE
		    [cc0] "=&r" (cc0),
		    [cc1] "=&r" (cc1),
#endif
		    [reg0] "=&r" (reg0),
		    [reg1] "=&r" (reg1),
		    [reg2] "=&r" (reg2),
		    [divsp] "+r" (divsp),
		    [remp] "+r" (remp),
		    [lsc] "+r" (lsc)
		    :
		    [loop] "r" (divs_degree),
		    [vec] "r" (vector),
		    [A0] "r" (GF8_A0)
		    );
#ifdef CYCLE
	    cc = cc1 - cc0 - 1;
#endif

#else // !__XTENSA__

	    for (int j = 0; j < divs_degree; j++) {
		gf8_t ind1 = work->coeff[j]; // index form of divisor
		gf8_t vec1 = (ind1 == GF8_A0) ? 0 : vector[ind1];

		int t = rem->coeff[j] ^ vec1; // poly - quo * divs
		rem->coeff[j] = lsc;
		lsc = t;
	    }

#endif // __XTENSA__

	    msc = lsc;

#else
	    for (int j = 0; j < divs_degree; j++) {
		int t = rem->coeff[j] ^ gf8_mul(quo, divisor->coeff[j]); // poly - quo * divs
		rem->coeff[j] = lsc;
		lsc = t;
	    }
	    msc = lsc;
#endif

	} else { // msc == 0

	    //printf("i = %d, msc = %02x, lsc = %02x\n", i, msc, lsc);

	    quotient->coeff[i - divs_degree + 1] = 0; // quotient is zero
	    
	    // shift one digit
	    msc = rem->coeff[rem_degree];
	    memmove(&rem->coeff[1], &rem->coeff[0], rem_degree);
	    rem->coeff[0] = lsc;

	}
    }

    free(work);

    //printf("\n");

    // adjust remainder position
    memmove(&rem->coeff[0], &rem->coeff[1], rem_degree);
    rem->coeff[rem_degree] = msc;

#ifdef CYCLE
    printf("poly8_div(): %d cycle\n", cc);
#endif

    return POLY8_OK;
}

/*
 * calculate the value of polynomial
 */
gf8_t poly8_subst(poly8_t *p, gf8_t x)
{
    if (p->items == 0) return 0;
    if (x == 0 || p->items == 1) return p->coeff[0];

    gf8_t sum; // value of p(x)

#ifdef __XTENSA__

    gf8_t indx = gf8_index[x];
    const gf8_t *vec = &gf8_vector[indx];

    uint32_t reg, c, tmp;
    uint32_t coeffp = (int)&p->coeff[p->items - 2];

    sum = p->coeff[p->items - 1];

    asm volatile(
	    "loopgtz	%[loop], %=f\n\t"

	    "add	%[reg], %[index], %[t]\n\t"
	    "l8ui	%[reg], %[reg], 0\n\t"

	    "add	%[reg], %[vec], %[reg]\n\t"
	    "l8ui	%[tmp], %[reg], 0\n\t"

	    "l8ui	%[c], %[coeffp], 0\n\t"
	    "addi	%[coeffp], %[coeffp], -1\n\t"

	    "movnez	%[t], %[tmp], %[t]\n\t"

	    "xor	%[t], %[t], %[c]\n\t"

	    "%=:\n\t"
	    :
	    [reg] "=&r" (reg),
	    [c] "=&r" (c),
	    [t] "+r" (sum),
	    [coeffp] "+r" (coeffp),
	    [tmp] "=&r" (tmp)
	    :
	    [loop] "r" (p->items - 1),
	    [vec] "r" (vec),
	    [index] "r" (gf8_index)
	    );

#else // !__XTENSA__

    sum = p->coeff[p->items - 1];
    for (int i = p->items - 2; i >= 0; i--) {
	sum = gf8_add(gf8_mul(sum, x), p->coeff[i]);
    }
    
#endif // __XTENSA__

    return sum;
}

int poly8_add(poly8_t *x, poly8_t *y, poly8_t *r)
{
    poly8_t *p1, *p2;

    if (x->items > y->items) {
	p1 = x;
	p2 = y;
    } else {
	p1 = y;
	p2 = x;
    }

#ifdef __XTENSA__

    uint32_t xx, yy;
    uint8_t *xp = p1->coeff;
    uint8_t *yp = p2->coeff;
    uint8_t *rp = r->coeff;

    asm volatile(
	    "loopgtz	%[loop], %=0f\n\t"

	    "l8ui	%[x], %[xp], 0\n\t"
	    "l8ui	%[y], %[yp], 0\n\t"
	    "addi	%[xp], %[xp], 1\n\t"

	    "bge	%[yp], %[ypmax], %=1f\n\t"

	    "addi	%[yp], %[yp], 1\n\t"

	    "xor	%[x], %[x], %[y]\n\t"

	    "%=1:\n\t"

	    "s8i	%[x], %[rp], 0\n\t"
	    "addi	%[rp], %[rp], 1\n\t"
	    
	    "%=0:\n\t"
	    :
	    [x] "=&r" (xx),
	    [y] "=&r" (yy),
	    [xp] "+r" (xp),
	    [yp] "+r" (yp),
	    [rp] "+r" (rp)
	    :
	    [loop] "r" (p1->items),
	    [ypmax] "r" (&p2->coeff[p2->items])
	    );

#else // __XTENSA__

    gf8_t c1, c2;

    for (int i = 0; i < p1->items; i++) {

	c1 = p1->coeff[i];
	c2 = (i < p2->items) ? p2->coeff[i] : 0;

    	r->coeff[i] = c1 ^ c2;
    }

#endif // __XTENSA__

    r->items = p1->items;

    return POLY8_OK;
}

void poly8_copy(poly8_t *dst, poly8_t *src)
{
#ifdef __XTENSA__

    uint32_t c;
    uint8_t *srcp = src->coeff;
    uint8_t *dstp = dst->coeff - 1;
    uint32_t loop = src->items;

    asm volatile(
	    "loopgtz	%[loop], %=f\n\t"

	    "l8ui	%[c], %[srcp], 0\n\t"

	    "addi	%[srcp], %[srcp], 1\n\t"
	    "addi	%[dstp], %[dstp], 1\n\t"

	    "s8i	%[c], %[dstp], 0\n\t"

	    "%=:\n\t"
	    :
	    [c] "=&r" (c),
	    [srcp] "+r" (srcp),
	    [dstp] "+r" (dstp)
	    :
	    [loop] "r" (loop)
	    );

#else // !__XTENSA__

    for (int i = 0; i < src->items; i++) {
	dst->coeff[i] = src->coeff[i];
    }

#endif // __XTENSA__

    dst->items = src->items;
}

int poly8_iszero(poly8_t *p)
{
    for (int i = 0; i < p->items; i++) {
	if (p->coeff[i]) return 0;
    }

    return 1;
}

void poly8_clear(poly8_t *p)
{
    for (int i = 0; i < p->items; i++) {
	p->coeff[i] = 0;
    }
}

void poly8_normalize(poly8_t *p)
{
    for (int i = p->items - 1; i >= 0; i--) {
	if (p->coeff[i]) {

	    p->items = i + 1;
	    break;
	}
    }
}

void poly8_print(const char s[], poly8_t *p)
{
    printf("%s(%d) =\n", s, p->items);

    for (int i = 0; i < p->items; i++) {
	printf("%02x, ", p->coeff[i]);
	if (i % 16 == 15) printf("\n");
    }
    printf("\n");
}

/*
 * extended Eucliean algorithm
 *
 * intput x, y, t; t specifies degree of remainder
 * output a, b, c; a*x + b*y = c (deg c < t)
 */
int poly8_euclid(poly8_t *x, poly8_t *y, poly8_t *a, poly8_t *b, poly8_t *c, int t)
{
    poly8_t *a0, *a1, *a2;
    poly8_t *b0, *b1, *b2;
    poly8_t *r0, *r1, *r2;
    poly8_t *q1;
    poly8_t *tmp, *tt;
    poly8_t *w;

#define POLY8_SIZE sizeof(poly8_t)
#define WORK_SIZE (POLY8_SIZE * 11)

    w = malloc(WORK_SIZE);
    assert(w != NULL);

    a0 = &w[0];
    a1 = &w[1];
    a2 = &w[2];
    b0 = &w[3];
    b1 = &w[4];
    b2 = &w[5];
    r0 = &w[6];
    r1 = &w[7];
    r2 = &w[8];
    q1 = &w[9];
    tmp = &w[10];

    // r0 = x
    //r0->items = x->items;
    poly8_copy(r0, x);

    // r1 = y
    //r1->items = y->items;
    poly8_copy(r1, y);
    poly8_normalize(r1);

    // a0 = 1
    a0->items = 1;
    a0->coeff[0] = 1;

    // a1 = 0
    a1->items = 1;
    a1->coeff[0] = 0;

    // b0 = 0
    b0->items = 1;
    b0->coeff[0] = 0;

    // b1 = 1
    b1->items = 1;
    b1->coeff[0] = 1;

    //int div_cnt = 0;
    while (poly8_degree(r1) >= t) {

    	// quotient
	q1->items = poly8_degree(r0) - poly8_degree(r1) + 1;

    	// remainder
	r2->items = poly8_degree(r1);

	if (poly8_div(r0, r1, q1, r2) != POLY8_OK) {
	    return POLY8_ERR;
	}
	//div_cnt++;

#if 0
	poly8_print("r0", r0);
	poly8_print("r1", r1);
	poly8_print("q1", q1);
	poly8_print("r2", r2);
#endif
	
	poly8_normalize(q1);
	poly8_normalize(r2);

	//printf("poly8_div(r0(%d), r1(%d), q1(%d), r2(%d))\n", r0->items, r1->items, q1->items, r2->items);

	tmp->items = q1->items - 1 + a1->items - 1 + 1;
	//printf("poly8_mul(q1(%d), a1(%d), tmp(%d))\n", q1->items, a1->items, tmp->items);

	assert(poly8_mul(q1, a1, tmp) == POLY8_OK);
	poly8_normalize(tmp);

	//a2->items = (a0->items > tmp->items) ? a0->items : tmp->items;
	//printf("poly8_add(a0(%d), tmp(%d), a2(%d))\n", a0->items, tmp->items, a2->items);

	poly8_add(a0, tmp, a2);
	//poly8_normalize(a2);

	tmp->items = q1->items - 1 + b1->items - 1 + 1;
	//printf("poly8_mul(q1(%d), b1(%d), tmp(%d))\n", q1->items, a1->items, tmp->items);

	assert(poly8_mul(q1, b1, tmp) == POLY8_OK);
	poly8_normalize(tmp);

	//b2->items = (b0->items > tmp->items) ? b0->items : tmp->items;
	//printf("poly8_add(b0(%d), tmp(%d), b2(%d))\n", b0->items, tmp->items, b2->items);

	poly8_add(b0, tmp, b2);
	//poly8_normalize(b2);

	// r0 <- r1 <- r2
	tt = r0;
	r0 = r1;
	r1 = r2;
	r2 = tt;

	// a0 <- a1 <- a2
	tt = a0;
	a0 = a1;
	a1 = a2;
	a2 = tt;

	// b0 <- b1 <- b2
	tt = b0;
	b0 = b1;
	b1 = b2;
	b2 = tt;
    }
    //printf("rs8_euclid(): division count = %d\n", div_cnt);

    if (c) {
	//c->items = r1->items;
	poly8_copy(c, r1); // c = r1
    }

    if (a) {
	//a->items = b1->items;
	poly8_copy(a, b1); // a = b1
    }

    if (b) {
	//b->items = a1->items;
	poly8_copy(b, a1); // b = a1
    }

    free(w);

    return POLY8_OK;
}

/*
 * differentiating the polynomial
 */
int poly8_diff(poly8_t *f, poly8_t *df)
{
    if (df->items < f->items - 1) { // df is too small
	return POLY8_ERR;
    }

    for (int i = 1; i < f->items; i++) {
	int j = i;
	gf8_t c = 0;
	gf8_t a = f->coeff[i];

	while (j-- > 0) {
	    c = gf8_add(c, a);
	}
	//df->coeff[i - 1] = (i & 1) ? f->coeff[i] : 0;
	df->coeff[i - 1] = c;
    }

    return POLY8_OK;
}

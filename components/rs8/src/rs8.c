/*
 * Reed Solomon fast encode/decode routine for FX.25
 *
 * GF(2^8): primitive polynomial, 0x11d
 * generator polynomial: G(x) = (x - alpha^1) * (x - alpha^2) * ... * (x - alpha^parity)
 * number of parity: 2..64 (even number only)
 *
 */
#include <stdio.h>
#include <string.h>

#include "gf8.h"
#include "poly8.h"
#include "rs8.h"
#include "rs8_gen_poly.h"

#define GEN_POLY_1	1	// minimum root of the generator polynomial is alpha^1
//#define DEBUG 1

/*
 * Reed-Solomon fast encoder
 *
 * input: data[0..info_len-1]: input data
 * input: info_len: length of data
 * output: parity[0..parity_len-1]: pointer to store the parity
 * input: parity_len: length of parity (only support 16, 32, 64 parities)
 */
int rs8_encode(uint8_t data[], int info_len, uint8_t parity[], int parity_len)
{
    const gf8_t *gen_poly; // coefficient of generator polynomial
    const gf8_t *vector;
    int q, m; // GF(2^8)
    int i;
    int zero_cnt = 0;
    int top;
    uint32_t rem;
    const int dividend_len = 255 - parity_len; // length of dividend including of shortened zero

    /*
     * generator polynomial, coefficent is descendig order
     * coefficent of "x^parity" is always "1", so not included in the array
     * poly[0]: coefficent of "x^(parity - 1)", index
     *   :
     * poly[pairy - 1]: coefficent of "1", index
     */

    gen_poly = rs8_gen_poly_table(parity_len);
    if (gen_poly == NULL) return RS8_ERR;

    top = 0;

    m = data[top++]; // most significant coefficient of information polynomial, vector

    q = gf8_index[m]; // q is quotient
    vector = &gf8_vector[q]; // offset the vector table to get product with q

    // rem is next least significant symbol of dividend
    if (top + parity_len < info_len) {
	rem = data[top + parity_len];
    } else {
	rem = 0;
    }

    // do subtraction while copying, copy in reverse order
    for (int j = parity_len - 1; j >= 0; j--) {
        gf8_t t;

	if (top + j < info_len) {
	    t = data[top + j];
	} else {
	    t = 0;
	}

	if (m != 0) t ^= vector[gen_poly[j]]; // pairty - poly * q

        parity[j] = rem;
        rem = t;
    }
    m = (gf8_t)rem;
    top++;

   // do division
   for (i = top; i < dividend_len; i++) { // start of quotient at &data[top - 1]

#ifdef DEBUG
       printf("%3d: (%02x), ", i - parity_len - 1, m);
       for (int j = 0; j < parity_len; j++) printf("%02x, ", parity[j]);
       printf("\n");
#endif

       // coefficient of information polynomial to add to parity polynomial next processing
       if (i + parity_len < info_len) {
	   rem = data[i + parity_len];
       } else {
	   rem = 0;
       }

       if (m != 0) { // m is most siginificant digit of dividend

	   q = gf8_index[m]; // q is quotient 
	   vector = &gf8_vector[q]; // offset the vector table to get product with q

#ifdef __XTENSA__

	   uint32_t reg0 = parity_len / 2, reg1, pol, par0, par1;
	   uint32_t polp = (int)(gen_poly + parity_len - 2);
	   uint32_t parp = (int)(parity + parity_len);

	   asm volatile(
		   "loopgtz	%[reg0], %=f\n\t"

		   "l16ui	%[pol], %[polp], 0\n\t"
		   "addi	%[polp], %[polp], -2\n\t"
		   "addi	%[parp], %[parp], -2\n\t"

		   "extui	%[reg1], %[pol], 8, 8\n\t"
		   "add		%[reg1], %[vec], %[reg1]\n\t"
		   "l8ui	%[reg1], %[reg1], 0\n\t"

		   "extui	%[reg0], %[pol], 0, 8\n\t"
		   "add		%[reg0], %[vec], %[reg0]\n\t"
		   "l8ui	%[reg0], %[reg0], 0\n\t"

		   "l8ui	%[par1], %[parp], 1\n\t"
		   "l8ui	%[par0], %[parp], 0\n\t"
		   "s8i		%[rem], %[parp], 1\n\t"
		   "xor		%[rem], %[par1], %[reg1]\n\t"
		   "s8i		%[rem], %[parp], 0\n\t"
		   "xor		%[rem], %[par0], %[reg0]\n\t"

		   "%=:\n\t"
		   :
		   [reg0] "+r" (reg0),
		   [reg1] "=&r" (reg1),
		   [par0] "=&r" (par0),
		   [par1] "=&r" (par1),
		   [pol] "=&r" (pol),
		   [polp] "+r" (polp),
		   [parp] "+r" (parp),
		   [rem] "+r" (rem)
		   :
		   [vec] "r" (vector)
		   );

#else // __XTENSA__

	   // do subtraction
	   for (int j = parity_len - 1; j >= 0; j--) {
	       gf8_t t = parity[j] ^ vector[gen_poly[j]]; // pairty - poly * q

	       parity[j] = rem;
	       rem = t;
	   }

#endif

	   // most significant coefficient of parity polynomial
	   m = (gf8_t)rem;

	} else { // if m == 0

	    // shift one digit, no need to subtract

	    zero_cnt++;

#ifdef __XTENSA__

	   uint32_t reg0, reg1;
	   uint32_t parp = (int)(parity + parity_len - 2); // prepare for word aligned access
	   uint32_t loop = parity_len / 2;

	   asm volatile(
		   "loopgtz	%[loop], %=0f\n\t"

		   "l8ui	%[reg1], %[parp], 1\n\t"
		   "l8ui	%[reg0], %[parp], 0\n\t"

		   "s8i		%[rem], %[parp], 1\n\t"
		   "s8i		%[reg1], %[parp], 0\n\t"

		   "addi	%[parp], %[parp], -2\n\t"
		   "mov		%[rem], %[reg0]\n\t"

		   "%=0:\n\t"
		   :
		   [rem]"+r"(rem),
		   [reg0]"=r"(reg0),
		   [reg1]"=r"(reg1),
		   [parp]"+r"(parp)
		   :
		   [loop]"r"(loop)
		   );

	   m = (gf8_t)rem;

#else // __XTENNSA__
	   
	   // shift one digit
	   m = parity[0];
	   memmove(&parity[0], &parity[1], parity_len - 1); // parity * x
	   parity[parity_len - 1] = rem;

#endif // __XTENSA__

	}

    }

    // do not need to shift on last subtraction
    if (m != 0) {

	q = gf8_index[m]; // quotient is always equal to most siginificant coefficient of parity polynomial, index
	vector = &gf8_vector[q]; // offset the vector table

#ifdef __XTENSA__

	uint32_t reg0, reg1, pol, par0, par1;
	uint32_t polp = (int)(gen_poly + parity_len - 2);
	uint32_t parp = (int)(parity + parity_len - 0);

	asm volatile(
		"loopgtz	%[loop], %=f\n\t"

		"l16ui		%[pol], %[polp], 0\n\t"
		"addi		%[polp], %[polp], -2\n\t"
		"addi		%[parp], %[parp], -2\n\t"

		"l8ui		%[par1], %[parp], 1\n\t"
		"l8ui		%[par0], %[parp], 0\n\t"

		"extui		%[reg1], %[pol], 8, 8\n\t"
		"add		%[reg1], %[vec], %[reg1]\n\t"
		"l8ui		%[reg1], %[reg1], 0\n\t"

		"extui		%[reg0], %[pol], 0, 8\n\t"
		"add		%[reg0], %[vec], %[reg0]\n\t"
		"l8ui		%[reg0], %[reg0], 0\n\t"

		"xor		%[par1], %[par1], %[reg1]\n\t"
		"s8i		%[par1], %[parp], 1\n\t"

		"xor		%[par0], %[par0], %[reg0]\n\t"
		"s8i		%[par0], %[parp], 0\n\t"

		"%=:\n\t"
		:
		[reg0] "=&r" (reg0),
		[reg1] "=&r" (reg1),
		[pol] "=&r" (pol),
		[par0] "=&r" (par0),
		[par1] "=&r" (par1),
		[polp] "+r" (polp),
		[parp] "+r" (parp)
		:
		[vec] "r" (vector),
		[loop] "r" (parity_len / 2)
		);

#else // __XTENSA__
	   
        // do not need to shift on last subtraction
	 for (int j = 0; j < parity_len; j++) {
	     parity[j] ^= vector[gen_poly[j]]; // pairty - poly * q
	 }

#endif // __XTENSA__

     }

    return RS8_OK;
}

/*
 * calculate syndrome (using Horner's method)
 */
static int rs8_syndrome(uint8_t data[], int code_len, uint8_t syndrome[], int parity_len)
{
#if 1
    if (parity_len < RS8_PARITY_MIN || parity_len > RS8_PARITY_MAX || (parity_len & 1) != 0) {
	// unsupported parity length
	return RS8_ERR;
    }
#endif

    if (code_len <= 0 || code_len > 255) {
	// wrong length of code
	return RS8_ERR;
    }

    int shortened_len = 255 - code_len;
    int info_len = code_len - parity_len;

    if (info_len <= 0) {
	return RS8_ERR;
    }

    //printf("rs8_syndrome(): shortened_len = %d, info_len = %d, parity_len = %d\n", shortened_len, info_len, parity_len);

    for (int i = 0; i < parity_len; i++) {
	gf8_t t;

#ifdef GEN_POLY_1
	gf8_t x = i + 1; // value of x: alpha^1, ..., alpha^parity_len
#else
	gf8_t x = i; // value of x: alpha^0, ..., alpha^(parity_len-1)
#endif	
	
	// calculate value of the code polynomial, S(alpha^(i+1))

	if (shortened_len > 0) {

#ifdef __XTENSA__

	gf8_t indx = x;
	const gf8_t *vec = &gf8_vector[indx]; // vector table for multiplying by x
	uint32_t reg, c, tmp;
	uint8_t *coeffp = &data[1];
	const gf8_t *vec_xs = &gf8_vector[(x * shortened_len) % GF8_MOD_MAX]; // vector table for multiplying by x^shortened_len

	t = data[0];

    asm volatile(
	    // information part
	    "loopgtz	%[loop0], %=0f\n\t"

	    "add	%[reg], %[index], %[t]\n\t"	// convert t to index form
	    "l8ui	%[reg], %[reg], 0\n\t"

	    "add	%[reg], %[vec], %[reg]\n\t"	// tmp = t * x
	    "l8ui	%[tmp], %[reg], 0\n\t"

	    "l8ui	%[c], %[coeffp], 0\n\t"		// load the coefficient
	    "addi	%[coeffp], %[coeffp], 1\n\t"	// advance the pointer

	    "movnez	%[t], %[tmp], %[t]\n\t"		// t = tmp if t != 0 else t = 0

	    "xor	%[t], %[t], %[c]\n\t"		// t = t + c

	    "%=0:\n\t"

	    // shortened part
	    "beqz	%[t], %=2f\n\t"			// skip if t == 0

	    "add	%[reg], %[index], %[t]\n\t"	// convert t to index form
	    "l8ui	%[reg], %[reg], 0\n\t"

	    "add	%[reg], %[vec_xs], %[reg]\n\t"	// t = t * x^shortened_len
	    "l8ui	%[tmp], %[reg], 0\n\t"

	    "movnez	%[t], %[tmp], %[t]\n\t"		// clear t if t == 0

	    "%=2:\n\t"

	    // parity part
	    "loopgtz	%[loop1], %=1f\n\t"

	    "add	%[reg], %[index], %[t]\n\t"	// convert t to index form
	    "l8ui	%[reg], %[reg], 0\n\t"

	    "add	%[reg], %[vec], %[reg]\n\t"	// tmp = t * x
	    "l8ui	%[tmp], %[reg], 0\n\t"

	    "l8ui	%[c], %[coeffp], 0\n\t"		// load the coefficient
	    "addi	%[coeffp], %[coeffp], 1\n\t"	// advance the pointer

	    "movnez	%[t], %[tmp], %[t]\n\t"		// t = tmp if t != 0 else t = 0

	    "xor	%[t], %[t], %[c]\n\t"		// t = t + c

	    "%=1:\n\t"
	    :
	    [reg] "=&r" (reg),
	    [c] "=&r" (c),
	    [t] "+r" (t),
	    [coeffp] "+r" (coeffp),
	    [tmp] "=&r" (tmp)
	    :
	    [loop0] "r" (info_len - 1),
	    [loop1] "r" (parity_len),
	    [vec] "r" (vec),
	    [vec_xs] "r" (vec_xs),
	    [index] "r" (gf8_index)
	    );

#else // !__XTENSA__

	    t = data[0];
	    for (int j = 1; j < info_len; j++) { // descending order
		t = gf8_add(gf8_mul(t, gf8_alpha(x)), data[j]);
	    }

	    t = gf8_mul(t, gf8_power(gf8_alpha(x), shortened_len)); // * x^(255 - code_len)

	    for (int j = info_len; j < code_len; j++) { // descending order
		t = gf8_add(gf8_mul(t, gf8_alpha(x)), data[j]);
	    }

#endif // __XTENSA__

	} else {

#ifdef __XTENSA__

	gf8_t indx = x;
	const gf8_t *vec = &gf8_vector[indx]; // vector table for * x
	
	uint32_t reg, c, tmp;
	uint8_t *coeffp = &data[1];

	t = data[0];

    asm volatile(
	    "loopgtz	%[loop], %=f\n\t"

	    "add	%[reg], %[index], %[t]\n\t"	// convert t to index form
	    "l8ui	%[reg], %[reg], 0\n\t"

	    "add	%[reg], %[vec], %[reg]\n\t"	// tmp = t * x
	    "l8ui	%[tmp], %[reg], 0\n\t"

	    "l8ui	%[c], %[coeffp], 0\n\t"		// load the coefficient
	    "addi	%[coeffp], %[coeffp], 1\n\t"	// advance the pointer

	    "movnez	%[t], %[tmp], %[t]\n\t"		// t = tmp if t != 0 else t = 0

	    "xor	%[t], %[t], %[c]\n\t"		// t = t + c

	    "%=:\n\t"
	    :
	    [reg] "=&r" (reg),
	    [c] "=&r" (c),
	    [t] "+r" (t),
	    [coeffp] "+r" (coeffp),
	    [tmp] "=&r" (tmp)
	    :
	    [loop] "r" (code_len - 1),
	    [vec] "r" (vec),
	    [index] "r" (gf8_index)
	    );

#else // !__XTENSA__

	t = data[0];
	for (int j = 1; j < code_len; j++) { // descending order
	    t = gf8_add(gf8_mul(t, gf8_alpha(x)), data[j]);
	}

#endif // !__XTENSA__

    }

	syndrome[parity_len - 1 - i] = t; // store reverse order for RS decoding

    }

    return RS8_OK;
}

/*
 * substitution
 *
 * using only the odd coefficients
 * (differenciating the polynomial and substituting x for error value calculation)
 *
 * input sigma(x), x
 * return value of dsigma(x)
 */
static gf8_t rs8_diff_subst(poly8_t *p, gf8_t x)
{

    int top = p->items - 1; // deg(p)

    if ((top & 1) == 0) top--; // most siginificant odd order

    if (top <= 0 || x == 0) return 0;

    gf8_t sum; // value of p(x)

#ifdef __XTENSA__

    sum = p->coeff[top];

    gf8_t indx = gf8_index[gf8_mul(x, x)]; // x^2
    const gf8_t *vec = &gf8_vector[indx];
    uint32_t reg, c;
    uint8_t *coeffp = &p->coeff[top - 2];

    asm volatile(
	    "loopgtz	%[loop], %=0f\n\t"

	    "beqz	%[t], %=1f\n\t"

	    "add	%[reg], %[index], %[t]\n\t"	// index of t
	    "l8ui	%[reg], %[reg], 0\n\t"

	    "add	%[reg], %[vec], %[reg]\n\t"	// mul by x^2
	    "l8ui	%[t], %[reg], 0\n\t"

	    "%=1:\n\t"

	    "l8ui	%[c], %[coeffp], 0\n\t"		// load coeff
	    "addi	%[coeffp], %[coeffp], -2\n\t"	// next odd coeff

	    "xor	%[t], %[t], %[c]\n\t"		// add coeff

	    "%=0:\n\t"
	    :
	    [reg] "=&r" (reg),
	    [c] "=&r" (c),
	    [t] "+r" (sum),
	    [coeffp] "+r" (coeffp)
	    :
	    [loop] "r" (top / 2),
	    [vec] "r" (vec),
	    [index] "r" (gf8_index)
	    );

#else // !__XTENSA__

    sum = p->coeff[top];
    gf8_t x2 = gf8_mul(x, x); // x^2
    for (int i = top - 2; i >= 1; i -= 2) {
	sum = gf8_add(gf8_mul(sum, x2), p->coeff[i]);
    }
    
#endif // __XTENSA__

    return sum;
}

/*
 * Reed-Solomon fast decoder for FX.25
 *
 * input parameter
 * data: received packet
 * code_len: length of received packet, 255, 144, 80, ..., etc
 * parity_len: length of parity, 2..64, even number
 */
int rs8_decode(uint8_t data[], int code_len, int parity_len)
{
    uint8_t *w; // for malloc
    poly8_t *s; // syndrome polynomial
    poly8_t *a; // x^(2*t)
    poly8_t *sigma;
    poly8_t *omega;
    //poly8_t *phi;

#if 0
    // check parity_len
    if (parity_len < RS8_PARITY_MIN || parity_len > RS8_PARITY_MAX || (parity_len & 1) != 0) {
	return RS8_ERR; // unsupported parity length
    }
#endif

    int size = (parity_len + 1)	// for *s
	+ (parity_len + 2)	// for *a
	+ sizeof(poly8_t) * 2;	// for *sigma, *omega
    w = malloc(size);
    assert(w != NULL);

    // syndrome
    s = (poly8_t *)w;
    s->items = parity_len;

    // calculate syndrome
    if (rs8_syndrome(data, code_len, s->coeff, parity_len) != RS8_OK) {
	return RS8_ERR;
    }

    if (poly8_iszero(s)) { // is syndrome zero?
	free(w);
	return RS8_OK; // there is no error, RS8_OK (0)
    }

    // set polynomial a(x) = x^(2*t)
    a = (poly8_t *)&s->coeff[s->items];
    a->items = parity_len + 1;
    poly8_clear(a);
    a->coeff[parity_len] = 1; // coefficient of x^(2*t)

    // sigam(x)
    sigma = (poly8_t *)&a->coeff[a->items];
    sigma->items = sizeof(poly8_t) - 1;
    
    // omega(x)
    omega = &sigma[1];
    omega->items = sizeof(poly8_t) - 1;

#if 0
    // phi(x)
    phi = &omega[1];
    phi->items = sizeof(poly8_t) - 1;
#endif
    
    // calculate sigma(x) and omega(x)
    poly8_euclid(a, s, sigma, omega, NULL, parity_len / 2);

    if (poly8_degree(sigma) > parity_len) { // something wrong
	free(w);

	return RS8_ERR;
    }

    // search error position and correct errors

    int errs = 0;
    for (int i = 0; i < code_len - parity_len; i++) {
	gf8_t an = gf8_alpha(254 - i); // alpha^i

	if (poly8_subst(sigma, an) == 0) { // found error position

	    gf8_t om = poly8_subst(omega, an);
	    gf8_t ds = rs8_diff_subst(sigma, an);

	    if (ds == 0) { // something wrong
		free(w);

		return RS8_ERR;
	    }

	    // get error value
#ifdef GEN_POLY_1
	    gf8_t e = gf8_div(om, gf8_mul(ds, an)); // need to divide by an
#else
	    gf8_t e = gf8_div(om, ds);
#endif

	    // correct error
	    data[i] ^= e;

	    errs++;
	    if (errs >= poly8_degree(sigma)) break; // all errors are corrected
	}
    }

#if 0
    if (errs != poly8_degree(sigma)) {
	free(w);
	return RS8_ERR;
    }
#endif

    free(w);

    return errs; // number of corrected errors
}

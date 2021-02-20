/*
 * bell202.c
 *
 * decode Bell 202 FSK 1200 baud signal
 */
#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"

#include "tnc.h"

#define TAG "bell202"

#if 0
// coefficient for low pass filter
static const int an[FIR_LPF_N] = {
#if FIR_LPF_N == 11
	    // LPF
	    // length: 11
	    // window: rectangular
	    // fc: 1200Hz
	    // multiplier: 2^16
	//-2618,-5080,-1527,8168,18786,23385,18786,8168,-1527,-5080,-2618 // 6728Hz
	47,331,1369,3236,5143,5957,5143,3236,1369,331,47 // 13200Hz
#elif FIR_LPF_N == 31
	    42,66,87,79,0,-175,-420,-635,-662,-334,453,1671,3139,4555,5582,5957,5582,4555,3139,1671,453,-334,-662,-635,-420,-175,0,79,87,66,42 // 13200Hz
#elif FIR_LPF_N == 15
	    -90,-61,149,862,2211,3922,5382,5958,5382,3922,2211,862,149,-61,-90
#else
#error
#endif
};
#endif

/*
 * low pass filter
 */
int bell202_decode(tcb_t *tp, int adc)
{
    //static int delayed[DELAYED_N];
    //static int delay_idx = 0;
    //static int x[FIR_LPF_N];
    //static int x_idx = 0;
    int m;
    int sum;

    //if (x_idx == 0) printf("%d ", adc);

    m = adc * tp->delayed[tp->delay_idx];
    tp->delayed[tp->delay_idx] = adc;
    tp->delay_idx = (tp->delay_idx + 1) % DELAYED_N;

#if 0
    tp->x[tp->x_idx] = m >> 7;

    sum = 0;
    for (i = 0; i < FIR_LPF_N; i++) {
	sum += an[i] * tp->x[(tp->x_idx + i) % FIR_LPF_N];
#if 0
	if (sum > (1 << 30)) printf("%d,", sum);
	else
	if (sum < -(1 << 30)) printf("%d,", sum);
#endif
    }
    tp->x_idx += FIR_LPF_N - 1;
    tp->x_idx %= FIR_LPF_N;

    //if (x_idx == 0) printf("%g ", sum);
#else
    sum = filter(tp->lpf, m >> 7);
#endif

    return sum;
}

void bell202_init(void)
{
    // do nothing
}

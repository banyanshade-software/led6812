/*
 * colorfunc.c
 *
 *  Created on: Jan 27, 2023
 *      Author: danielbraun
 */


#include <stdint.h>
#include "colorfunc.h"


#define PERIOD 100000
#define LEDVAL(_l) (((_l)*PERIOD)/NUM_LEDS)

static int32_t mapfunc_circular(const led_ramp_color_t *c, int lednum, int32_t t)
{
	int32_t v = LEDVAL(lednum) + t*c->w + c->t0;
	v = v % PERIOD;
	if (v<0) {
		v = (v + PERIOD) % PERIOD;
	}
	return v;
}

static uint8_t base_ifunc(const led_ramp_color_t *c, int32_t v)
{
	if (v<0) {
		return 0;
	}
	if (v > 2*c->p1) {
		return 0;
	}
	v = v - c->p1;
	if (v<0) v = -v;
	int k = (c->p1-v);

	return k*100/c->p1;
	//return k/100;
}
static uint8_t zero_ifunc(const led_ramp_color_t *c, int32_t v)
{
	return 0;
}



static int value_for(const led_ramp_color_t *c, int lednum, uint32_t t)
{
	mapfunc_t mfunc;
	switch (c->timefunc) {
	default: // FALLTHRU
	case TFUNC_CIRCULAR:
		mfunc = mapfunc_circular;
		break;
	}

	ifunc_t ifunc;
	switch (c->ifunc) {
	default: // FALLTHRU
	case IFUNC_ZERO:
		ifunc = zero_ifunc;
		break;
	case IFUNC_TRIANGLE:
		ifunc = base_ifunc;
		break;
	}

	int32_t v = mfunc(c, lednum, t);
	int k = ifunc(c, v);
	return k;
}

#define RGB(r,g,b)			((uint32_t)((g<<16)|(r<<8)|(b<<0)))

uint32_t rgb_for(const led_ramp_t *c, int lednum, uint32_t t)
{
	int r = value_for(&c->r, lednum, t);
	int g = value_for(&c->g, lednum, t);
	int b = value_for(&c->b, lednum, t);

	return RGB(r, g, b);
}

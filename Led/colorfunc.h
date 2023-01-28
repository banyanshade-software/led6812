/*
 * colorfunc.h
 *
 *  Created on: Jan 27, 2023
 *      Author: danielbraun
 */

#ifndef COLORFUNC_H_
#define COLORFUNC_H_

#include <stdint.h>

#define NUM_LEDS 30

typedef struct {
	uint8_t timefunc;
	uint8_t ifunc;
	// timefunc params
	int16_t t0;
	int16_t w;
	// ifunc params
	int16_t p1;
	int16_t p2;
} led_ramp_color_t;

typedef struct {
	led_ramp_color_t r;
	led_ramp_color_t g;
	led_ramp_color_t b;
} led_ramp_t;

#define TFUNC_CIRCULAR 1

#define IFUNC_ZERO     1
#define IFUNC_TRIANGLE 2

uint32_t rgb_for(const led_ramp_t *c, int lednum, uint32_t t);

/*
 * function that map LED number and time to [0:100000]
 */
typedef int32_t (*mapfunc_t)(const led_ramp_color_t *c, int lednum, int32_t t);


typedef uint8_t (*ifunc_t)(const led_ramp_color_t *c, int32_t v);



#endif /* COLORFUNC_H_ */

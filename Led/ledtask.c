/*
 * ledtask.c
 *
 *  Created on: Jan 26, 2023
 *      Author: danielbraun
 */

#include <stdint.h>
#include "main.h"
#include "cmsis_os.h"
#include "ledtask.h"
#include "colorfunc.h"


#define ABS(_a) ((_a)<0 ? -(_a) : (_a))

//#define NO_LEDS             30
#define NO_COLORS           3
#define NO_BIT_DATA         (NUM_LEDS * NO_COLORS * 4 + 4)
#define BIT_ZERO            0x08
#define BIT_ONE             0x0C
#define RGB(r,g,b)			((uint32_t)((g<<16)|(r<<8)|(b<<0)))
static uint8_t bit_data[NO_BIT_DATA] = {0x00};

static volatile uint8_t dmaOnProgress = 0;

static void ledTick(uint32_t t);

uint32_t ntick1 = 0;
uint32_t ntick2 = 0;

static volatile uint32_t t0,t1,t2,t3,t4,t5;

static volatile int reset_time = 0;



void StartMainTask(void const * argument)
{
	reset_time = 1;
	for (;;) {
		uint32_t notif = 0;
		t5 = HAL_GetTick();
		xTaskNotifyWait(0, 0xFFFFFFFF, &notif, portMAX_DELAY);
		ntick1++;
		uint32_t t1 = HAL_GetTick();
		ledTick(t1);
		//(void)t0; (void)t1; // t0 and t1 are unused, only used if time measurement is needed
	}
}
static void Fill_BitData(uint32_t);


static void ledTick(uint32_t t)
{
	static uint32_t tbase = 0;
	if (dmaOnProgress) {
		// error ?
		// DMA took more than 10ms to complete, this is not
		// normal and should not happen (and does not)
		t4 = HAL_GetTick();
	} else {
		if (reset_time) {
			reset_time = 0;
			tbase = t;
		}
		t0 = HAL_GetTick();
		// Board LED for debug
		if ((1)) {
			static int n=0;
			n++;
			if ((n%10)==0) HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
		}

		Fill_BitData(t-tbase);
		dmaOnProgress = 1;
		t1 = HAL_GetTick();
		HAL_SPI_Transmit_DMA(&hspi1, bit_data, sizeof(bit_data));
		t2 = HAL_GetTick();
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	t3 = HAL_GetTick();
	dmaOnProgress = 0;
}


static void _gen_bitarray(uint32_t *color_map, int numled);



static void _gen_bitarray(uint32_t *color_map, int numled)
{
    int i, j=0, k;
    int upper = 0;
    uint8_t nibble = 0x00;

    bit_data[j++] = 0x00;
    bit_data[j++] = 0x00;

    for(i = 0; i < numled; i++)  {
        for(k = 0; k < 24; k++) {
            if ((color_map[i] << (k + 8)) & 0x80000000 ) {
                nibble = BIT_ONE;
            } else {
                nibble = BIT_ZERO;
            }

            if (upper) {
                bit_data[j++] |= (nibble << 4);
                upper = 0;
            } else {
                bit_data[j] = nibble;
                upper = 1;
            }
        }
    }
}

#define NUM_PATTERN 9

static const led_ramp_t ramps[NUM_PATTERN] = {
		{
				/*r*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=40,   .ifunc=IFUNC_TRIANGLE, .p1=25000, .p2=0},
				/*g*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=41,   .ifunc=IFUNC_TRIANGLE, .p1=25000, .p2=0},
				/*b*/ {.timefunc = TFUNC_CIRCULAR, .t0=0,      .w=42,   .ifunc=IFUNC_TRIANGLE, .p1=25000, .p2=0},
		},
		{
				/*r*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=40,   .ifunc=IFUNC_TRIANGLE, .p1=25000, .p2=0},
				/*g*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=41,   .ifunc=IFUNC_TRIANGLE, .p1=25000, .p2=0},
				/*b*/ {.timefunc = TFUNC_CIRCULAR, .t0=0,      .w=-42,   .ifunc=IFUNC_TRIANGLE, .p1=25000, .p2=0},
		},
		{
				/*r*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=250,   .ifunc=IFUNC_TRIANGLE, .p1=25000, .p2=0},
				/*g*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=521,   .ifunc=IFUNC_TRIANGLE, .p1=25000, .p2=0},
				/*b*/ {.timefunc = TFUNC_CIRCULAR, .t0=0,      .w=-72,   .ifunc=IFUNC_TRIANGLE, .p1=15000, .p2=0},
		},
		{
				/*r*/ {.timefunc = TFUNC_CIRCULAR, .t0=0,  .w=5,   .ifunc=IFUNC_TRIANGLE, .p1=5000, .p2=0},
				/*g*/ {.timefunc = TFUNC_CIRCULAR, .t0=0,  .w=6,    .ifunc=IFUNC_TRIANGLE, .p1=5000, .p2=0},
				/*b*/ {.timefunc = TFUNC_CIRCULAR, .t0=0,  .w=7,    .ifunc=IFUNC_TRIANGLE, .p1=5000, .p2=0},
		},
		{
				/*r*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=80,   .ifunc=IFUNC_TRIANGLE, .p1=15000, .p2=0},
				/*g*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=81,  .ifunc=IFUNC_TRIANGLE, .p1=15000, .p2=0},
				/*b*/ {.timefunc = TFUNC_CIRCULAR, .t0=0,      .w=-72,   .ifunc=IFUNC_TRIANGLE, .p1=15000, .p2=0},
		},
		{
				/*r*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=40,   .ifunc=IFUNC_TRIANGLE, .p1=10000, .p2=0},
				/*g*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=40,   .ifunc=IFUNC_TRIANGLE, .p1=10000, .p2=0},
				/*b*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=40,   .ifunc=IFUNC_TRIANGLE, .p1=10000, .p2=0},
		},
		{
				/*r*/ {.timefunc = TFUNC_CIRCULAR, .t0=8000,  .w=40,   .ifunc=IFUNC_TRIANGLE, .p1=10000, .p2=0},
				/*g*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=40,   .ifunc=IFUNC_TRIANGLE, .p1=10000, .p2=0},
				/*b*/ {.timefunc = TFUNC_CIRCULAR, .t0=12000,  .w=40,   .ifunc=IFUNC_TRIANGLE, .p1=10000, .p2=0},
		},
		{
				/*r*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=40,   .ifunc=IFUNC_ZERO, .p1=10000, .p2=0},
				/*g*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=40,   .ifunc=IFUNC_TRIANGLE, .p1=30000, .p2=0},
				/*b*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=40,   .ifunc=IFUNC_ZERO, .p1=10000, .p2=0},
		},
		{
				/*r*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=40,   .ifunc=IFUNC_TRIANGLE, .p1=10000, .p2=0},
				/*g*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=40,   .ifunc=IFUNC_ZERO, .p1=10000, .p2=0},
				/*b*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=40,   .ifunc=IFUNC_ZERO, .p1=10000, .p2=0},
		},
};

static volatile int16_t pattern_number_raw = 0;

static void bh(void)
{

}

static void Fill_BitData(uint32_t t)
{
    static uint32_t color_map[NUM_LEDS] = {0};
    int rampnum = ABS(pattern_number_raw) % NUM_PATTERN;
    if (rampnum==2) {
    	bh();
    }
    const led_ramp_t *ramp = &ramps[rampnum];
    for (int i=0; i<NUM_LEDS; i++) {
    	//color_map[i] = RGB(0, 10, 0);
   		color_map[i] = rgb_for(ramp, i, t);
    }

    _gen_bitarray(color_map, NUM_LEDS);
}




// ----------------------------------------

// CtrlTask unused for now, intended (f.i.) to handle commands on usb/serial
// to configure LEDs

extern TIM_HandleTypeDef htim1;


static int16_t get_rotary(TIM_HandleTypeDef *ptdef)
{
	uint16_t p = __HAL_TIM_GET_COUNTER(ptdef);

	int16_t sp = (int16_t)p;
	return sp/2;
}




void StartCtrlTask(void const * argument)
{
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	int16_t rot = get_rotary(&htim1);
	for (;;) {
		uint32_t notif = 0;
		xTaskNotifyWait(0, 0xFFFFFFFF, &notif, portMAX_DELAY);
		int16_t nrot = get_rotary(&htim1);
		if (nrot != rot) {
			// action
			pattern_number_raw = nrot;
			reset_time = 1;
			// update rot
			rot = nrot;
		}

		// osDelay(1);

		ntick2++;
	}
}


void vApplicationStackOverflowHook( TaskHandle_t xTask,  signed char *pcTaskName)
{
	//itm_debug1(DBG_ERR, "STK OVF", 1);
	//FatalError("STKo", "stack overflow", Error_Stack);
	for (;;) {

	}
}


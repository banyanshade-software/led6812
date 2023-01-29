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

void StartMainTask(void const * argument)
{
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
	if (dmaOnProgress) {
		// error ?
		// DMA took more than 10ms to complete, this is not
		// normal and should not happen (and does not)
		t4 = HAL_GetTick();
	} else {
		t0 = HAL_GetTick();
		// Board LED for debug
		if ((1)) {
			static int n=0;
			n++;
			if ((n%10)==0) HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
		}

		Fill_BitData(t);
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


static const led_ramp_t ramp = {
		/*r*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=59,   .ifunc=IFUNC_TRIANGLE, .p1=10000, .p2=0},
		/*g*/ {.timefunc = TFUNC_CIRCULAR, .t0=10000,  .w=60,  .ifunc=IFUNC_TRIANGLE, .p1=5000, .p2=0},
		/*b*/ {.timefunc = TFUNC_CIRCULAR, .t0=0,  .w=-35,   .ifunc=IFUNC_TRIANGLE, .p1=5000, .p2=0},
};


static void Fill_BitData(uint32_t t)
{
    static uint32_t color_map[NUM_LEDS] = {0};

    for (int i=0; i<NUM_LEDS; i++) {
   		color_map[i] = rgb_for(&ramp, i, t);
    }

    _gen_bitarray(color_map, NUM_LEDS);
}


#if 0
# if 0
// obsolete, may still be usefull for tests
static void Fill_BitData(uint32_t t)
{
    static uint32_t color_map[NO_LEDS] = {
        RGB(8,  0,  0),
		RGB(50,  0,  0),
		RGB(  100,  0,  0),
		RGB(  0,  8,  0),
		RGB(  0,  50,  0),
		RGB(  0,  100,  0),
		RGB(  0,  0,  8),
		RGB(  0,  0,  50),
		RGB(  0,  0,  100),


		RGB(  100,  0,  0),
		RGB(  90,  0,  10),
		RGB(  80,  0,  20),
		RGB(  70,  0,  30),
		RGB(  60,  0,  40),
		RGB(  50,  0,  50),
		RGB(  40,  0,  60),
		RGB(  30,  0,  70),
		RGB(  20,  0,  80),
		RGB(  10,  0,  90),
		RGB(  0,  0,  100),

		RGB(  0,  10,  90),
		RGB(  0,  20,   80),
		RGB(  0,  30,   70),
		RGB(  0,  40,   60),
		RGB(  0,  50,   50),
		RGB(  0,  60,   40),
		RGB(  0,  70,   30),
		RGB(  0,  80,   20),
		RGB(  0,  90,   10),
		RGB(  0,  100,   0),


    };
    if ((0)) {
    	int	rand (void);
    	for (int i=10; i<NO_LEDS; i++) {
    		color_map[i] = RGB(rand()%50, rand()%50, rand()%50);
    	}
    }
    _gen_bitarray(color_map, NO_LEDS);
}

#else


#define PERIOD 100000
#define BACK 0
#define LEDVAL(_l) (((_l)*PERIOD)/NO_LEDS)

static int color_func(int led, int val)
{
	int v;
	if (BACK) {
		int t = val % (2*PERIOD);
		t = t-PERIOD;
		if (t<0) {
			v = LEDVAL(led)-t;
		} else {
			v = LEDVAL(led)+t;
		}
	} else {
		v = LEDVAL(led)+val;
	}
	v = v % PERIOD;


	if (v<0) {
		v = (v + PERIOD) % PERIOD;
	}
	if (v<0) {
		return 0;
	}
	if ((v>20000)) {
		return 0;
	}
	v = v-10000;
	if (v<0) v=-v;
	int k = (10000-v);

	return k/100;
}


static void Fill_BitData(uint32_t t)
{
    static uint32_t color_map[NO_LEDS] = {0};

    for (int i=0; i<NO_LEDS; i++) {
    	int r = color_func(i, 30*t);
    	int g = color_func(i, LEDVAL(i)-20*t+5000);
    	int b = color_func(i, LEDVAL(i)+29*t+10000);
   		color_map[i]=RGB(r, g, b);
    }
    if ((0)) {
    	int	rand (void);
    	for (int i=10; i<NO_LEDS; i++) {
    		color_map[i] = RGB(rand()%50, rand()%50, rand()%50);
    	}
    }
    _gen_bitarray(color_map, NO_LEDS);
}

#endif
#endif


// ----------------------------------------

// CtrlTask unused for now, intended (f.i.) to handle commands on usb/serial
// to configure LEDs



void StartCtrlTask(void const * argument)
{
	for (;;) {
		uint32_t notif = 0;
		xTaskNotifyWait(0, 0xFFFFFFFF, &notif, portMAX_DELAY);
		osDelay(1);

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


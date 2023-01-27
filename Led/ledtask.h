/*
 * ledtask.h
 *
 *  Created on: Jan 26, 2023
 *      Author: danielbraun
 */

#ifndef LED_LEDTASK_H_
#define LED_LEDTASK_H_

extern SPI_HandleTypeDef hspi1;


#define NOTIF_TICK 0x00000001


void StartMainTask(void const * argument);

extern volatile uint8_t ledReady;

#endif /* LED_LEDTASK_H_ */

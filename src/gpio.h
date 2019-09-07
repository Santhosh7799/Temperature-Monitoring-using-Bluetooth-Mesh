/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "common.h"
//#include "gpiointerrupt.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

extern uint8_t Pb0_status;
extern uint8_t Pb1_status;

#define	LED0_port gpioPortF
#define LED0_pin  4
#define LED1_port gpioPortF
#define LED1_pin  5


#define GPIO_SET_DISPLAY_EXT_COMIN_IMPLEMENTED 	1

#define GPIO_DISPLAY_SUPPORT_IMPLEMENTED		1

#define DISPLAY_ENABLE_PORT gpioPortD
#define DISPLAY_ENABLE_PIN  15
#define DISPLAY_EXTCOMIN_PORT gpioPortD
#define DISPLAY_EXTCOMIN_PIN  13


void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();
void gpioEnableDisplay();
void gpioSetDisplayExtcomin(bool high);
void enable_PB0_interrupt();
void disable_PB0_interrupt();
#endif /* SRC_GPIO_H_ */

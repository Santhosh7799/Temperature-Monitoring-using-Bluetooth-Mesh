/*
 * File : Le_timer.h
 * Author: Santhosh Thummanapalli
 * Created on 1/29/2019
 *
 *Description: This is header file used for function declarations  for the le_timer
 *
 *
 */

#ifndef LE_TIMER_H_
#define LE_TIMER_H_
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_letimer.h"
//#include "common.h"

extern uint32_t TotalCyclesCompleted;
extern uint32_t Reload_value;

#define TOTAL_PERIOD  1000

#define SCHEDULER_SUPPORTS_DISPLAY_UPDATE_EVENT 1
#define TIMER_SUPPORTS_1HZ_TIMER_EVENT	1

typedef enum EnergyModeVal{
	EM0,
	EM1,
	EM2,
	EM3,
	EM4
}Energystate;


uint8_t Allow_temp;
uint32_t PrescalerValCal(uint32_t period);
void letimer_init();
void timerWaitUs(uint32_t us_wait);
void timerSetEventInMs(uint32_t ms_wait);

#endif

/*
 * File : Le_timer.c
 * Author: Santhosh Thummanapalli
 * Created on 1/29/2019
 *
 *Description: This is the file used for function definitions  for the le_timer
 *
 *
 */

#include "letimer.h"
#include "gpio.h"
#include "sleep.h"
#include "log.h"
#include "display.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"


uint32_t TotalCyclesCompleted = 0;
uint32_t Reload_value =0;
uint32_t PrescalerValCal(uint32_t period)
{
   uint32_t a,b;
   b=0;
   a = ((65535 * 1000)/period);
   if(a >= 32768)
   {
	   b=1;
   }
   else if(32768>a && a>= 16384)
   {
	   b=2;
   }
   else if(a < 16384 && a>= 8192)
   {
	   b=4;
   }
   else
	   printf("please enter value less than 8000");

   return b;
}




void letimer_init()
{
	uint32_t Divider,comp0value,Clockfreq; //comp1value is removed

	CMU_ClockEnable(cmuClock_HFLE, true);

	CMU_OscillatorEnable (cmuOsc_LFXO, true, true);
    CMU_ClockSelectSet (cmuClock_LFA, cmuSelect_LFXO);
	Divider = PrescalerValCal(TOTAL_PERIOD);
	CMU_ClockEnable(cmuClock_LFA,true);
	CMU_ClockDivSet(cmuClock_LETIMER0,Divider);
	CMU_ClockEnable(cmuClock_LETIMER0,true);

	LETIMER_Init_TypeDef letimerInit = LETIMER_INIT_DEFAULT;
	letimerInit.comp0Top  = true;
	LETIMER_Init(LETIMER0, &letimerInit );



	Clockfreq = CMU_ClockFreqGet(cmuClock_LFA);


	comp0value=((Clockfreq*TOTAL_PERIOD)/(Divider*1000));
	Reload_value = comp0value;

	LETIMER_CompareSet(LETIMER0, 0, comp0value);


	LETIMER_IntEnable(LETIMER0,LETIMER_IF_UF);
	NVIC_EnableIRQ(LETIMER0_IRQn);
	LETIMER_Enable(LETIMER0, true);
}

void timerWaitUs(uint32_t us_wait)
{
//	GPIO_PinOutSet(LED0_port, LED0_pin);
	uint32_t Clockfreq,Required_ticks,initial_tickvalue,comp1value;
	Clockfreq = CMU_ClockFreqGet(cmuClock_LFA);
	Required_ticks = us_wait/Clockfreq ;
	initial_tickvalue= LETIMER_CounterGet(LETIMER0);
	if(Required_ticks < initial_tickvalue)
	{
		comp1value = initial_tickvalue - Required_ticks;
	}
	else
	{
		comp1value = Reload_value - (Required_ticks - initial_tickvalue);
	}
	LETIMER_CompareSet(LETIMER0, 0, comp1value);
	LETIMER_IntEnable(LETIMER0,LETIMER_IF_COMP1);
}




// need to change the handler for this and update it

void timerSetEventInMs(uint32_t ms_wait)
{
//	GPIO_PinOutSet(LED0_port, LED0_pin);
	uint32_t Clockfreq,Required_ticks,initial_tickvalue,comp1value;

	Clockfreq = CMU_ClockFreqGet(cmuClock_LETIMER0);
	Required_ticks = (ms_wait*Clockfreq)/1000 ;
	initial_tickvalue= LETIMER_CounterGet(LETIMER0);
	if(Required_ticks < initial_tickvalue)
	{
		comp1value = initial_tickvalue - Required_ticks;
	}
	else
	{
		comp1value = Reload_value - (Required_ticks - initial_tickvalue);
	}
	LETIMER_CompareSet(LETIMER0, 1, comp1value);
    LETIMER_IntClear(LETIMER0,LETIMER_IF_COMP1);
	LETIMER_IntEnable(LETIMER0,LETIMER_IEN_COMP1);
}




void LETIMER0_IRQHandler(void)
{

  // Get pending flags and clear
  int irq_flags =  LETIMER_IntGet(LETIMER0);
  LETIMER_IntClear(LETIMER0, irq_flags);
  if((LETIMER_IF_UF & irq_flags))
  {

	//  gecko_external_signal(EventHandlePowerOn);
	  gecko_external_signal(EventHandle1Sec);
	  SchedulerEventSet[EventHandle1Sec]=1;
	  TotalCyclesCompleted++;
		//displayUpdate();
	   // gecko_external_signal(EventHandlePowerOn);
	 // LOG_INFO("Timer flag set");
  }
  if((LETIMER_IF_COMP1 & irq_flags))
  {
	//  SchedulerEventSet[EventHandleI2CEnabled]=1;
	//  LETIMER_IntDisable(LETIMER0,LETIMER_IF_COMP1);
	//  gecko_external_signal(EventHandleI2CEnabled);
//	  GPIO_PinOutClear(LED0_port, LED0_pin);
  }
 //GPIO_PinOutToggle(LED0_port, LED0_pin);

}

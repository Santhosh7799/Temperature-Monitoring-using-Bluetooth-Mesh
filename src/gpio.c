/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 */
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>
#include "hal-config-board.h"


#define	LED0_port gpioPortF
#define LED0_pin	4
#define LED1_port gpioPortF
#define LED1_pin 5

uint8_t Pb0_status =0;
uint8_t Pb1_status =0;

void gpioInit()
{
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateStrong);
	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);
	GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPull, 1);
	  GPIO_PinModeSet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, gpioModeInputPull, 1);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, true, true, true);
    GPIO_ExtIntConfig(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, BSP_BUTTON1_PIN, true, true, true);
}

void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}


void gpioEnableDisplay()
{
	GPIO_PinModeSet(DISPLAY_ENABLE_PORT, DISPLAY_ENABLE_PIN, gpioModePushPull, 1);
	GPIO_PinOutSet(DISPLAY_ENABLE_PORT,DISPLAY_ENABLE_PIN);
	GPIO_PinModeSet(DISPLAY_EXTCOMIN_PORT, DISPLAY_EXTCOMIN_PORT, gpioModePushPull, 1);
}
void gpioSetDisplayExtcomin(bool high)
{
	if(high == true)
	{
		GPIO_PinOutSet(DISPLAY_EXTCOMIN_PORT, DISPLAY_EXTCOMIN_PIN);
	}
	else
	{
		GPIO_PinOutClear(DISPLAY_EXTCOMIN_PORT, DISPLAY_EXTCOMIN_PIN);
	}
}


//code for button interrupts--working but is commented due to use of callback register.

/*void enable_PB0_interrupt(void)
{

	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	GPIO_ExtIntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, BSP_BUTTON0_PIN, true, false, true);
}

void disable_PB0_interrupt()
{
	GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeDisabled, 0);
	NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
	NVIC_DisableIRQ(GPIO_EVEN_IRQn);

}*/

void GPIO_EVEN_IRQHandler(void)
{
  // Clear all even pin interrupt flags
  uint32_t even_int_flag = GPIO_IntGet();
  GPIO_IntClear(even_int_flag);
  Pb0_status = GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN);
  SchedulerEventSet[EventHandlePB0Pressed]=1;
  gecko_external_signal(EventHandlePB0Pressed);
}

void GPIO_ODD_IRQHandler(void)
{
  // Clear all even pin interrupt flags
	 uint32_t odd_int_flag = GPIO_IntGet();
	 GPIO_IntClear(odd_int_flag);
  Pb1_status = GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN);
  SchedulerEventSet[EventHandlePB1Pressed]=1;
  gecko_external_signal(EventHandlePB1Pressed);
}

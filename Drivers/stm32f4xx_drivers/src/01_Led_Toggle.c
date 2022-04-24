/*
 * 01_Led_Toggle.c
 *
 *  Created on: Oct 3, 2021
 *      Author: Abdel
 */


#include "stm32f407xx.h"

void delay()
{
	for (uint32_t i = 0 ; i < 500000 ; i ++);
}

int main()
{
	GPIO_Handle_t BRD_led;

	BRD_led.pGPIOx = GPIOD;
	BRD_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	BRD_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	BRD_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	BRD_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	BRD_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	 GPIO_PeriClockControl(GPIOD,ENABLE);
	 GPIO_Init(&BRD_led);

	while(1)
	{
		GPIO_ToggleOutputpin(GPIOD,GPIO_PIN_NO_13);
		delay();
	}
   return 0;
}

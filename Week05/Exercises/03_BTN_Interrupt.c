/*
 * 03_BTN_Interrupt.c
 *
 *  Created on: Oct 4, 2021
 *      Author: Abdel
 */

#include "stm32f407xx.h"
#include <string.h>

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int main()
{
	GPIO_Handle_t BRD_led,BRD_BTN;
	memset(&BRD_led,0,sizeof(BRD_led));
	memset(&BRD_BTN,0,sizeof(BRD_BTN));


	BRD_led.pGPIOx = GPIOD;
	BRD_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	BRD_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	BRD_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	BRD_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	BRD_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&BRD_led);

	BRD_BTN.pGPIOx = GPIOA;
	BRD_BTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	BRD_BTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	BRD_BTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	BRD_BTN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&BRD_BTN);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI0 , ENABLE);
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI0,NVIC_IRQ_PRI15 ) ;
    while(1);

   return 0;
}

void EXTI0_IRQHandler()
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);

	GPIO_ToggleOutputpin(GPIOD,GPIO_PIN_NO_13);

}

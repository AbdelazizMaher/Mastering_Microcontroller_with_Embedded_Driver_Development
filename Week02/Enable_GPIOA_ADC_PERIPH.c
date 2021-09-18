#include <stdint.h>
#include "stm32f446xx.h"


int main()
{
	ADC_TypeDef *pADC;
	RCC_TypeDef *pRCC;
	GPIO_TypeDef *pGPIO;
	
	pRCC=RCC;
	pADC=ADC;
	pGPIO=GPIOA;
	
	pRCC->APB2ENR =pRCC->APB2ENR |(1<<8);
	pRCC->AHB1ENR =pRCC->AHB1ENR |(1);
	
	pADC->CR1=0x55;
	pGPIO->BSRR=0x11;
	
	return 0;
}
	
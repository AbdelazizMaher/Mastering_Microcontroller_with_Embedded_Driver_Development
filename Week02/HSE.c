#include <stdint.h>
#include "stm32f407xx.h"


int main()
{
	RCC_TypeDef*pRCC;
	pRCC=RCC;
	
	pRCC->CR |= (1<<16);
	
	while(!(pRCC->CR & (1<<17) ));
	
	pRCC->CFGR &= ~(0x3);
	pRCC->CFGR |= (0x1);
	
	return 0;
}
	
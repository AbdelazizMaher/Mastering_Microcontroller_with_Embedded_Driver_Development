
#include <stdint.h>

#include "stm32f446xx.h"

RTC_TypeDef *pRTC;

int main()
{
	
	pRTC=RTC;
	pRTC->ALRMBR=0x24;
	
	
	return 0;
}
	
/*
 * 04_SPI_Tx_Testing.c
 *
 *  Created on: Oct 13, 2021
 *      Author: Abdel
 */

#include<string.h>
#include "stm32f407xx.h"


/**
  * PB14 --> SPI2_MISO
  * PB15 --> SPI2_MOSI
  * PB13 -> SPI2_SCLK
  * PB12 --> SPI2_NSS
  * ALT function mode : 5
  */

void GPIO_Def()
{
	GPIO_Handle_t GPIO_SPI_Con;

	GPIO_SPI_Con.pGPIOx = GPIOB;
	GPIO_SPI_Con.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_ALTFN;
	GPIO_SPI_Con.GPIO_PinConfig.GPIO_AltFunMode = 5;
	GPIO_SPI_Con.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_SPI_Con.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_SPI_Con.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	GPIO_SPI_Con.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&GPIO_SPI_Con);

	//MOSI
	GPIO_SPI_Con.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&GPIO_SPI_Con);

	//MISO
	//GPIO_SPI_Con.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&GPIO_SPI_Con);

	//NSS
	//GPIO_SPI_Con.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	//GPIO_Init(&GPIO_SPI_Con);


}

void SPI_Def()
{
	SPI_Handle_t SPI2_Test;

	SPI2_Test.pSPIx = SPI2;
	SPI2_Test.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Test.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2_Test.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_Test.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2_Test.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Test.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Test.SPIConfig.SPI_SSM = SPI_SSM_EN ;

	SPI_Init(&SPI2_Test);

}

int main()
{
	GPIO_Def();

	SPI_Def();

	char Data[] = "Hello World";

	SPI_SSIConfig(SPI2,ENABLE);

	SPI_PeriphControl(SPI2,ENABLE);

	SPI_SendData(SPI2,(uint8_t*)Data, strlen(Data));

	while(SPI_GetFlagStatus(SPI2 , SPI_BUSY_FLAG));

	SPI_PeriphControl(SPI2,DISABLE);

	while(1);

   return 0;
}

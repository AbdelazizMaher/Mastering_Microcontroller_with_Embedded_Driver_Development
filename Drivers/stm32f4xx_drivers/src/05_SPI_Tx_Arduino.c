/*
 * 05_SPI_Tx_Arduino.c
 *
 *  Created on: Oct 15, 2021
 *      Author: Abdel
 */



#include<string.h>
#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

/**
  * PB14 --> SPI2_MISO
  * PB15 --> SPI2_MOSI
  * PB13 -> SPI2_SCLK
  * PB12 --> SPI2_NSS
  * ALT function mode : 5
  */


/* SPI Slave Demo
 *
 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select . Arduino SPI pins respond only if SS pulled low by the master
 *
 */

void GPIO_SPIDef()
{
	GPIO_Handle_t GPIO_SPI_Con;

	GPIO_SPI_Con.pGPIOx = GPIOB;
	GPIO_SPI_Con.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_ALTFN;
	GPIO_SPI_Con.GPIO_PinConfig.GPIO_AltFunMode = 5;
	GPIO_SPI_Con.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_SPI_Con.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
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
	GPIO_SPI_Con.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&GPIO_SPI_Con);


}

void SPI_Def()
{
	SPI_Handle_t SPI2_Test;

	SPI2_Test.pSPIx = SPI2;
	SPI2_Test.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_Test.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2_Test.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_Test.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2_Test.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_Test.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2_Test.SPIConfig.SPI_SSM = SPI_SSM_DI ;

	SPI_Init(&SPI2_Test);

}

void GPIO_BTN_Def()
{
	GPIO_Handle_t BRD_BTN;

	 BRD_BTN.pGPIOx = GPIOA;
	 BRD_BTN.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	 BRD_BTN.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	 BRD_BTN.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	 BRD_BTN.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	 GPIO_Init(&BRD_BTN);
}


int main()
{
	//this functions is used in initialization
	GPIO_SPIDef();
	SPI_Def();
	GPIO_BTN_Def();

	SPI_SSOEConfig(SPI2,ENABLE);

	char Data[] = "Hello World";

	while(1)
	{
		while(!GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		uint8_t DataLen = strlen(Data);

		//enable the SPI2 peripheral
		SPI_PeriphControl(SPI2,ENABLE);

		//sending length information
		SPI_SendData(SPI2,&DataLen, 1);

		SPI_SendData(SPI2,(uint8_t*)Data, strlen(Data));

		//confirming SPI is not busy
		while(SPI_GetFlagStatus(SPI2 , SPI_BUSY_FLAG));

		SPI_PeriphControl(SPI2,DISABLE);
	}


   return 0;
}


/*
 * 08_I2C_Master_Tx_Testing.c
 *
 *  Created on: Oct 31, 2021
 *      Author: Abdel
 */

#include "stm32f407xx.h"
#include<stdio.h>
#include<string.h>

#define MY_ADDR 0x61;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

/**
  * PB6-> SCL
  * PB9 or PB7 -> SDA
  */
void GPIO_I2CDef()
{
	GPIO_Handle_t GPIO_I2C_Con;

	GPIO_I2C_Con.pGPIOx = GPIOB;
	GPIO_I2C_Con.GPIO_PinConfig.GPIO_PinMode =  GPIO_MODE_ALTFN;
	GPIO_I2C_Con.GPIO_PinConfig.GPIO_AltFunMode = 4;
	GPIO_I2C_Con.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIO_I2C_Con.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIO_I2C_Con.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	GPIO_I2C_Con.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&GPIO_I2C_Con);

	//sda
	//Note : since we found a glitch on PB9 , you can also try with PB7
	GPIO_I2C_Con.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&GPIO_I2C_Con);
}

void I2C_Def()
{
	I2C_Handle_t I2C_Test;

	I2C_Test.pI2Cx = I2C1;
	I2C_Test.I2C_Config->I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C_Test.I2C_Config->I2C_AckControl = I2C_ACK_ENABLE;
	I2C_Test.I2C_Config->I2C_DeviceAddress = MY_ADDR;
	I2C_Test.I2C_Config->I2C_FMDutyCycle = I2C_FM_DUTY_2;

	I2C_Init(&I2C_Test);
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
	GPIO_I2CDef();
	I2C_Def();
	GPIO_BTN_Def();


    return 0;
}

/*
 * stm32f407xx_I2C_driver.c
 *
 *  Created on: Oct 25, 2021
 *      Author: Abdel
 */
#include "stm32f407xx_I2C_driver.h"

uint16_t AHB_prescaler[8]={2,4,8,16,64,128,256,512};
uint16_t ABP_prescaler[4]={2,4,8,16};

static void I2C_SendSlaveAddressWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_SendSlaveAddressRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);


void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= 1 << I2C_CR1_START;
}

static void I2C_SendSlaveAddressWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR |= SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
    uint32_t dummy_read;
    dummy_read = pI2Cx->SR1;
    dummy_read = pI2Cx->SR2;
	(void)dummy_read;
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= 1 << I2C_CR1_STOP;
}

static void I2C_SendSlaveAddressRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= (1);
	pI2Cx->DR |= SlaveAddr;
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}
	else
	{
		//disable the ack
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}


uint32_t RCC_GetPLLOutputClk()
{
	return 0;
}



uint32_t RCC_GetAPB1Clock()
{
	uint32_t PCLK1,SYSCLCK;
	uint8_t Clksrc,AHB1p,temp,APB1p;

	Clksrc = ((RCC->CFGR >>2) & (0x3));

	if(Clksrc == 0)
	{
		SYSCLCK = 16000000;
	}
	else if(Clksrc == 1)
	{
		SYSCLCK = 8000000;
	}
	else if(Clksrc == 2)
	{
		SYSCLCK = RCC_GetPLLOutputClk();
	}

	temp = ((RCC->CFGR >>4) & (0xF));

	if(temp < 8)
	{
		AHB1p= 1;
	}
	else
	{
		AHB1p = AHB_prescaler[temp-8];
	}

	temp = ((RCC->CFGR >>10) & (0x3));

	if(temp < 4)
	{
		APB1p= 1;
	}
	else
	{
		APB1p = ABP_prescaler[temp-4];
	}

	PCLK1 = (SYSCLCK/AHB1p)/APB1p;

	return PCLK1;
}


/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	 }
}

/*********************************************************************
 * @fn      		  - I2C_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	uint32_t tempreg=0;

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg = RCC_GetAPB1Clock()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress >>  1;
	tempreg |= 1 << 14;
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	tempreg = 0;
	uint16_t CCR_value = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		CCR_value = RCC_GetAPB1Clock() / (2* pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (CCR_value & 0xFFF);
	}
	else
	{
		//mode is fast mode
		tempreg |= 1 << 15;
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			CCR_value = RCC_GetAPB1Clock() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			CCR_value = RCC_GetAPB1Clock() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (CCR_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration

}

/*********************************************************************
 * @fn      		  - I2C_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/**
  *  FlagStatus
  */

/*********************************************************************
 * @fn      		  - I2C_GetFlagStatus
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_SB)));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
    I2C_SendSlaveAddressWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR )));

	//5. clear the ADDR flag according to its software sequence
    I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	//6. send the data until len becomes 0
	while(Len >0)
	{
		while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_TXE)));
		//Wait till TXE is set
		pI2CHandle->pI2Cx->DR |= *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_TXE)));
	while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_BTF)));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_SendSlaveAddressRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in teh SR1
	while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_ADDR )));

	//procedure to read only 1 byte from slave
    if(Len == 1)
    {
		//Disable Acking
    	I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

    	// clear the ADDR flag according to its software sequence
        I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		//wait until  RXNE becomes 1
    	while( !I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

		//generate STOP condition
    	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

    	*pRxBuffer = pI2CHandle->pI2Cx->DR;
    	Len--;
    }
    else
    {
		// clear the ADDR flag according to its software sequence
	    I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

    	while(Len > 0)
    	{
    		//wait until  RXNE becomes 1
        	while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_RXNE )));

    	   	if(Len == 2)
    	   	{
    			//Disable Acking
    	   		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);
    	   	}
   			//generate STOP condition
    	    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

    	    *pRxBuffer = pI2CHandle->pI2Cx->DR;
    	    pRxBuffer++;
    	    Len--;

    	}
    }
	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}




/*
 * stm32f407xx_SPI_driver.c
 *
 *  Created on: Oct 12, 2021
 *      Author: Abdel
 */

#include "stm32f407xx_SPI_driver.h"

static void  SPI_TXE_Interrupt_Handle(SPI_Handle_t*pSPIHandle);
static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t*pSPIHandle);
static void SPI_OVR_Interrupt_Handle(SPI_Handle_t*pSPIHandle);


/**
  *  Peripheral Clock setup
  */

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - Base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1 )
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2 )
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3 )
		{
			SPI3_PCLK_EN();
		}

	}
	else
	{
		if(pSPIx == SPI1 )
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2 )
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3 )
		{
			SPI3_PCLK_DI();
		}
	}

}


/**
  *  Init and DeInit
  */

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initialize the given pin modes
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_Init(SPI_Handle_t*pSPIHandle)
{

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t TempReg=0;

	TempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	TempReg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	TempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	TempReg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	TempReg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	TempReg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD )
	{
		TempReg &= ~(1<< 15);

	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		TempReg |= (1<< SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		TempReg &= ~(1<< SPI_CR1_BIDIMODE);
		TempReg |= (1<< SPI_CR1_RXONLY);
	}

	pSPIHandle->pSPIx->CR1 = TempReg;




}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function Reset the given pin
 *
 * @param[in]         - Base address of the SPI peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1 )
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2 )
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3 )
	{
		SPI3_REG_RESET();
	}


}

/**
  *  FlagStatus
  */

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
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
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName )
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}



}
/*
 * Data Send and Receive
 */

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - This is blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{
	while( Len > 0)
	{
		while( SPI_GetFlagStatus(pSPIx , SPI_TXE_FLAG)  == FLAG_RESET  );

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			(uint16_t*)pTxBuffer++;
			Len--;
			Len--;
		}
		else
		{
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
			Len--;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while( Len > 0)
	{
		while( SPI_GetFlagStatus(pSPIx , SPI_RXNE_FLAG)  == FLAG_RESET  );

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			(uint16_t*)pRxBuffer++;
			Len--;
			Len--;
		}
		else
		{
			*(pRxBuffer) = pSPIx->DR;
			pRxBuffer++;
			Len--;
		}
	}
}

uint8_t SPI_SendData_IT(SPI_Handle_t*pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t State = pSPIHandle->TxState;

	if(State != SPI_BUSY_IN_TX)
	{
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		pSPIHandle->pSPIx->CR2 |= 1 << SPI_CR2_TXEIE;

	}

	return State;
}
uint8_t SPI_ReceiveData_IT(SPI_Handle_t*pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t State = pSPIHandle->RxState;

	if(State != SPI_BUSY_IN_RX)
	{
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		pSPIHandle->pSPIx->CR2 |= 1 << SPI_CR2_RXNEIE;

	}

	return State;
}


/*********************************************************************
 * @fn      		  - SPI_SSIConfig
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
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= 1 << SPI_CR1_SSI;
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/*********************************************************************
 * @fn      		  - SPI_PeriphControl
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
void SPI_PeriphControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= 1 << SPI_CR1_SPE;
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
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
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= 1 << SPI_CR2_SSOE;
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}




/**
  *  IRQ configuration and ISR handling
  */

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -
 *
 * @Note              - none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31 )
		{
			*(NVIC_ISER0) |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <=  63 )
		{
			*(NVIC_ISER1) |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber <=  95 )
		{
			*(NVIC_ISER2) |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31 )
		{
			*(NVIC_ICER0) |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <=  63 )
		{
			*(NVIC_ICER1) |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber <=  95 )
		{
			*(NVIC_ICER2) |= (1 << (IRQNumber % 64));
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4 ;
	uint8_t shift_amount = (8* iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}



/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - none
 */
void SPI_IRQHandling(SPI_Handle_t*pSPIHandle)
{
	uint8_t Temp1,Temp2;

	Temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	Temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);
	if( Temp1 && Temp2 )
	{
		SPI_TXE_Interrupt_Handle(pSPIHandle);
	}

	Temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	Temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if( Temp1 && Temp2 )
	{
		SPI_RXNE_Interrupt_Handle(pSPIHandle);
	}

	Temp1 = pSPIHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	Temp2 = pSPIHandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);
	if( Temp1 && Temp2 )
	{
		SPI_OVR_Interrupt_Handle(pSPIHandle);
	}
}




static void  SPI_TXE_Interrupt_Handle(SPI_Handle_t*pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		(uint16_t*)pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
	}
	else
	{
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen--;
	}
	if(! pSPIHandle->TxLen )
	{
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}

static void  SPI_RXNE_Interrupt_Handle(SPI_Handle_t*pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		(uint16_t*)pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
	}
	else
	{
		*(pSPIHandle->pRxBuffer) =pSPIHandle-> pSPIx->DR;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
	}
	if(! pSPIHandle->RxLen )
	{
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}

static void SPI_OVR_Interrupt_Handle(SPI_Handle_t*pSPIHandle)
{
	uint8_t Temp;

	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		Temp = pSPIHandle->pSPIx->DR;
		Temp = pSPIHandle->pSPIx->SR;
	}
	(void) Temp;
	SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_OVR_ERR);

}

void SPI_CloseTransmission(SPI_Handle_t*pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t*pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

__weak void SPI_ApplicationEventCallBack(SPI_Handle_t*pSPIHandle,uint8_t AppEvent)
{
	//This is a weak implementation . the user application may override this function.
}







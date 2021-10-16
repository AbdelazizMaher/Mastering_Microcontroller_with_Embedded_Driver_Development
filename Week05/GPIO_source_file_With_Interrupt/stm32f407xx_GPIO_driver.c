/*
 * stm32f4xx_GPIO_driver.c
 *
 *  Created on: Sep 30, 2021
 *      Author: Abdel
 */


#include "stm32f407xx_GPIO_driver.h"






/**
  *  Peripheral Clock setup
  */

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN();
			}
		else if (pGPIOx == GPIOB)
		        {
			        GPIOB_PCLK_EN();
		        }
		else if (pGPIOx == GPIOC)
				{
					GPIOC_PCLK_EN();
				}
		else if (pGPIOx == GPIOD)
				{
					GPIOD_PCLK_EN();
				}
		else if (pGPIOx == GPIOE)
				{
					GPIOE_PCLK_EN();
				}
		else if (pGPIOx == GPIOF)
				{
					GPIOF_PCLK_EN();
				}
		else if (pGPIOx == GPIOG)
				{
					GPIOG_PCLK_EN();
				}
		else if (pGPIOx == GPIOH)
				{
					GPIOH_PCLK_EN();
				}
		else if (pGPIOx == GPIOI)
				{
					GPIOI_PCLK_EN();
				}
	}
	else
	{
		if (pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}
		else if (pGPIOx == GPIOB)
			    {
					GPIOB_PCLK_DI();
			    }
		else if (pGPIOx == GPIOC)
			    {
					GPIOC_PCLK_DI();
			    }
		else if (pGPIOx == GPIOD)
			    {
					GPIOD_PCLK_DI();
			    }
		else if (pGPIOx == GPIOE)
			    {
					GPIOE_PCLK_DI();
			    }
		else if (pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}
		else if (pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}
		else if (pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}
		else if (pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}
	}

}


/**
  *  Init and DeInit
  */

/*********************************************************************
 * @fn      		  - GPIO_Init
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
void GPIO_Init(GPIO_Handle_t*pGPIOHandle)
{
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);

	uint32_t temp=0; //temp. register

	//1 . configure the mode of gpio pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp =( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			EXTI->RTSR &= ~(1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			EXTI->FTSR &= ~(1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->FTSR |= (1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			EXTI->RTSR |= (1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		}

		// configure the GPIO port selection in SYSCFG_EXTICR
		SYSCFG_PCLK_EN();
		uint8_t Temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t Temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG->EXTICR[Temp1] |= (PortCode<<(4*Temp2));
		// enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	}

	temp=0;

	//2. configure the speed

	temp =( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp=0;
	//3. configure the pupd settings

	temp =( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp=0;
	//4. configure the optype
	temp =( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp=0;
	//5. configure the alt functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN )
	{
		uint8_t temp1= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8 ;
		uint8_t temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8 ;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4*temp2) );
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_AltFunMode << (4*temp2));
	}


}


/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function Reset the given pin
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
		{
		    GPIOA_REG_RESET();
		}
	else if (pGPIOx == GPIOB)
		{
		    GPIOB_REG_RESET();
		}
	else if (pGPIOx == GPIOC)
		{
		    GPIOC_REG_RESET();
		}
	else if (pGPIOx == GPIOD)
		{
		    GPIOD_REG_RESET();
		}
	else if (pGPIOx == GPIOE)
		{
		    GPIOE_REG_RESET();
		}
	else if (pGPIOx == GPIOF)
		{
		    GPIOF_REG_RESET();
		}
	else if (pGPIOx == GPIOG)
		{
		    GPIOG_REG_RESET();
		}
	else if (pGPIOx == GPIOH)
		{
		    GPIOH_REG_RESET();
		}
	else if (pGPIOx == GPIOI)
		{
		    GPIOI_REG_RESET();
		}

}



/**
  *  Read and Write Data
  */

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads the input from the pin
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - Pin number of the chosen port
 * @param[in]         -
 *
 * @return            - the 8-bit value from the pin
 *
 * @Note              - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t Value;
	Value= (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1 );
	return Value;
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads the input from the port
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - the 16-bit value from the port
 *
 * @Note              - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t Value;
	Value= (uint16_t)( pGPIOx->IDR );
	return Value;

}


/*********************************************************************
 * @fn      		  - GPIO_WriteIntoOutputPin
 *
 * @brief             - This function write to the pin
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - Pin number
 * @param[in]         - Value (0 or 1)
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteIntoOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}


/*********************************************************************
 * @fn      		  - GPIO_WriteIntoOutputPort
 *
 * @brief             - This function write to the port
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - Value (0 or 1)
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteIntoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR =  Value;

}



/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputpin
 *
 * @brief             - This function toggles the pin
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - Pin number
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_ToggleOutputpin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);

}



/**
  *  IRQ configuration and ISR handling
  */

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4 ;
	uint8_t shift_amount = (8* iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}


/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
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
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1 << PinNumber);
	}
}


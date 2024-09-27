/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Sep 16, 2024
 *      Author: hoangngo
 */
#include "stm32f103xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        } else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        } else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        } else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
    } else
    {
        // TODO
    }
}

static uint8_t Get_CRLH_Position(uint16_t PinNumber){
    uint8_t u8_l_RetVal = 0;
    if (PinNumber < GPIO_PIN_NO_8){
        u8_l_RetVal = PinNumber * 4;
    } else
    {
        u8_l_RetVal = (PinNumber - 8) * 4;
    }
    return u8_l_RetVal;
}

ErrState GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    ErrState e_l_ErrState = E_NOK;
    if (pGPIOHandle != NULL)
    {
        uint8_t u8_l_PinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
        uint8_t u8_l_PinMode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
        // Get target register
        __vo uint32_t *p_u32_l_ConfigReg = NULL;
        //Enable Peripheral clock
        GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
        if(u8_l_PinNumber < GPIO_PIN_NO_8)
        {
            p_u32_l_ConfigReg = &pGPIOHandle->pGPIOx->CRL;

        } else
        {
            p_u32_l_ConfigReg = &pGPIOHandle->pGPIOx->CRH;
        }
        // Clear MODE, CNF
        *p_u32_l_ConfigReg &= ~(0xF << Get_CRLH_Position(u8_l_PinNumber));
        //Check PIN for input or output
        if((u8_l_PinMode == GPIO_MODE_OUTPUT_PP) || (u8_l_PinMode == GPIO_MODE_OUTPUT_OD) ||
                (u8_l_PinMode == GPIO_MODE_OUTPUT_AF_PP) || (u8_l_PinMode == GPIO_MODE_OUTPUT_AF_OD))
        {
            //Set MODE and CFN
            // PinMode - 4 because macro value define
            *p_u32_l_ConfigReg |= ((u8_l_PinMode - 4) << 2 | (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed & 0x0F)) << Get_CRLH_Position(u8_l_PinNumber);
            e_l_ErrState = E_OK;
        } else
        {
            if(u8_l_PinMode == GPIO_MODE_INPUT_FLO || u8_l_PinMode == GPIO_MODE_ANALOG)
            {
                *p_u32_l_ConfigReg |= (((u8_l_PinMode << 2) | 0x0) & 0x0F) << Get_CRLH_Position(u8_l_PinNumber);
                e_l_ErrState = E_OK;
            } else if(u8_l_PinMode == GPIO_MODE_INPUT_AF)
            {
                *p_u32_l_ConfigReg |= (((GPIO_MODE_INPUT_FLO << 2) | 0x0) & 0x0F) << Get_CRLH_Position(u8_l_PinNumber);
                e_l_ErrState = E_OK;
            } else // Pull-up or Pull-down input
            {
                *p_u32_l_ConfigReg |= (((GPIO_MODE_INPUT_PU << 2) | 0x0) & 0x0F) << Get_CRLH_Position(u8_l_PinNumber);
                if(u8_l_PinMode == GPIO_MODE_INPUT_PU)
                {
                    pGPIOHandle->pGPIOx->ODR |= (1 << u8_l_PinNumber);
                } else
                {
                    pGPIOHandle->pGPIOx->ODR &= ~(1 << u8_l_PinNumber);
                }
                e_l_ErrState = E_OK;
            }
            e_l_ErrState = E_OK;
        }

    } else
    {
        e_l_ErrState = E_NULL;
    }
    return e_l_ErrState;
}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
    //TODO
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t retVal;
    retVal = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return retVal;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t retVal;
    retVal = (uint16_t)pGPIOx->IDR;
    return retVal;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if(Value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (1 << PinNumber);
    } else
    {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}










/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Sep 13, 2024
 *      Author: hoangngo
 */

#ifndef DRIVERS_INC_STM32F103XX_GPIO_DRIVER_H_
#define DRIVERS_INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103xx.h"

/*
 * Configuration structure for a GPIO pin
 */
typedef struct {
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
} GPIO_PinConfig_t;

/*
 * Handle structure for a GPIO pin
 */
typedef struct {
    GPIO_RegDef_t *pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*
 * GPIO pin possible modes
 */

#define GPIO_SPEED_10MHZ         1
#define GPIO_SPEED_2MHZ          2
#define GPIO_SPEED_50MHZ         3

#define GPIO_MODE_ANALOG                0x00000000UL    // Select Analog mode
#define GPIO_MODE_INPUT_FLO             0x00000001UL    // Select Floating input (reset state)
#define GPIO_MODE_INPUT_PU              0x00000002UL    // Select Input with pull-up
#define GPIO_MODE_INPUT_PD              0x00000003UL    // Select Input with pull-down
#define GPIO_MODE_OUTPUT_PP             0x00000004UL    // Select General purpose output push-pull
#define GPIO_MODE_OUTPUT_OD             0x00000005UL    // Select General purpose output Open-drain
#define GPIO_MODE_OUTPUT_AF_PP          0x00000006UL    // Select Alternate function output Push-pull
#define GPIO_MODE_OUTPUT_AF_OD          0x00000007UL    // Select Alternate function output Open-drain
#define GPIO_MODE_INPUT_AF              0x00000008UL    // Alternate function input

#define GPIO_MODE_IT_FT                 0x00000009UL
#define GPIO_MODE_IT_RT                 0x0000000AUL
#define GPIO_MODE_IT_FRT                0x0000000BUL

/*
 * GPIO PinNumber
 */
#define GPIO_PIN_NO_0               0
#define GPIO_PIN_NO_1               1
#define GPIO_PIN_NO_2               2
#define GPIO_PIN_NO_3               3
#define GPIO_PIN_NO_4               4
#define GPIO_PIN_NO_5               5
#define GPIO_PIN_NO_6               6
#define GPIO_PIN_NO_7               7
#define GPIO_PIN_NO_8               8
#define GPIO_PIN_NO_9               9
#define GPIO_PIN_NO_10              10
#define GPIO_PIN_NO_11              11
#define GPIO_PIN_NO_12              12
#define GPIO_PIN_NO_13              13
#define GPIO_PIN_NO_14              14
#define GPIO_PIN_NO_15              15




/*
 * APIs supported by this driver
 */

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
ErrState GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);





























#endif /* DRIVERS_INC_STM32F103XX_GPIO_DRIVER_H_ */

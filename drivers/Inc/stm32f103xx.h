/*
 * stm32f103xx.h
 *
 *  Created on: Sep 12, 2024
 *      Author: hoangngo
 */

#ifndef DRIVERS_INC_STM32F103XX_H_
#define DRIVERS_INC_STM32F103XX_H_
#include <stdint.h>
/*
 * Generic macros
 */
#define ENABLE                          1
#define DISABLE                         0
#define SET                             ENABLE
#define RESET                           DISABLE
#define GPIO_PIN_SET                    SET
#define GPIO_PIN_RESET                  RESET

#define NULL                            0


#define __vo                            volatile

/*
 * Base address of FLASH and SRAM memories
 */

#define FLASH_BASEADDR                  0x08000000U
#define SRAM_BASEADDR                   0x40000000U
#define ROM_BASEADDR                    0x1FFFF000U

#define PERIPH_BASEADDR                 0x40000000U
#define APB1PERIPH_BASEADDR             PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR             0x40010000U
#define AHBPERIPH_BASEADDR              0x40018000U

/*
 * Define base address for APB1 bus
 */
#define TIM2_BASEADDR                   (APB1PERIPH_BASEADDR + 0x0000U)

//TODO: Add more peripherals when need to use
#define USART2_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR                 (APB1PERIPH_BASEADDR + 0x4800U)

/*
 * Define base address for APB2 bus
 */
#define AFIO_BASEADDR                   (APB2PERIPH_BASEADDR + 0x0000U)
#define EXTI_BASEADDR                   (APB2PERIPH_BASEADDR + 0x0400U)
#define GPIOA_BASEADDR                  (APB2PERIPH_BASEADDR + 0x0800U)
#define GPIOB_BASEADDR                  (APB2PERIPH_BASEADDR + 0x0C00U)
#define GPIOC_BASEADDR                  (APB2PERIPH_BASEADDR + 0x1000U)
#define GPIOD_BASEADDR                  (APB2PERIPH_BASEADDR + 0x1400U)
#define GPIOE_BASEADDR                  (APB2PERIPH_BASEADDR + 0x1800U)

#define SPI1_BASEADDR                   (APB2PERIPH_BASEADDR + 0x3000U)
#define SPI2_BASEADDR                   (APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR                   (APB1PERIPH_BASEADDR + 0x3C00U)

/*
 * Define base address for AHB bus
 */
#define RCC_BASEADDR                    (AHBPERIPH_BASEADDR + 0x9000U)


typedef struct {
    __vo uint32_t CRL;
    __vo uint32_t CRH;
    __vo uint32_t IDR;
    __vo uint32_t ODR;
    __vo uint32_t BSRR;
    __vo uint32_t BRR;
    __vo uint32_t LCKR;
} GPIO_RegDef_t;


typedef struct {
    __vo uint32_t CR;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t APB2RSTR;
    __vo uint32_t APB1RSTR;
    __vo uint32_t AHBENR;
    __vo uint32_t APB2ENR;
    __vo uint32_t APB1ENR;
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
    __vo uint32_t AHBSTR;
    __vo uint32_t CFGR2;
} RCC_RegDef_t;

typedef enum {
    E_NOK,
    E_OK,
    E_NULL
}ErrState;

typedef struct {
    __vo uint32_t IMR;
    __vo uint32_t EMR;
    __vo uint32_t RTSR;
    __vo uint32_t FTSR;
    __vo uint32_t SWIER;
    __vo uint32_t PR;
}EXTI_RegDef_t;


typedef struct {
    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t SR;
    __vo uint32_t DR;
    __vo uint32_t CRCPR;
    __vo uint32_t RXCRCR;
    __vo uint32_t TXCRCR;
    __vo uint32_t I2SCFGR;
    __vo uint32_t I2SPR;
}SPI_RegDef_t;

/*
 * Define peripherals GPIO
 */
#define GPIOA                           ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                           ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                           ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                           ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                           ((GPIO_RegDef_t*)GPIOE_BASEADDR)


#define RCC                             ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI                            ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SPI1                            ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2                            ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                            ((SPI_RegDef_t*)SPI3_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()                   (RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()                   (RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()                   (RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()                   (RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()                   (RCC->APB2ENR |= (1 << 6))


/*
 * Clock disable macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()                   (RCC->ABP2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()                   (RCC->ABP2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()                   (RCC->ABP2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()                   (RCC->ABP2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()                   (RCC->ABP2ENR &= ~(1 << 6))


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()                  do {(RCC->ABP2RSTR |= (1 << 2)); (RCC->ABP2RSTR &= ~(1 << 2));} while(0)
#define GPIOB_REG_RESET()                  do {(RCC->ABP2RSTR |= (1 << 3)); (RCC->ABP2RSTR &= ~(1 << 3));} while(0)
#define GPIOC_REG_RESET()                  do {(RCC->ABP2RSTR |= (1 << 4)); (RCC->ABP2RSTR &= ~(1 << 4));} while(0)
#define GPIOD_REG_RESET()                  do {(RCC->ABP2RSTR |= (1 << 5)); (RCC->ABP2RSTR &= ~(1 << 5));} while(0)
#define GPIOE_REG_RESET()                  do {(RCC->ABP2RSTR |= (1 << 6)); (RCC->ABP2RSTR &= ~(1 << 6));} while(0)

#include "stm32f103xx_gpio_driver.h"

#endif /* DRIVERS_INC_STM32F103XX_H_ */

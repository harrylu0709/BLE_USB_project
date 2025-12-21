/*
 * stm3f407xx.h
 *
 *  Created on: Jan 29, 2019
 *      Author: admin
 */

#ifndef INC_STM3F407XX_H_
#define INC_STM3F407XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))
#define ARRAY_LENGTH(a)  (sizeof(a)/sizeof(a[0]))


/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR						0x08000000U   		/*!<explain this macro briefly here  */
#define SRAM1_BASEADDR						0x20000000U  		/*!<explain this macro briefly here  */
#define SRAM2_BASEADDR						0x2001C000U 		/*!<explain this macro briefly here  */
#define ROM_BASEADDR						0x1FFF0000U
#define SRAM 								SRAM1_BASEADDR


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR 						0x40000000U
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR						0x40010000U
#define AHB1PERIPH_BASEADDR						0x40020000U
#define AHB2PERIPH_BASEADDR						0x50000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * TODO : Complete for all other peripherals
 */
//RF 2.3 memory map
#define GPIOA_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                   (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR 					 (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR                     (AHB1PERIPH_BASEADDR + 0x3800)
/*
 * Base addresses of peripherals which are hanging on APB1 bus
 * TODO : Complete for all other peripherals
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR						(APB1PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR						(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR						(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASEADDR + 0x5000)

#define TIM5_BASEADDR						(APB1PERIPH_BASEADDR + 0x0C00)
#define TIM2_BASEADDR						(APB1PERIPH_BASEADDR + 0x0000)
/*
 * Base addresses of peripherals which are hanging on APB2 bus
 * TODO : Complete for all other peripherals
 */
#define EXTI_BASEADDR						(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR						(APB2PERIPH_BASEADDR + 0x3000)
#define SYSCFG_BASEADDR        				(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR						(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASEADDR + 0x1400)





/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

typedef struct
{
	__vo uint32_t MODER;                        /*!< GPIO port mode register,                    	Address offset: 0x00      */
	__vo uint32_t OTYPER;                       /*!< TODO,     										Address offset: 0x04      */
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];					 /*!< AFR[0] : GPIO alternate function low register, AF[1] : GPIO alternate function high register    		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;



/*
 * peripheral register definition structure for RCC_
 */
typedef struct
{
  __vo uint32_t CR;            /*!< TODO,     										Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< TODO,     										Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< TODO,     										Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< TODO,     										Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< TODO,     										Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< TODO,     										Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< TODO,     										Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
  __vo uint32_t APB1RSTR;      /*!< TODO,     										Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< TODO,     										Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
  __vo uint32_t AHB1ENR;       /*!< TODO,     										Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< TODO,     										Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< TODO,     										Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
  __vo uint32_t APB1ENR;       /*!< TODO,     										Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< TODO,     										Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
  __vo uint32_t AHB1LPENR;     /*!< TODO,     										Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< TODO,     										Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< TODO,     										Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
  __vo uint32_t APB1LPENR;     /*!< TODO,     										Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< RTODO,     										Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
  __vo uint32_t BDCR;          /*!< TODO,     										Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< TODO,     										Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
  __vo uint32_t SSCGR;         /*!< TODO,     										Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< TODO,     										Address offset: 0x84 */
  __vo uint32_t PLLSAICFGR;    /*!< TODO,     										Address offset: 0x88 */
  __vo uint32_t DCKCFGR;       /*!< TODO,     										Address offset: 0x8C */
  __vo uint32_t CKGATENR;      /*!< TODO,     										Address offset: 0x90 */
  __vo uint32_t DCKCFGR2;      /*!< TODO,     										Address offset: 0x94 */

} RCC_RegDef_t;



/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Give a short description,          	  	    Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< TODO,                						Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< TODO,  									     Address offset: 0x08 */
	__vo uint32_t FTSR;   /*!< TODO, 										Address offset: 0x0C */
	__vo uint32_t SWIER;  /*!< TODO,  									   Address offset: 0x10 */
	__vo uint32_t PR;     /*!< TODO,                   					   Address offset: 0x14 */

}EXTI_RegDef_t;


/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CRCPR;      /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /*!< TODO,     										Address offset: 0x1C */
	__vo uint32_t I2SPR;      /*!< TODO,     										Address offset: 0x20 */
} SPI_RegDef_t;


/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       /*!< Give a short description,                    Address offset: 0x00      */
	__vo uint32_t PMC;          /*!< TODO,     									  Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    /*!< TODO , 									  Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2];  /*!< TODO          							  Reserved, 0x18-0x1C    	*/
	__vo uint32_t CMPCR;        /*!< TODO         								  Address offset: 0x20      */
	uint32_t      RESERVED2[2];  /*!<                                             Reserved, 0x24-0x28 	    */
	__vo uint32_t CFGR;         /*!< TODO                                         Address offset: 0x2C   	*/
} SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
} USART_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CR2;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t SMCR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t DIER;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t SR;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t EGR;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t CCMR1;       /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t CCMR2;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CCER;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t CNT;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t PSC;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t ARR;        /*!< TODO,     										Address offset: 0x10 */
	uint32_t RESERVED1;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t CCR1;       /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t CCR2;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t CCR3;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t CCR4;        /*!< TODO,     										Address offset: 0x08 */
	uint32_t RESERVED2;         /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t DCR;       /*!< TODO,     										Address offset: 0x18 */
	__vo uint32_t DMAR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t OR2;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t OR5;        /*!< TODO,     										Address offset: 0x08 */
} TIM_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIO_A  				((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIO_B  				((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIO_C  				((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIO_D  				((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIO_E  				((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIO_F  				((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIO_G  				((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIO_H  				((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIO_I  				((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC_ 				((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI_				((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG_				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI_1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI_2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI_3  				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C_1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C_2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C_3  				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART_1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART_2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART_3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART_4  			((USART_RegDef_t*)UART4_BASEADDR)
#define UART_5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART_6  			((USART_RegDef_t*)USART6_BASEADDR)
#define TIM_2  				((TIM_RegDef_t*)TIM2_BASEADDR)
#define TIM_5  				((TIM_RegDef_t*)TIM5_BASEADDR)
/*
 * Clock Enable Macros for GPIOx peripherals
 * RCC_ registers 6.3
 */

#define GPIOA_PCLK_EN()    	(RCC_->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC_->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC_->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC_->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC_->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC_->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC_->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC_->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC_->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC_->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC_->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC_->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx peripheralsbu
 */
#define SPI1_PCLK_EN() (RCC_->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC_->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC_->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC_->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC_->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC_->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC_->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  (RCC_->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC_->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC_->APB1ENR |= (1 << 5))


#define TIM2_PCLK_EN() (RCC_->APB1ENR |= (1 << 0))
#define TIM5_PCLK_EN() (RCC_->APB1ENR |= (1 << 3))

#define TIM2_PCLK_DI() (RCC_->APB1ENR &= ~ (1 << 0))
#define TIM5_PCLK_DI() (RCC_->APB1ENR &= ~ (1 << 3))
/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC_->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI()    	(RCC_->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC_->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC_->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC_->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC_->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC_->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC_->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC_->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC_->AHB1ENR &= ~(1 << 8))


#define I2C1_PCLK_DI() (RCC_->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC_->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC_->APB1ENR &= ~(1 << 23))
/*
 * Clock Disable Macros for SPIx peripherals
 */

/*
 * Clock Disable Macros for USARTx peripherals
 */


/*
 * Clock Disable Macros for SYSCFG peripheral
 */


/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC_->AHB1RSTR |= (1 << 0)); (RCC_->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC_->AHB1RSTR |= (1 << 1)); (RCC_->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC_->AHB1RSTR |= (1 << 2)); (RCC_->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC_->AHB1RSTR |= (1 << 3)); (RCC_->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC_->AHB1RSTR |= (1 << 4)); (RCC_->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC_->AHB1RSTR |= (1 << 5)); (RCC_->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC_->AHB1RSTR |= (1 << 6)); (RCC_->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC_->AHB1RSTR |= (1 << 7)); (RCC_->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC_->AHB1RSTR |= (1 << 8)); (RCC_->AHB1RSTR &= ~(1 << 8)); }while(0)


/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIO_A)?0:\
										(x == GPIO_B)?1:\
										(x == GPIO_C)?2:\
										(x == GPIO_D)?3:\
								        (x == GPIO_E)?4:\
								        (x == GPIO_F)?5:\
								        (x == GPIO_G)?6:\
								        (x == GPIO_H)?7: \
								        (x == GPIO_I)?8:0)


/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 * reference manual 12.2 table 63 Acronym - position
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84

#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_I2C2_EV     33
#define IRQ_NO_I2C2_ER     34
#define IRQ_NO_I2C3_EV     79
#define IRQ_NO_I2C3_ER     80
#define IRQ_NO_USART1	   37
#define IRQ_NO_USART2	   38
#define IRQ_NO_TIM2		   28 //APB1 45MHz
#define IRQ_NO_TIM5		   50 //APB1 45MHz
//#define IRQ_NO_USART3	    39
//#define IRQ_NO_UART4	    52
//#define IRQ_NO_UART5	    53
//#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0     0
#define NVIC_IRQ_PRI1     1
#define NVIC_IRQ_PRI2     2
#define NVIC_IRQ_PRI3     3
#define NVIC_IRQ_PRI4     4
#define NVIC_IRQ_PRI5     5
#define NVIC_IRQ_PRI6     6
#define NVIC_IRQ_PRI7     7
#define NVIC_IRQ_PRI8     8
#define NVIC_IRQ_PRI9     9
#define NVIC_IRQ_PRI10    10
#define NVIC_IRQ_PRI11    11
#define NVIC_IRQ_PRI12    12
#define NVIC_IRQ_PRI13    13
#define NVIC_IRQ_PRI14    14
#define NVIC_IRQ_PRI15    15


//some generic macros

// #define ENABLE 				1
// #define DISABLE 			0
// #define SET 				ENABLE
// #define RESET 				DISABLE
#define GPIO_PIN_SET        1
#define GPIO_PIN_RESET      0

#define FLAG_SET 			1
#define FLAG_RESET         	0



/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA_DEF     				 0
#define SPI_CR1_CPOL_DEF      				 1
#define SPI_CR1_MSTR_DEF     				 2
#define SPI_CR1_BR_DEF   					 3
#define SPI_CR1_SPE_DEF     				 6
#define SPI_CR1_LSBFIRST_DEF   			 	 7
#define SPI_CR1_SSI_DEF     				 8
#define SPI_CR1_SSM_DEF      				 9
#define SPI_CR1_RXONLY_DEF      		 	10
#define SPI_CR1_DFF_DEF     			 	11
#define SPI_CR1_CRCNEXT_DEF   			 	12
#define SPI_CR1_CRCEN_DEF   			 	13
#define SPI_CR1_BIDIOE_DEF     			 	14
#define SPI_CR1_BIDIMODE_DEF      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN_DEF		 			0
#define SPI_CR2_TXDMAEN_DEF				 	1
#define SPI_CR2_SSOE_DEF				 	2
#define SPI_CR2_FRF_DEF						4
#define SPI_CR2_ERRIE_DEF					5
#define SPI_CR2_RXNEIE_DEF				 	6
#define SPI_CR2_TXEIE_DEF					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE_DEF						0
#define SPI_SR_TXE_DEF				 		1
#define SPI_SR_CHSIDE_DEF				 	2
#define SPI_SR_UDR_DEF					 	3
#define SPI_SR_CRCERR_DEF				 	4
#define SPI_SR_MODF_DEF					 	5
#define SPI_SR_OVR_DEF					 	6
#define SPI_SR_BSY_DEF					 	7
#define SPI_SR_FRE_DEF					 	8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE_DEF						0
#define I2C_CR1_NOSTRETCH_DEF  				7
#define I2C_CR1_START_DEF 					8
#define I2C_CR1_STOP_DEF  				 	9
#define I2C_CR1_ACK_DEF 				 	10
#define I2C_CR1_SWRST_DEF  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ_DEF				 	0
#define I2C_CR2_ITERREN_DEF				 	8
#define I2C_CR2_ITEVTEN_DEF				 	9
#define I2C_CR2_ITBUFEN_DEF 			    10 //buffer interrupt enable, txe or rxne

/*
 * Bit position definitions I2C_OAR1
 * own address register
 */
#define I2C_OAR1_ADD0_DEF    				 0
#define I2C_OAR1_ADD71_DEF 				 	 1
#define I2C_OAR1_ADD98_DEF  			 	 8
#define I2C_OAR1_ADDMODE_DEF   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB_DEF 					 	0
#define I2C_SR1_ADDR_DEF 				 	1
#define I2C_SR1_BTF_DEF 					2
#define I2C_SR1_ADD10_DEF 					3
#define I2C_SR1_STOPF_DEF 					4
#define I2C_SR1_RXNE_DEF 					6
#define I2C_SR1_TXE_DEF 					7
#define I2C_SR1_BERR_DEF 					8
#define I2C_SR1_ARLO_DEF 					9
#define I2C_SR1_AF_DEF 					 	10
#define I2C_SR1_OVR_DEF 					11
#define I2C_SR1_TIMEOUT_DEF 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL_DEF						0
#define I2C_SR2_BUSY_DEF 					1
#define I2C_SR2_TRA_DEF 					2
#define I2C_SR2_GENCALL_DEF 				4
#define I2C_SR2_DUALF_DEF 					7

/*
 * Bit position definitions I2C_CCR
 * clock control register
 */
#define I2C_CCR_CCR_DEF 					 0
#define I2C_CCR_DUTY_DEF 					14
#define I2C_CCR_FS_DEF  				 	15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK_DEF					0
#define USART_CR1_RWU_DEF 					1
#define USART_CR1_RE_DEF  					2
#define USART_CR1_TE_DEF 					3
#define USART_CR1_IDLEIE_DEF 				4
#define USART_CR1_RXNEIE_DEF  				5
#define USART_CR1_TCIE_DEF					6
#define USART_CR1_TXEIE_DEF					7
#define USART_CR1_PEIE_DEF 					8
#define USART_CR1_PS_DEF 					9
#define USART_CR1_PCE_DEF 					10
#define USART_CR1_WAKE_DEF  				11
#define USART_CR1_M_DEF 					12
#define USART_CR1_UE_DEF 					13
#define USART_CR1_OVER8_DEF  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD_DEF   				0
#define USART_CR2_LBDL_DEF   				5
#define USART_CR2_LBDIE_DEF  				6
#define USART_CR2_LBCL_DEF   				8
#define USART_CR2_CPHA_DEF   				9
#define USART_CR2_CPOL_DEF   				10
#define USART_CR2_STOP_DEF   				12
#define USART_CR2_LINEN_DEF   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE_DEF   				0
#define USART_CR3_IREN_DEF   				1
#define USART_CR3_IRLP_DEF  				2
#define USART_CR3_HDSEL_DEF   				3
#define USART_CR3_NACK_DEF   				4
#define USART_CR3_SCEN_DEF   				5
#define USART_CR3_DMAR_DEF  				6
#define USART_CR3_DMAT_DEF   				7
#define USART_CR3_RTSE_DEF   				8
#define USART_CR3_CTSE_DEF   				9
#define USART_CR3_CTSIE_DEF   				10
#define USART_CR3_ONEBIT_DEF   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PEE_DEF        			0
#define USART_SR_FE_DEF        				1
#define USART_SR_NE_DEF        				2
#define USART_SR_ORE_DEF       				3
#define USART_SR_IDLE_DEF       			4
#define USART_SR_RXNE_DEF        			5
#define USART_SR_TC_DEF        				6
#define USART_SR_TXE_DEF        			7
#define USART_SR_LBD_DEF        			8
#define USART_SR_CTS_DEF        			9

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_tim_driver.h"
#endif /* INC_STM3F407XX_H_ */

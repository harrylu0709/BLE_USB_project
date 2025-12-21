///*
// * stm32f407xx_gpio_driver.h
// *
// *  Created on: Jun 22, 2024
// *      Author: bo
// */
//
//#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
//#define INC_STM32F407XX_GPIO_DRIVER_H_
//
//#include "stm32f407.h"
//
////typedef struct
////{
////	volatile uint32_t MODER;
////	volatile uint32_t OTYPER;
////	volatile uint32_t OSPEEDR;
////	volatile uint32_t PUPDR;
////	volatile uint32_t IDR;
////	volatile uint32_t ODR;
////	volatile uint32_t BSRR;
////	volatile uint32_t LCKR;
////	volatile uint32_t ARF[2]; //ARF[0] low register ARF[1] high register
////}GPIO_RegDef_t;
//
//typedef struct
//{
//	uint8_t GPIO_PinNumber;			//from @GPIO_PIN_NUMBERS
//	uint8_t GPIO_PinMode;			//from @GPIO_PIN_MODES
//	uint8_t GPIO_PinSpeed;			//from @GPIO_PIN_SPEED
//	uint8_t GPIO_PinPuPdControl;    //from @GPIO_PIN_PUPD
//	uint8_t GPIO_PinOPType;			//from @GPIO_OP_TYPE
//	uint8_t GPIO_PinAltFunMode;
//}GPIO_PinConfig_t;
//
//typedef struct
//{
//	GPIO_RegDef_t *pGPIOx;                    //This hold base address of GPIO port
//	GPIO_PinConfig_t GPIO_PinConfig;          //GPIO pin configuration settings
//}GPIO_Handle_t;
//
//
///*
// * @GPIO_PIN_NUMBERS
// * GPIO pin numbers
// */
//#define GPIO_PIN_NO_0  				0
//#define GPIO_PIN_NO_1  				1
//#define GPIO_PIN_NO_2  				2
//#define GPIO_PIN_NO_3  				3
//#define GPIO_PIN_NO_4  				4
//#define GPIO_PIN_NO_5  				5
//#define GPIO_PIN_NO_6  				6
//#define GPIO_PIN_NO_7  				7
//#define GPIO_PIN_NO_8  				8
//#define GPIO_PIN_NO_9  				9
//#define GPIO_PIN_NO_10  			10
//#define GPIO_PIN_NO_11 				11
//#define GPIO_PIN_NO_12  			12
//#define GPIO_PIN_NO_13 				13
//#define GPIO_PIN_NO_14 				14
//#define GPIO_PIN_NO_15 				15
//
///*
// * @GPIO_PIN_MODES
// * GPIO pin possible modes
// */
//#define GPIO_MODE_IN 		0
//#define GPIO_MODE_OUT 		1
//#define GPIO_MODE_ALTFN 	2
//#define GPIO_MODE_ANALOG 	3
//#define GPIO_MODE_IT_FT     4
//#define GPIO_MODE_IT_RT     5
//#define GPIO_MODE_IT_RFT    6
//
///*
// * @GPIO_OP_TYPE
// * GPIO pin possible output type
// */
//#define GPIO_OP_TYPE_PP   0
//#define GPIO_OP_TYPE_OD   1
//
//
///*
// * @GPIO_PIN_SPEED
// * GPIO pin possible output speeds
// */
//#define GPIO_SPEED_LOW			0
//#define GPIO_SPEED_MEDIUM		1
//#define GPIO_SPEED_FAST			2
//#define GPOI_SPEED_HIGH			3
//
//
///*@GPIO_PIN_PUPD
// * GPIO pin pull up AND pull down configuration macros
// */
//#define GPIO_NO_PUPD   		0
//#define GPIO_PIN_PU			1
//#define GPIO_PIN_PD			2
////APIs supported by this driver
//
////Peripheral Clock Setup
//void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
//
//// Init and De-init
//void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
//void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);
//
////Data read and write
//uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
//uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
//void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
//void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
//void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
//
////IRQ configuration and handler
//void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
//void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
//void GPIO_IRQHandling(uint8_t PinNumber);
//
//#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jan 29, 2019
 *      Author: admin
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407.h"



/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;			/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;			/*!< possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;       		/*!< This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t GPIO_PinConfig;   /*!< This holds GPIO pin configuration settings >*/

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6


/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP   0
#define GPIO_OP_TYPE_OD   1


/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3


/*
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

typedef struct
{
  uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins_define */

  uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */

  uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */

  uint32_t Speed;     /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define */

  uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins. 
                            This parameter can be a value of @ref GPIO_Alternate_function_selection */
}GPIO_InitTypeDef;

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#define GPIO_MODE_Pos                           0U
#define GPIO_MODE                               (0x3UL << GPIO_MODE_Pos)
#define MODE_INPUT                              (0x0UL << GPIO_MODE_Pos)
#define MODE_OUTPUT                             (0x1UL << GPIO_MODE_Pos)
#define MODE_AF                                 (0x2UL << GPIO_MODE_Pos)
#define MODE_ANALOG                             (0x3UL << GPIO_MODE_Pos)


#define EXTI_MODE_Pos                           16U
#define EXTI_MODE                               (0x3UL << EXTI_MODE_Pos)
#define EXTI_IT                                 (0x1UL << EXTI_MODE_Pos)
#define EXTI_EVT                                (0x2UL << EXTI_MODE_Pos)

#define TRIGGER_MODE_Pos                         20U
#define TRIGGER_MODE                            (0x7UL << TRIGGER_MODE_Pos)
#define TRIGGER_RISING                          (0x1UL << TRIGGER_MODE_Pos)
#define TRIGGER_FALLING                         (0x2UL << TRIGGER_MODE_Pos)


#define  GPIO_MODE_INPUT                        MODE_INPUT    

#define OUTPUT_TYPE_Pos                         4U
#define OUTPUT_TYPE                             (0x1UL << OUTPUT_TYPE_Pos)
#define OUTPUT_PP                               (0x0UL << OUTPUT_TYPE_Pos)
#define  GPIO_NOPULL        0x00000000U
#define  GPIO_MODE_AF_PP                        (MODE_AF | OUTPUT_PP)                                       /*!< Alternate Function Push Pull Mode     */
#define  GPIO_SPEED_FREQ_LOW         0x00000000U
#define GPIO_OSPEEDR_OSPEED0_Pos         (0U)                                  
#define GPIO_OSPEEDR_OSPEED0_Msk         (0x3UL << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000003 */
#define GPIO_OSPEEDR_OSPEED0             GPIO_OSPEEDR_OSPEED0_Msk 
#define GPIO_OSPEEDER_OSPEEDR0           GPIO_OSPEEDR_OSPEED0

#define GPIO_OTYPER_OT0_Pos              (0U)                                  
#define GPIO_OTYPER_OT0_Msk              (0x1UL << GPIO_OTYPER_OT0_Pos)         /*!< 0x00000001 */
#define GPIO_OTYPER_OT0                  GPIO_OTYPER_OT0_Msk
#define GPIO_OTYPER_OT_0                 GPIO_OTYPER_OT0

#define GPIO_PUPDR_PUPD0_Pos             (0U)                                  
#define GPIO_PUPDR_PUPD0_Msk             (0x3UL << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000003 */
#define GPIO_PUPDR_PUPD0                 GPIO_PUPDR_PUPD0_Msk 
#define GPIO_PUPDR_PUPDR0                GPIO_PUPDR_PUPD0

#define GPIO_MODER_MODER0_Pos            (0U)                                  
#define GPIO_MODER_MODER0_Msk            (0x3UL << GPIO_MODER_MODER0_Pos)       /*!< 0x00000003 */
#define GPIO_MODER_MODER0                GPIO_MODER_MODER0_Msk 

#define GPIO_GET_INDEX(__GPIOx__)    (uint8_t)(((__GPIOx__) == (GPIO_A))? 0U :\
                                               ((__GPIOx__) == (GPIO_B))? 1U :\
                                               ((__GPIOx__) == (GPIO_C))? 2U :\
                                               ((__GPIOx__) == (GPIO_D))? 3U :\
                                               ((__GPIOx__) == (GPIO_E))? 4U :\
                                               ((__GPIOx__) == (GPIO_F))? 5U :\
                                               ((__GPIOx__) == (GPIO_G))? 6U :\
                                               ((__GPIOx__) == (GPIO_H))? 7U : 8U)



#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */

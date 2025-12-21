#ifndef INC_STM32F407XX_TIM_DRIVER_H_
#define INC_STM32F407XX_TIM_DRIVER_H_

#include <stdio.h>
#include "stm32f407.h"
// #include "stm32f407xx.h"
// #include "stm32f4xx_hal_def.h"
typedef struct
{
	TIM_RegDef_t 	*pTIMx;
	uint32_t		Prescalar;
	uint32_t        Reload_Val;
}TIM_Handle_t;

void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t EnorDi);
void timer_init(TIM_Handle_t *pTIMHandle);
void timer_reset(TIM_Handle_t *pTIMHandle);
void TIM_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void timer_set_ms(TIM_Handle_t *pTIMHandle, uint16_t period_ms);
void TIM_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void TIM_IRQHandling(TIM_Handle_t *pTIM_Handle);
#if 0
void TIM_Base_SetConfig(TIM_RegDef_t *TIMx, TIM_Base_InitTypeDef *Structure);
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority);
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);
void NVIC_EnableIRQ(uint16_t IRQn);
#define TIM_SR_UIF_Pos            (0U)                                         
#define TIM_SR_UIF_Msk            (0x1UL << TIM_SR_UIF_Pos)                     /*!< 0x00000001 */
#define TIM_SR_UIF                TIM_SR_UIF_Msk                               

#define TIM_DIER_UIE_Pos          (0U)                                         
#define TIM_DIER_UIE_Msk          (0x1UL << TIM_DIER_UIE_Pos)                   /*!< 0x00000001 */
#define TIM_DIER_UIE              TIM_DIER_UIE_Msk 


#define __HAL_TIM_GET_FLAG(__HANDLE__, __FLAG__)          (((__HANDLE__)->Instance->SR &(__FLAG__)) == (__FLAG__))
#define __HAL_TIM_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->DIER & (__INTERRUPT__)) \
                                                             == (__INTERRUPT__)) ? SET : RESET)
#define __HAL_TIM_CLEAR_IT(__HANDLE__, __INTERRUPT__)      ((__HANDLE__)->Instance->SR = ~(__INTERRUPT__))
#define TIM_FLAG_UPDATE                    TIM_SR_UIF                           /*!< Update interrupt flag         */
#define TIM_IT_UPDATE                      TIM_DIER_UIE 

#define TIM_SR_CC1IF_Pos          (1U)                                         
#define TIM_SR_CC1IF_Msk          (0x1UL << TIM_SR_CC1IF_Pos)                   /*!< 0x00000002 */
#define TIM_SR_CC1IF              TIM_SR_CC1IF_Msk  

#define TIM_SR_CC2IF_Pos          (2U)                                         
#define TIM_SR_CC2IF_Msk          (0x1UL << TIM_SR_CC2IF_Pos)                   /*!< 0x00000004 */
#define TIM_SR_CC2IF              TIM_SR_CC2IF_Msk 

#define TIM_SR_CC3IF_Pos          (3U)                                         
#define TIM_SR_CC3IF_Msk          (0x1UL << TIM_SR_CC3IF_Pos)                   /*!< 0x00000008 */
#define TIM_SR_CC3IF              TIM_SR_CC3IF_Msk 

#define TIM_SR_CC4IF_Pos          (4U)                                         
#define TIM_SR_CC4IF_Msk          (0x1UL << TIM_SR_CC4IF_Pos)                   /*!< 0x00000008 */
#define TIM_SR_CC4IF              TIM_SR_CC4IF_Msk 

#define TIM_SR_COMIF_Pos          (5U)                                         
#define TIM_SR_COMIF_Msk          (0x1UL << TIM_SR_COMIF_Pos)                   /*!< 0x00000020 */
#define TIM_SR_COMIF              TIM_SR_COMIF_Msk   


#define TIM_DIER_CC1IE_Pos        (1U)                                         
#define TIM_DIER_CC1IE_Msk        (0x1UL << TIM_DIER_CC1IE_Pos)                 /*!< 0x00000002 */
#define TIM_DIER_CC1IE            TIM_DIER_CC1IE_Msk 

#define TIM_DIER_CC2IE_Pos        (2U)                                         
#define TIM_DIER_CC2IE_Msk        (0x1UL << TIM_DIER_CC2IE_Pos)                 /*!< 0x00000004 */
#define TIM_DIER_CC2IE            TIM_DIER_CC2IE_Msk

#define TIM_DIER_CC3IE_Pos        (3U)                                         
#define TIM_DIER_CC3IE_Msk        (0x1UL << TIM_DIER_CC3IE_Pos)                 /*!< 0x00000008 */
#define TIM_DIER_CC3IE            TIM_DIER_CC3IE_Msk   

#define TIM_DIER_CC4IE_Pos        (4U)                                         
#define TIM_DIER_CC4IE_Msk        (0x1UL << TIM_DIER_CC4IE_Pos)                 /*!< 0x00000008 */
#define TIM_DIER_CC4IE            TIM_DIER_CC4IE_Msk 

#define TIM_DIER_COMIE_Pos        (5U)                                         
#define TIM_DIER_COMIE_Msk        (0x1UL << TIM_DIER_COMIE_Pos)                 /*!< 0x00000020 */
#define TIM_DIER_COMIE            TIM_DIER_COMIE_Msk                           /*!<COM interrupt enable                 */
#define TIM_DIER_TIE_Pos          (6U)                                         
#define TIM_DIER_TIE_Msk          (0x1UL << TIM_DIER_TIE_Pos)                   /*!< 0x00000040 */
#define TIM_DIER_TIE              TIM_DIER_TIE_Msk                             /*!<Trigger interrupt enable             */
#define TIM_DIER_BIE_Pos          (7U)                                         
#define TIM_DIER_BIE_Msk          (0x1UL << TIM_DIER_BIE_Pos)                   /*!< 0x00000080 */
#define TIM_DIER_BIE              TIM_DIER_BIE_Msk  


#define TIM_CCMR1_CC1S_Pos        (0U)                                         
#define TIM_CCMR1_CC1S_Msk        (0x3UL << TIM_CCMR1_CC1S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR1_CC1S            TIM_CCMR1_CC1S_Msk  

#define TIM_CCMR1_CC2S_Pos        (8U)                                         
#define TIM_CCMR1_CC2S_Msk        (0x3UL << TIM_CCMR1_CC2S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR1_CC2S            TIM_CCMR1_CC2S_Msk  

#define TIM_CCMR2_CC3S_Pos        (0U)                                         
#define TIM_CCMR2_CC3S_Msk        (0x3UL << TIM_CCMR2_CC3S_Pos)                 /*!< 0x00000003 */
#define TIM_CCMR2_CC3S            TIM_CCMR2_CC3S_Msk 

#define TIM_CCMR2_CC4S_Pos        (8U)                                         
#define TIM_CCMR2_CC4S_Msk        (0x3UL << TIM_CCMR2_CC4S_Pos)                 /*!< 0x00000300 */
#define TIM_CCMR2_CC4S            TIM_CCMR2_CC4S_Msk  

#define TIM_FLAG_CC1              TIM_SR_CC1IF
#define TIM_FLAG_CC2              TIM_SR_CC2IF 
#define TIM_FLAG_CC3              TIM_SR_CC3IF
#define TIM_FLAG_CC4              TIM_SR_CC4IF

#define TIM_IT_CC1                TIM_DIER_CC1IE 
#define TIM_IT_CC2                TIM_DIER_CC2IE 
#define TIM_IT_CC3                TIM_DIER_CC3IE 
#define TIM_IT_CC4                TIM_DIER_CC4IE 
#define TIM_IT_COM                TIM_DIER_COMIE                       /*!< Commutation interrupt       */
#define TIM_IT_TRIGGER            TIM_DIER_TIE                         /*!< Trigger interrupt           */
#define TIM_IT_BREAK              TIM_DIER_BIE  

#define TIM_SR_TIF_Pos            (6U)                                         
#define TIM_SR_TIF_Msk            (0x1UL << TIM_SR_TIF_Pos)                     /*!< 0x00000040 */
#define TIM_SR_TIF                TIM_SR_TIF_Msk 
#define TIM_SR_BIF_Pos            (7U)                                         
#define TIM_SR_BIF_Msk            (0x1UL << TIM_SR_BIF_Pos)                     /*!< 0x00000080 */
#define TIM_SR_BIF                TIM_SR_BIF_Msk

#define TIM_FLAG_TRIGGER          TIM_SR_TIF                           /*!< Trigger interrupt flag        */
#define TIM_FLAG_BREAK            TIM_SR_BIF 
#define TIM_FLAG_COM              TIM_SR_COMIF
#endif
#endif

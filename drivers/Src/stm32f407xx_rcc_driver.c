/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Mar 29, 2019
 *      Author: admin
 */


#include "stm32f407xx_rcc_driver.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};



uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0 )
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if (clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}



	//apb1
	temp = ((RCC->CFGR >> 10 ) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 =  (SystemClk / ahbp) /apb1p;

	return pclk1;
}



/*********************************************************************
 * @fn      		  - RCC_GetPCLK2Value
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
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t SystemClock=0,tmp,pclk2;
	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

	uint8_t ahbp,apb2p;

	if(clk_src == 0)
	{
		SystemClock = 16000000;
	}else
	{
		SystemClock = 8000000;
	}
	tmp = (RCC->CFGR >> 4 ) & 0xF;

	if(tmp < 0x08)
	{
		ahbp = 1;
	}else
	{
       ahbp = AHB_PreScaler[tmp-8];
	}

	tmp = (RCC->CFGR >> 13 ) & 0x7;
	if(tmp < 0x04)
	{
		apb2p = 1;
	}else
	{
		apb2p = APB1_PreScaler[tmp-4];
	}

	pclk2 = (SystemClock / ahbp )/ apb2p;

	return pclk2;
}

uint32_t  RCC_GetPLLOutputClock()
{

	return 0;
}




// /*
//  * stm32f407xx_rcc_driver.c
//  *
//  *  Created on: Mar 29, 2019
//  *      Author: admin
//  */


// #include "stm32f407xx_rcc_driver.h"
// #include "stm32f4xx_hal_def.h"
// #include "stm32f4xx_hal.h"


// uint32_t SystemCoreClock = 16000000;

// uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
// uint8_t APB1_PreScaler[4] = { 2, 4 , 8, 16};

// const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
// const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};


// /**
//   * @brief Provides a tick value in millisecond.
//   * @note This function is declared as __weak to be overwritten in case of other 
//   *       implementations in user file.
//   * @retval tick value
//   */


// uint32_t RCC_GetPCLK1Value(void)
// {
// 	uint32_t pclk1,SystemClk;

// 	uint8_t clksrc,temp,ahbp,apb1p;

// 	clksrc = ((RCC->CFGR >> 2) & 0x3);

// 	if(clksrc == 0 )
// 	{
// 		SystemClk = HSI_VALUE;
// 	}else if(clksrc == 1)
// 	{
// 		SystemClk = HSE_VALUE;
// 	}else if (clksrc == 2)
// 	{
// 		SystemClk = RCC_GetPLLOutputClock();
// 	}

// 	//for ahb
// 	temp = ((RCC->CFGR >> 4 ) & 0xF);

// 	if(temp < 8)
// 	{
// 		ahbp = 1;
// 	}else
// 	{
// 		ahbp = AHB_PreScaler[temp-8];
// 	}



// 	//apb1
// 	temp = ((RCC->CFGR >> 10 ) & 0x7);

// 	if(temp < 4)
// 	{
// 		apb1p = 1;
// 	}else
// 	{
// 		apb1p = APB1_PreScaler[temp-4];
// 	}

// 	pclk1 =  (SystemClk / ahbp) /apb1p;

// 	return pclk1;
// }



// /*********************************************************************
//  * @fn      		  - RCC_GetPCLK2Value
//  *
//  * @brief             -
//  *
//  * @param[in]         -
//  * @param[in]         -
//  * @param[in]         -
//  *
//  * @return            -
//  *
//  * @Note              -

//  */
// uint32_t RCC_GetPCLK2Value(void)
// {
// 	uint32_t SystemClock=0,tmp,pclk2;
// 	uint8_t clk_src = ( RCC->CFGR >> 2) & 0X3;

// 	uint8_t ahbp,apb2p;

// 	if(clk_src == 0)
// 	{
// 		SystemClock = 16000000;
// 	}else
// 	{
// 		SystemClock = 8000000;
// 	}
// 	tmp = (RCC->CFGR >> 4 ) & 0xF;

// 	if(tmp < 0x08)
// 	{
// 		ahbp = 1;
// 	}else
// 	{
//        ahbp = AHB_PreScaler[tmp-8];
// 	}

// 	tmp = (RCC->CFGR >> 13 ) & 0x7;
// 	if(tmp < 0x04)
// 	{
// 		apb2p = 1;
// 	}else
// 	{
// 		apb2p = APB1_PreScaler[tmp-4];
// 	}

// 	pclk2 = (SystemClock / ahbp )/ apb2p;

// 	return pclk2;
// }

// uint32_t  RCC_GetPLLOutputClock()
// {
// 	return 0;
// }

// void Set_PLL_Clock(void)
// {
// 	volatile uint32_t tmp;
// 	RCC->APB1ENR |= (1<<28);
// 	tmp = RCC->APB1ENR & (1<<28);
// 	(void)tmp;
// 	RCC->CFGR &= ~(0xFFFF);
// 	RCC->CFGR |= PLL_ENABLE;
// 	RCC->PLLCFGR = (3<<24) | (PLL_SRC_HSE<<22) | (MAIN_PLL_DIV_2 << 16) | (72 << 6) | (4); 
// 	/*
// 	1. PLL Source Mux = HSE
// 	2. /M = /4
// 	3. *N = x72
// 	4. /P = /2
// 	4. /Q = /3
// 	*/
// }


// uint32_t HAL_RCC_GetSysClockFreq(void) 
// {
//   uint32_t pllm = 0U, pllvco = 0U, pllp = 0U;
//   uint32_t sysclockfreq = 0U;

//   /* Get SYSCLK source -------------------------------------------------------*/
//   switch (RCC->CFGR & RCC_CFGR_SWS)
//   {
//     case RCC_CFGR_SWS_HSI:  /* HSI used as system clock source */
//     {
//       sysclockfreq = HSI_VALUE;
//        break;
//     }
//     case RCC_CFGR_SWS_HSE:  /* HSE used as system clock  source */
//     {
//       sysclockfreq = HSE_VALUE;
//       break;
//     }
//     case RCC_CFGR_SWS_PLL:  /* PLL used as system clock  source */
//     {
//       /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
//       SYSCLK = PLL_VCO / PLLP */
//       pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
//       if(__HAL_RCC_GET_PLL_OSCSOURCE() != RCC_PLLSOURCE_HSI)
//       {
//         /* HSE used as PLL clock source */
//         pllvco = (uint32_t) ((((uint64_t) HSE_VALUE * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t)pllm);
//       }
//       else
//       {
//         /* HSI used as PLL clock source */
//         pllvco = (uint32_t) ((((uint64_t) HSI_VALUE * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t)pllm);
//       }
//       pllp = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1U) *2U);

//       sysclockfreq = pllvco/pllp;
//       break;
//     }
//     default:
//     {
//       sysclockfreq = HSI_VALUE;
//       break;
//     }
//   }
//   return sysclockfreq;
// }

// uint32_t HAL_RCC_GetHCLKFreq(void)
// {
// 	return SystemCoreClock;
// }

// HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
// {
//   uint32_t tickstart, pll_config;

//   /* Check Null pointer */
//   if(RCC_OscInitStruct == NULL)
//   {
//     return HAL_ERROR;
//   }

//   /* Check the parameters */
//   //assert_param(IS_RCC_OSCILLATORTYPE(RCC_OscInitStruct->OscillatorType));
//   /*------------------------------- HSE Configuration ------------------------*/
//   if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
//   {
//     /* Check the parameters */
//     //assert_param(IS_RCC_HSE(RCC_OscInitStruct->HSEState));
//     /* When the HSE is used as system clock or clock source for PLL in these cases HSE will not disabled */
// #if defined(STM32F446xx)
//     if((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSE)                                                                     ||\
//       ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE)) ||\
//       ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLLR) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE)))
// #else
//     if((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSE)                                                                     ||\
//       ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE)))
// #endif /* STM32F446xx */
//     {
//       if((__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET) && (RCC_OscInitStruct->HSEState == RCC_HSE_OFF))
//       {
//         return HAL_ERROR;
//       }
//     }
//     else
//     {
//       /* Set the new HSE configuration ---------------------------------------*/
//       __HAL_RCC_HSE_CONFIG(RCC_OscInitStruct->HSEState);

//       /* Check the HSE State */
//       if((RCC_OscInitStruct->HSEState) != RCC_HSE_OFF)
//       {
//         /* Get Start Tick*/
//         tickstart = HAL_GetTick();

//         /* Wait till HSE is ready */
//         while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
//         {
//           if((HAL_GetTick() - tickstart ) > HSE_TIMEOUT_VALUE)
//           {
//             return HAL_TIMEOUT;
//           }
//         }
//       }
//       else
//       {
//         /* Get Start Tick*/
//         tickstart = HAL_GetTick();

//         /* Wait till HSE is bypassed or disabled */
//         while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET)
//         {
//           if((HAL_GetTick() - tickstart ) > HSE_TIMEOUT_VALUE)
//           {
//             return HAL_TIMEOUT;
//           }
//         }
//       }
//     }
//   }
// #if 0
//   /*----------------------------- HSI Configuration --------------------------*/
//   if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI)
//   {
//     /* Check the parameters */
//     assert_param(IS_RCC_HSI(RCC_OscInitStruct->HSIState));
//     assert_param(IS_RCC_CALIBRATION_VALUE(RCC_OscInitStruct->HSICalibrationValue));

//     /* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock */
// #if defined(STM32F446xx)
//     if((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSI)                                                                     ||\
//       ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)) ||\
//       ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLLR) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)))
// #else
//     if((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSI)                                                                     ||\
//       ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)))
// #endif /* STM32F446xx */
//     {
//       /* When HSI is used as system clock it will not disabled */
//       if((__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET) && (RCC_OscInitStruct->HSIState != RCC_HSI_ON))
//       {
//         return HAL_ERROR;
//       }
//       /* Otherwise, just the calibration is allowed */
//       else
//       {
//         /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
//         __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
//       }
//     }
//     else
//     {
//       /* Check the HSI State */
//       if((RCC_OscInitStruct->HSIState)!= RCC_HSI_OFF)
//       {
//         /* Enable the Internal High Speed oscillator (HSI). */
//         __HAL_RCC_HSI_ENABLE();

//         /* Get Start Tick*/
//         tickstart = HAL_GetTick();

//         /* Wait till HSI is ready */
//         while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
//         {
//           if((HAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
//           {
//             return HAL_TIMEOUT;
//           }
//         }

//         /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
//         __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
//       }
//       else
//       {
//         /* Disable the Internal High Speed oscillator (HSI). */
//         __HAL_RCC_HSI_DISABLE();

//         /* Get Start Tick*/
//         tickstart = HAL_GetTick();

//         /* Wait till HSI is ready */
//         while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET)
//         {
//           if((HAL_GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
//           {
//             return HAL_TIMEOUT;
//           }
//         }
//       }
//     }
//   }
//   /*------------------------------ LSI Configuration -------------------------*/
//   if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI)
//   {
//     /* Check the parameters */
//     assert_param(IS_RCC_LSI(RCC_OscInitStruct->LSIState));

//     /* Check the LSI State */
//     if((RCC_OscInitStruct->LSIState)!= RCC_LSI_OFF)
//     {
//       /* Enable the Internal Low Speed oscillator (LSI). */
//       __HAL_RCC_LSI_ENABLE();

//       /* Get Start Tick*/
//       tickstart = HAL_GetTick();

//       /* Wait till LSI is ready */
//       while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET)
//       {
//         if((HAL_GetTick() - tickstart ) > LSI_TIMEOUT_VALUE)
//         {
//           return HAL_TIMEOUT;
//         }
//       }
//     }
//     else
//     {
//       /* Disable the Internal Low Speed oscillator (LSI). */
//       __HAL_RCC_LSI_DISABLE();

//       /* Get Start Tick*/
//       tickstart = HAL_GetTick();

//       /* Wait till LSI is ready */
//       while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) != RESET)
//       {
//         if((HAL_GetTick() - tickstart ) > LSI_TIMEOUT_VALUE)
//         {
//           return HAL_TIMEOUT;
//         }
//       }
//     }
//   }
//   /*------------------------------ LSE Configuration -------------------------*/
//   if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
//   {
//     FlagStatus       pwrclkchanged = RESET;

//     /* Check the parameters */
//     assert_param(IS_RCC_LSE(RCC_OscInitStruct->LSEState));

//     /* Update LSE configuration in Backup Domain control register    */
//     /* Requires to enable write access to Backup Domain of necessary */
//     if(__HAL_RCC_PWR_IS_CLK_DISABLED())
//     {
//       __HAL_RCC_PWR_CLK_ENABLE();
//       pwrclkchanged = SET;
//     }

//     if(HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
//     {
//       /* Enable write access to Backup domain */
//       SET_BIT(PWR->CR, PWR_CR_DBP);

//       /* Wait for Backup domain Write protection disable */
//       tickstart = HAL_GetTick();

//       while(HAL_IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
//       {
//         if((HAL_GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE)
//         {
//           return HAL_TIMEOUT;
//         }
//       }
//     }

//     /* Set the new LSE configuration -----------------------------------------*/
//     __HAL_RCC_LSE_CONFIG(RCC_OscInitStruct->LSEState);
//     /* Check the LSE State */
//     if((RCC_OscInitStruct->LSEState) != RCC_LSE_OFF)
//     {
//       /* Get Start Tick*/
//       tickstart = HAL_GetTick();

//       /* Wait till LSE is ready */
//       while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
//       {
//         if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
//         {
//           return HAL_TIMEOUT;
//         }
//       }
//     }
//     else
//     {
//       /* Get Start Tick*/
//       tickstart = HAL_GetTick();

//       /* Wait till LSE is ready */
//       while(__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) != RESET)
//       {
//         if((HAL_GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
//         {
//           return HAL_TIMEOUT;
//         }
//       }
//     }

//     /* Restore clock configuration if changed */
//     if(pwrclkchanged == SET)
//     {
//       __HAL_RCC_PWR_CLK_DISABLE();
//     }
//   }
// #endif
//   /*-------------------------------- PLL Configuration -----------------------*/
//   /* Check the parameters */
//   //assert_param(IS_RCC_PLL(RCC_OscInitStruct->PLL.PLLState));
//   if ((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_NONE)
//   {
//     /* Check if the PLL is used as system clock or not */
//     if(__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL)
//     {
//       if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON)
//       {
//         /* Check the parameters */
//         // assert_param(IS_RCC_PLLSOURCE(RCC_OscInitStruct->PLL.PLLSource));
//         // assert_param(IS_RCC_PLLM_VALUE(RCC_OscInitStruct->PLL.PLLM));
//         // assert_param(IS_RCC_PLLN_VALUE(RCC_OscInitStruct->PLL.PLLN));
//         // assert_param(IS_RCC_PLLP_VALUE(RCC_OscInitStruct->PLL.PLLP));
//         // assert_param(IS_RCC_PLLQ_VALUE(RCC_OscInitStruct->PLL.PLLQ));

//         /* Disable the main PLL. */
//         __HAL_RCC_PLL_DISABLE();

//         /* Get Start Tick*/
//         tickstart = HAL_GetTick();

//         /* Wait till PLL is ready */
//         while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET)
//         {
//           if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
//           {
//             return HAL_TIMEOUT;
//           }
//         }

//         /* Configure the main PLL clock source, multiplication and division factors. */
//         WRITE_REG(RCC->PLLCFGR, (RCC_OscInitStruct->PLL.PLLSource                                            | \
//                                  RCC_OscInitStruct->PLL.PLLM                                                 | \
//                                  (RCC_OscInitStruct->PLL.PLLN << RCC_PLLCFGR_PLLN_Pos)                       | \
//                                  (((RCC_OscInitStruct->PLL.PLLP >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos)        | \
//                                  (RCC_OscInitStruct->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos)));
//         /* Enable the main PLL. */
//         __HAL_RCC_PLL_ENABLE();

//         /* Get Start Tick*/
//         tickstart = HAL_GetTick();

//         /* Wait till PLL is ready */
//         while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
//         {
//           if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
//           {
//             return HAL_TIMEOUT;
//           }
//         }
//       }
//       else
//       {
//         /* Disable the main PLL. */
//         __HAL_RCC_PLL_DISABLE();

//         /* Get Start Tick*/
//         tickstart = HAL_GetTick();

//         /* Wait till PLL is ready */
//         while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET)
//         {
//           if((HAL_GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
//           {
//             return HAL_TIMEOUT;
//           }
//         }
//       }
//     }
//     else
//     {
//       /* Check if there is a request to disable the PLL used as System clock source */
//       if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_OFF)
//       {
//         return HAL_ERROR;
//       }
//       else
//       {
//         /* Do not return HAL_ERROR if request repeats the current configuration */
//         pll_config = RCC->PLLCFGR;
// #if defined (RCC_PLLCFGR_PLLR)
//         if (((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_OFF) ||
//             (READ_BIT(pll_config, RCC_PLLCFGR_PLLSRC) != RCC_OscInitStruct->PLL.PLLSource) ||
//             (READ_BIT(pll_config, RCC_PLLCFGR_PLLM) != (RCC_OscInitStruct->PLL.PLLM) << RCC_PLLCFGR_PLLM_Pos) ||
//             (READ_BIT(pll_config, RCC_PLLCFGR_PLLN) != (RCC_OscInitStruct->PLL.PLLN) << RCC_PLLCFGR_PLLN_Pos) ||
//             (READ_BIT(pll_config, RCC_PLLCFGR_PLLP) != (((RCC_OscInitStruct->PLL.PLLP >> 1U) - 1U)) << RCC_PLLCFGR_PLLP_Pos) ||
//             (READ_BIT(pll_config, RCC_PLLCFGR_PLLQ) != (RCC_OscInitStruct->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos)) ||
//             (READ_BIT(pll_config, RCC_PLLCFGR_PLLR) != (RCC_OscInitStruct->PLL.PLLR << RCC_PLLCFGR_PLLR_Pos)))
// #else
//         if (((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_OFF) ||
//             (READ_BIT(pll_config, RCC_PLLCFGR_PLLSRC) != RCC_OscInitStruct->PLL.PLLSource) ||
//             (READ_BIT(pll_config, RCC_PLLCFGR_PLLM) != (RCC_OscInitStruct->PLL.PLLM) << RCC_PLLCFGR_PLLM_Pos) ||
//             (READ_BIT(pll_config, RCC_PLLCFGR_PLLN) != (RCC_OscInitStruct->PLL.PLLN) << RCC_PLLCFGR_PLLN_Pos) ||
//             (READ_BIT(pll_config, RCC_PLLCFGR_PLLP) != (((RCC_OscInitStruct->PLL.PLLP >> 1U) - 1U)) << RCC_PLLCFGR_PLLP_Pos) ||
//             (READ_BIT(pll_config, RCC_PLLCFGR_PLLQ) != (RCC_OscInitStruct->PLL.PLLQ << RCC_PLLCFGR_PLLQ_Pos)))
// #endif
//         {
//           return HAL_ERROR;
//         }
//       }
//     }
//   }
//   return HAL_OK;
// }



// HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency)
// {
//   uint32_t tickstart;

//   /* Check Null pointer */
//   if(RCC_ClkInitStruct == NULL)
//   {
// 	  printf("null ptr\n");
//     return HAL_ERROR;
//   }

//   /* Check the parameters */
// //   assert_param(IS_RCC_CLOCKTYPE(RCC_ClkInitStruct->ClockType));
// //   assert_param(IS_FLASH_LATENCY(FLatency));

//   /* To correctly read data from FLASH memory, the number of wait states (LATENCY)
//     must be correctly programmed according to the frequency of the CPU clock
//     (HCLK) and the supply voltage of the device. */

//   /* Increasing the number of wait states because of higher CPU frequency */
//   if(FLatency > __HAL_FLASH_GET_LATENCY())
//   {
//     /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
//     __HAL_FLASH_SET_LATENCY(FLatency);

//     /* Check that the new number of wait states is taken into account to access the Flash
//     memory by reading the FLASH_ACR register */
//     if(__HAL_FLASH_GET_LATENCY() != FLatency)
//     {
// 	    printf("flatency\n");
//       return HAL_ERROR;
//     }
//   }

//   /*-------------------------- HCLK Configuration --------------------------*/
//   if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
//   {
//     /* Set the highest APBx dividers in order to ensure that we do not go through
//        a non-spec phase whatever we decrease or increase HCLK. */
//     if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
//     {
//       MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV16);
//     }

//     if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
//     {
//       MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, (RCC_HCLK_DIV16 << 3));
//     }

//     //assert_param(IS_RCC_HCLK(RCC_ClkInitStruct->AHBCLKDivider));
//     MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);
//   }

//   /*------------------------- SYSCLK Configuration ---------------------------*/
//   if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
//   {
//     //assert_param(IS_RCC_SYSCLKSOURCE(RCC_ClkInitStruct->SYSCLKSource));

//     /* HSE is selected as System Clock Source */
//     if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
//     {
//       /* Check the HSE ready flag */
//       if(__HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
//       {
// 		    printf("hse rdy\n");
//         return HAL_ERROR;
//       }
//     }
//     /* PLL is selected as System Clock Source */
//     else if((RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)   ||
//             (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLRCLK))
//     {
//       /* Check the PLL ready flag */
//       if(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
//       {
// 		    printf("pll rdy\n");
//         return HAL_ERROR;
//       }
//     }
//     /* HSI is selected as System Clock Source */
//     else
//     {
//       /* Check the HSI ready flag */
//       if(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
//       {
// 		    printf("hsi rdy\n");
//         return HAL_ERROR;
//       }
//     }

//     __HAL_RCC_SYSCLK_CONFIG(RCC_ClkInitStruct->SYSCLKSource);

//     /* Get Start Tick */
//     tickstart = HAL_GetTick();

//     while (__HAL_RCC_GET_SYSCLK_SOURCE() != (RCC_ClkInitStruct->SYSCLKSource << RCC_CFGR_SWS_Pos))
//     {
//       if ((HAL_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE)
//       {
//         return HAL_TIMEOUT;
//       }
//     }
//   }

//   /* Decreasing the number of wait states because of lower CPU frequency */
//   if(FLatency < __HAL_FLASH_GET_LATENCY())
//   {
//      /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
//     __HAL_FLASH_SET_LATENCY(FLatency);

//     /* Check that the new number of wait states is taken into account to access the Flash
//     memory by reading the FLASH_ACR register */
//     if(__HAL_FLASH_GET_LATENCY() != FLatency)
//     {
// 	    printf("flatency 2\n");
//       return HAL_ERROR;
//     }
//   }

//   /*-------------------------- PCLK1 Configuration ---------------------------*/
//   if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
//   {
//     //assert_param(IS_RCC_PCLK(RCC_ClkInitStruct->APB1CLKDivider));
//     MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);
//   }

//   /*-------------------------- PCLK2 Configuration ---------------------------*/
//   if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
//   {
//     //assert_param(IS_RCC_PCLK(RCC_ClkInitStruct->APB2CLKDivider));
//     MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_ClkInitStruct->APB2CLKDivider) << 3U));
//   }

//   /* Update the SystemCoreClock global variable */
//   SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE)>> RCC_CFGR_HPRE_Pos];

//   /* Configure the source of time base considering new system clocks settings */
//   HAL_InitTick(uwTickPrio);

//   return HAL_OK;
// }

// void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t *pFLatency)
// {
//   /* Set all possible values for the Clock type parameter --------------------*/
//   RCC_ClkInitStruct->ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

//   /* Get the SYSCLK configuration --------------------------------------------*/
//   RCC_ClkInitStruct->SYSCLKSource = (uint32_t)(RCC->CFGR & RCC_CFGR_SW);

//   /* Get the HCLK configuration ----------------------------------------------*/
//   RCC_ClkInitStruct->AHBCLKDivider = (uint32_t)(RCC->CFGR & RCC_CFGR_HPRE);

//   /* Get the APB1 configuration ----------------------------------------------*/
//   RCC_ClkInitStruct->APB1CLKDivider = (uint32_t)(RCC->CFGR & RCC_CFGR_PPRE1);

//   /* Get the APB2 configuration ----------------------------------------------*/
//   RCC_ClkInitStruct->APB2CLKDivider = (uint32_t)((RCC->CFGR & RCC_CFGR_PPRE2) >> 3U);

//   /* Get the Flash Wait State (Latency) configuration ------------------------*/
//   *pFLatency = (uint32_t)(FLASH->ACR & FLASH_ACR_LATENCY);
// }

// uint32_t HAL_RCC_GetPCLK1Freq(void)
// {
//   /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
//   return (HAL_RCC_GetHCLKFreq() >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos]);
// }

// uint32_t NVIC_GetPriorityGrouping(void)
// {
//   return ((uint32_t)((SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));
// }

// void NVIC_SetPriority(int IRQn, uint32_t priority)
// {
//   if ((int32_t)(IRQn) >= 0)
//   {
//     NVIC->IP[((uint32_t)IRQn)]               = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
//   }
//   else
//   {
//     SCB->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
//   }
// }

// uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
// {
//   uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);   /* only values 0..7 are used          */
//   uint32_t PreemptPriorityBits;
//   uint32_t SubPriorityBits;

//   PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(__NVIC_PRIO_BITS)) ? (uint32_t)(__NVIC_PRIO_BITS) : (uint32_t)(7UL - PriorityGroupTmp);
//   SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(__NVIC_PRIO_BITS)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(__NVIC_PRIO_BITS));

//   return (
//            ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
//            ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
//          );
// }     
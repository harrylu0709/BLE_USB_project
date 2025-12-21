/*
 * system_init.c
 *
 *  Created on: Oct 18, 2025
 *      Author: bo
 */
#include <stdint.h>
#include "system_stm32f4xx.h"
#include "stm32f4xx.h"
#include "logger.h"
/* Update by SystemCoreClockUpdate funciton
, we don't change this value in runtime, so it can be stationary
HCLK
*/
LogLevel system_log_level = LOG_LEVEL_DEBUG;
uint32_t SystemCoreClock = 72000000; 
/*
PLL: M=4, N=72, P=2,Q=3
AHB prescalar = 1
APB prescalar1 = 2, APB prescalar2 = 1
MCO1 prescalar = 2

Procedure for configuring the system clock when using HSE: Enable HSE and await HSE stabilisation
Configure the prescaler factors for AHB, APB2, and APB1
Set the PLL clock source and PLL multiplier; this is where most frequency configurations are established
Enable the PLL and await PLL stabilisation
Switch PLLCK to the system clock SYSCLK
Read the clock selection status bit to ensure PLLCLK is selected as the system clock

*/

void configure_clock()
{
    /* Configures flash latency */
    MODIFY_REG(FLASH->ACR, 
        FLASH_ACR_LATENCY, 
        _VAL2FLD(FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2WS)
        //FLASH_ACR_LATENCY_2WS << FLASH_ACR_LATENCY_Pos)
    );

    /* Enable HSE */
    SET_BIT(RCC->CR, RCC_CR_HSEON);

    /* Wait unitl HSE is stable */
    while(!READ_BIT(RCC->CR, RCC_CR_HSERDY));

    /* Configure PLL: source = HSE, PLLCLK = 72MHz */
    MODIFY_REG(RCC->PLLCFGR, 
        (RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLSRC | RCC_PLLCFGR_PLLP), 
        (_VAL2FLD(RCC_PLLCFGR_PLLM, 4) | _VAL2FLD(RCC_PLLCFGR_PLLN, 72) |_VAL2FLD(RCC_PLLCFGR_PLLQ, 3) | RCC_PLLCFGR_PLLSRC_HSE)
    );

    /* Enable PLL module */
    SET_BIT(RCC->CR, RCC_CR_PLLON);


    /* Wait unitl PLL is stable */
    while(!READ_BIT(RCC->CR, RCC_CR_PLLRDY));

    /* Switches system clock to PLL */
    MODIFY_REG(RCC->CFGR, 
        RCC_CFGR_SW, 
        _VAL2FLD(RCC_CFGR_SW, RCC_CFGR_SW_PLL)
    );

    /* Configure the APB PRE1 (APB PRE2 = 1, AHB PRE = 1 by default) */
    MODIFY_REG(RCC->CFGR, 
        RCC_CFGR_PPRE1, 
        _VAL2FLD(RCC_CFGR_PPRE1, 4)
    );

    /* Wait unitl PLL is used, ensuring that clock source (PLL) is used as system clock */
    while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    /* Disable HSI */
    CLEAR_BIT(RCC->CR, RCC_CR_HSION);
}

void configure_MCO1()
{

}

void SystemInit(void)
{
    configure_clock();
}

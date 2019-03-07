#include <bsp.h>
#include "stm32f10x_flash.h"
#include "stm32f10x_rcc.h"

#define  BSP_RCC_TO_VAL                  0x00000FFF             /* Max Timeout for RCC register                             */

static void  BSP_RCC_Init (void);

/*
*********************************************************************************************************
*                                         OS_CPU_SysTickClkFreq()
*
* Description : Get system tick clock frequency.
*
* Argument(s) : none.
*
* Return(s)   : Clock frequency (of system tick).
*
* Caller(s)   : BSP_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/
INT32U  OS_CPU_SysTickClkFreq (void)
{
    INT32U  freq;


    freq = BSP_CPU_ClkFreq();
    return (freq);
}
/*
*********************************************************************************************************
*                                       BSP_CPU_ClkFreq()
*
* Description : This function reads CPU registers to determine the CPU clock frequency of the chip in KHz.
*
* Argument(s) : none.
*
* Return(s)   : The CPU clock frequency, in Hz.
*
* Caller(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

unsigned int  BSP_CPU_ClkFreq (void)
{

    static  RCC_ClocksTypeDef  rcc_clocks;


    RCC_GetClocksFreq(&rcc_clocks);

   //return ((CPU_INT32U)rcc_clocks.HCLK_Frequency);
    return ((unsigned int)rcc_clocks.HCLK_Frequency);

}
void  BSP_IntDisAll (void)
{
    //CPU_IntDis();
}


/*
*********************************************************************************************************
*                                         BSP_RCC_Init()
*
* Description : Initializes the RCC module. Set the FLASH memmory timing and the system clock dividers
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/

static void  BSP_RCC_Init (void)
{
   // CPU_INT32U  rcc_to;                                          /* RCC registers timeout                                    */
    unsigned int rcc_to;

    RCC_DeInit();                                                /*  Reset the RCC clock config to the default reset state   */

    RCC_HSEConfig(RCC_HSE_ON);                                   /*  HSE Oscillator ON                                       */


    rcc_to = BSP_RCC_TO_VAL;

    while ((rcc_to > 0) &&
           (RCC_WaitForHSEStartUp() != SUCCESS)) {               /* Wait until the oscilator is stable                       */
        rcc_to--;
    }

    FLASH_SetLatency(FLASH_Latency_2);
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);         /* Fcpu = (PLL_src * PLL_MUL) = (8 Mhz / 1) * (9) = 72Mhz   */
    RCC_PLLCmd(ENABLE);

    rcc_to = BSP_RCC_TO_VAL;

    while ((rcc_to > 0) &&
           (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)) {
        rcc_to--;
    }

    RCC_HCLKConfig(RCC_SYSCLK_Div1);                             /* Set system clock dividers                                */
    RCC_PCLK2Config(RCC_HCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    FLASH_SetLatency(FLASH_Latency_2);                           /* Embedded Flash Configuration                             */
    FLASH_HalfCycleAccessCmd(FLASH_HalfCycleAccess_Disable);
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
									|RCC_APB2Periph_GPIOF
									| RCC_APB2Periph_GPIOC
									|RCC_APB2Periph_AFIO
									|RCC_APB2Periph_USART1, ENABLE);//Ê¹ÄÜLEDµÆ
}


void  BSP_Init (void)
{
    BSP_RCC_Init();                                             /* Initialize the Reset and Control (RCC) module             */

}


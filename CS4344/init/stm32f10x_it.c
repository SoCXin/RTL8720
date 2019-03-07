/**
  ******************************************************************************
  * @file USART/Interrupt/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version  V3.0.0
  * @date  04/06/2009
  * @brief  Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and 
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "includes.h"
#include "Hapc_BSP.h"


extern OS_EVENT* SemBufWr;
extern BuffDef* BuffP;
extern BuffDef Buff[2];
extern void BuffSwitch(void);
extern SD_ProcessIRQSrc(void);
/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
   while(1);
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{

}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  

}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTickHandler(void)
{
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/
void DMA1_Channel5_IRQHandler(void) 
{
	OSIntEnter();
	//static u8 xx=0;
	if(DMA_GetITStatus(DMA1_IT_TC5) != RESET)
	{
		
		DMA_ClearITPendingBit(DMA1_IT_TC5);
		
		//LED1_OFF;
		//LED2_OFF;
		OSSemPost(SemBufWr);

		//DMA_Cmd(DMA1_Channel5, DISABLE);
		
	}
	OSIntExit();
}



void SDIO_IRQHandler(void)
{
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
}

/**
  * @brief  This function handles USART1 global interrupt request.
  * @param  None
  * @retval : None
  */

	
void USART1_IRQHandler(void)      //串口1 中断服务程序
{
 	static u8 mask=0;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	   //判断读寄存器是否非空
	{
		USART_ClearFlag(USART1, USART_FLAG_RXNE);
		UsartRecData[mask] = USART_ReceiveData(USART1);//得到一个字节数据
		 mask ++ ;
		 if(mask >= USART_RECDATAMAX)//若达到一帧长度
		{
			mask = 0 ;
			//USART_SendData(USART1, 'a');

			UsartRecFinish=1;//帧接收结束标志置位	
		}
		else UsartRecFinish=0;//若未达到一帧长度则清零
	}
	  
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)//这段是为了避免STM32 USART 第一个字节发不出去的BUG 
	{ 
		 USART_ITConfig(USART1, USART_IT_TXE, DISABLE);//禁止发缓冲器空中断， 
	}	
  
}


void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)	
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

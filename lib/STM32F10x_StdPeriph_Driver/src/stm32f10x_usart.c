/**
  ******************************************************************************
  * @file    stm32f10x_usart.c
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    10/15/2010
  * @brief   This file provides all the USART firmware functions.
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
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @defgroup USART 
  * @brief USART driver modules
  * @{
  */

/** @defgroup USART_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup USART_Private_Defines
  * @{
  */

#define CR1_UE_Set                ((uint16_t)0x2000)  /*!< USART Enable Mask */
#define CR1_UE_Reset              ((uint16_t)0xDFFF)  /*!< USART Disable Mask */

#define CR1_WAKE_Mask             ((uint16_t)0xF7FF)  /*!< USART WakeUp Method Mask */

#define CR1_RWU_Set               ((uint16_t)0x0002)  /*!< USART mute mode Enable Mask */
#define CR1_RWU_Reset             ((uint16_t)0xFFFD)  /*!< USART mute mode Enable Mask */
#define CR1_SBK_Set               ((uint16_t)0x0001)  /*!< USART Break Character send Mask */
#define CR1_CLEAR_Mask            ((uint16_t)0xE9F3)  /*!< USART CR1 Mask */
#define CR2_Address_Mask          ((uint16_t)0xFFF0)  /*!< USART address Mask */

#define CR2_LINEN_Set              ((uint16_t)0x4000)  /*!< USART LIN Enable Mask */
#define CR2_LINEN_Reset            ((uint16_t)0xBFFF)  /*!< USART LIN Disable Mask */

#define CR2_LBDL_Mask             ((uint16_t)0xFFDF)  /*!< USART LIN Break detection Mask */
#define CR2_STOP_CLEAR_Mask       ((uint16_t)0xCFFF)  /*!< USART CR2 STOP Bits Mask */
#define CR2_CLOCK_CLEAR_Mask      ((uint16_t)0xF0FF)  /*!< USART CR2 Clock Mask */

#define CR3_SCEN_Set              ((uint16_t)0x0020)  /*!< USART SC Enable Mask */
#define CR3_SCEN_Reset            ((uint16_t)0xFFDF)  /*!< USART SC Disable Mask */

#define CR3_NACK_Set              ((uint16_t)0x0010)  /*!< USART SC NACK Enable Mask */
#define CR3_NACK_Reset            ((uint16_t)0xFFEF)  /*!< USART SC NACK Disable Mask */

#define CR3_HDSEL_Set             ((uint16_t)0x0008)  /*!< USART Half-Duplex Enable Mask */
#define CR3_HDSEL_Reset           ((uint16_t)0xFFF7)  /*!< USART Half-Duplex Disable Mask */

#define CR3_IRLP_Mask             ((uint16_t)0xFFFB)  /*!< USART IrDA LowPower mode Mask */
#define CR3_CLEAR_Mask            ((uint16_t)0xFCFF)  /*!< USART CR3 Mask */

#define CR3_IREN_Set              ((uint16_t)0x0002)  /*!< USART IrDA Enable Mask */
#define CR3_IREN_Reset            ((uint16_t)0xFFFD)  /*!< USART IrDA Disable Mask */
#define GTPR_LSB_Mask             ((uint16_t)0x00FF)  /*!< Guard Time Register LSB Mask */
#define GTPR_MSB_Mask             ((uint16_t)0xFF00)  /*!< Guard Time Register MSB Mask */
#define IT_Mask                   ((uint16_t)0x001F)  /*!< USART Interrupt Mask */

/* USART OverSampling-8 Mask */
#define CR1_OVER8_Set             ((u16)0x8000)  /* USART OVER8 mode Enable Mask */
#define CR1_OVER8_Reset           ((u16)0x7FFF)  /* USART OVER8 mode Disable Mask */

/* USART One Bit Sampling Mask */
#define CR3_ONEBITE_Set           ((u16)0x0800)  /* USART ONEBITE mode Enable Mask */
#define CR3_ONEBITE_Reset         ((u16)0xF7FF)  /* USART ONEBITE mode Disable Mask */

/**
  * @}
  */

/** @defgroup USART_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USART_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup USART_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup USART_Private_Functions
  * @{
  */

/**
  * @brief  Deinitializes the USARTx peripheral registers to their default reset values.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values: USART1, USART2, USART3, UART4 or UART5.
  * @retval None
  */
  void USART_DeInit(USART_TypeDef* USARTx)
{
	/* 检查输入参数 */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
 
	if (USARTx == USART1)
	{
		/* 强制或者释放APB2外设复位 */
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
	}
	/* 强制或者释放APB1外设复位 */
	else if (USARTx == USART2)
	{
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);
	}
	else if (USARTx == USART3)
	{
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);
	}    
	else if (USARTx == UART4)
	{
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, ENABLE);
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, DISABLE);
	}    
	else
	{
		if (USARTx == UART5)
		{ 
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, ENABLE);
			RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, DISABLE);
		}
	}
}


/**
  * @brief  Initializes the USARTx peripheral according to the specified
  *   parameters in the USART_InitStruct .
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure
  *   that contains the configuration information for the specified USART peripheral.
  * @retval None
  */
/*-----------------------------------------------------------------------------------------------------
// 功能描述  根据USART_InitStruct中指定的参数初始化外设USARTx寄存器
// 输入参数  USARTx：选择USART或 UART 外设
//     可以是下列值之一：
//         USART1, USART2, USART3, UART4 或 UART5
// 输入参数  USART_InitStruct：指向结构USART_InitTypeDef的指针，包含了外设USART的配置信息
// 返回值  无
-----------------------------------------------------------------------------------------------------*/
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct)
{
	uint32_t tmpreg = 0x00, apbclock = 0x00;
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;
	uint32_t usartxbase = 0;
	RCC_ClocksTypeDef RCC_ClocksStatus;
	/* 检查输入参数 */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_BAUDRATE(USART_InitStruct->USART_BaudRate));  
	assert_param(IS_USART_WORD_LENGTH(USART_InitStruct->USART_WordLength));
	assert_param(IS_USART_STOPBITS(USART_InitStruct->USART_StopBits));
	assert_param(IS_USART_PARITY(USART_InitStruct->USART_Parity));
	assert_param(IS_USART_MODE(USART_InitStruct->USART_Mode));
	assert_param(IS_USART_HARDWARE_FLOW_CONTROL(USART_InitStruct->USART_HardwareFlowControl));
	/* 硬件流控制只对USART1, USART2 和 USART3 有效 */
	if (USART_InitStruct->USART_HardwareFlowControl != USART_HardwareFlowControl_None)
	{
		assert_param(IS_USART_123_PERIPH(USARTx));
	}
 
	usartxbase = (uint32_t)USARTx;
 
	/*---------------------------- USART CR2 配置-----------------------*/
	tmpreg = USARTx->CR2;
	/* 清除STOP[13:12] */
	tmpreg &= CR2_STOP_CLEAR_Mask;
	/* 配置USART 停止位，时钟，时钟极性，时钟相位和最后一位------------*/
	/* 根据USART_StopBits设置STOP[13:12] */
	tmpreg |= (uint32_t)USART_InitStruct->USART_StopBits;
 
	/* 写USART CR2 */
	USARTx->CR2 = (uint16_t)tmpreg;
 
	/*---------------------------- USART CR1 配置-----------------------*/
	tmpreg = USARTx->CR1;
	/* 清除M, PCE, PS, TE 和 RE */
	tmpreg &= CR1_CLEAR_Mask;
	/* 配置USART 字长，校验和模式 ----------------------- */
	/* 根据USART_WordLength设置M */
	/* 根据USART_Parity设置PCE和PS */
	/* 根据USART_Mode设置TE和RE */
	tmpreg |= (uint32_t)USART_InitStruct->USART_WordLength | USART_InitStruct->USART_Parity | 
                            USART_InitStruct->USART_Mode;
	/* 写USART CR1 */
	USARTx->CR1 = (uint16_t)tmpreg;
 
	/*---------------------------- USART CR3 配置 -----------------------*/  
	tmpreg = USARTx->CR3;
	/* 清除CTSE和RTSE */
	tmpreg &= CR3_CLEAR_Mask;
	/* 配置USART硬件流控制 -------------------------------------------------*/
	/* 根据USART_HardwareFlowControl设置CTSE和RTSE */
	tmpreg |= USART_InitStruct->USART_HardwareFlowControl;
	/* 写USART CR3 */
	USARTx->CR3 = (uint16_t)tmpreg;
 
	/*---------------------------- USART BRR 配置 -----------------------*/
	/* 配置USART波特率 -------------------------------------------*/
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	if (usartxbase == USART1_BASE)
	{
		apbclock = RCC_ClocksStatus.PCLK2_Frequency;
	}
	else
	{
		apbclock = RCC_ClocksStatus.PCLK1_Frequency;
	}
 
	/* 整数部分 */
	if ((USARTx->CR1 & CR1_OVER8_Set) != 0)
	{
		/* 在过采样模式为8次采样下的整数部分的计算 */
		integerdivider = ((25 * apbclock) / (2 * (USART_InitStruct->USART_BaudRate)));    
	}
	else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
	{
		/*在过采样模式为16次采样下的整数部分的计算 */
		integerdivider = ((25 * apbclock) / (4 * (USART_InitStruct->USART_BaudRate)));    
	}
	tmpreg = (integerdivider / 100) << 4;
 
	/* 小数部分 */
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
 
	/* Implement the fractional part in the register */
	if ((USARTx->CR1 & CR1_OVER8_Set) != 0)
	{
		tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
	}
	else /* if ((USARTx->CR1 & CR1_OVER8_Set) == 0) */
	{
		tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);
	}
 
	/* 写USART BRR */
	USARTx->BRR = (uint16_t)tmpreg;
}
/**
  * @brief  Fills each USART_InitStruct member with its default value.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure
  *   which will be initialized.
  * @retval None
  */
/*-----------------------------------------------------------------------------------------------------
// 功能描述  把USART_InitStruct中的每一个参数按缺省值填入
// 输入参数  USART_InitStruct：指向结构USART_InitTypeDef的指针，待初始化
// 返回值  无
-----------------------------------------------------------------------------------------------------*/
void USART_StructInit(USART_InitTypeDef* USART_InitStruct)
{
	/* USART_InitStruct 成员缺省值 */
	USART_InitStruct->USART_BaudRate = 9600; //波特率9600
	USART_InitStruct->USART_WordLength = USART_WordLength_8b; //字长8位
	USART_InitStruct->USART_StopBits = USART_StopBits_1; //停止位1位
	USART_InitStruct->USART_Parity = USART_Parity_No ; //不校验
	USART_InitStruct->USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //收发模式
	USART_InitStruct->USART_HardwareFlowControl = USART_HardwareFlowControl_None; //没有硬件流控制
}

/**
  * @brief  Initializes the USARTx peripheral Clock according to the 
  *   specified parameters in the USART_ClockInitStruct .
  * @param  USARTx: where x can be 1, 2, 3 to select the USART peripheral.
  * @param  USART_ClockInitStruct: pointer to a USART_ClockInitTypeDef
  *   structure that contains the configuration information for the specified 
  *   USART peripheral.  
  * @note The Smart Card mode is not available for UART4 and UART5.
  * @retval None
  */
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct)
{
  uint32_t tmpreg = 0x00;
  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));
  assert_param(IS_USART_CLOCK(USART_ClockInitStruct->USART_Clock));
  assert_param(IS_USART_CPOL(USART_ClockInitStruct->USART_CPOL));
  assert_param(IS_USART_CPHA(USART_ClockInitStruct->USART_CPHA));
  assert_param(IS_USART_LASTBIT(USART_ClockInitStruct->USART_LastBit));
  
/*---------------------------- USART CR2 Configuration -----------------------*/
  tmpreg = USARTx->CR2;
  /* Clear CLKEN, CPOL, CPHA and LBCL bits */
  tmpreg &= CR2_CLOCK_CLEAR_Mask;
  /* Configure the USART Clock, CPOL, CPHA and LastBit ------------*/
  /* Set CLKEN bit according to USART_Clock value */
  /* Set CPOL bit according to USART_CPOL value */
  /* Set CPHA bit according to USART_CPHA value */
  /* Set LBCL bit according to USART_LastBit value */
  tmpreg |= (uint32_t)USART_ClockInitStruct->USART_Clock | USART_ClockInitStruct->USART_CPOL | 
                 USART_ClockInitStruct->USART_CPHA | USART_ClockInitStruct->USART_LastBit;
  /* Write to USART CR2 */
  USARTx->CR2 = (uint16_t)tmpreg;
}

/**
  * @brief  Fills each USART_ClockInitStruct member with its default value.
  * @param  USART_ClockInitStruct: pointer to a USART_ClockInitTypeDef
  *   structure which will be initialized.
  * @retval None
  */
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct)
{
  /* USART_ClockInitStruct members default value */
  USART_ClockInitStruct->USART_Clock = USART_Clock_Disable;
  USART_ClockInitStruct->USART_CPOL = USART_CPOL_Low;
  USART_ClockInitStruct->USART_CPHA = USART_CPHA_1Edge;
  USART_ClockInitStruct->USART_LastBit = USART_LastBit_Disable;
}

/**
  * @brief  Enables or disables the specified USART peripheral.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USARTx peripheral.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
/*-----------------------------------------------------------------------------------------------------
// 功能描述  使能或者失能USART外设
// 输入参数  USARTx：选择USART或 UART 外设
//     可以是下列值之一：
//         USART1, USART2, USART3, UART4 或 UART5
// 输入参数  NewState：外设USARTx的新状态
//     可以是：ENABLE或DISABLE
// 返回值  无
-----------------------------------------------------------------------------------------------------*/
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
	/* 检查输入参数 */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
 
	if (NewState != DISABLE)
	{
		/* 通过设置CR1中的UE使能选择的USART */
		USARTx->CR1 |= CR1_UE_Set;
	}
	else
	{
		/* 通过设置CR1中的UE失能选择的USART */
		USARTx->CR1 &= CR1_UE_Reset;
	}
}

/**
  * @brief  Enables or disables the specified USART interrupts.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt sources to be enabled or disabled.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TXE:  Tansmit Data Register empty interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *     @arg USART_IT_IDLE: Idle line detection interrupt
  *     @arg USART_IT_PE:   Parity Error interrupt
  *     @arg USART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @param  NewState: new state of the specified USARTx interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
/*-----------------------------------------------------------------------------------------------------
// 功能描述  使能或失能指定的USART中断
// 输入参数  USARTx：选择USART或 UART 外设
//     可以是下列值之一：
//     USART1，USART2，USART3，UART4 或 UART5
// 输入参数  USART_IT：待使能或者失能的USART中断源
//     可以是下列值之一：
//           USART_IT_CTS：  	CTS 变化中断(UART4 和 UART5不可用)
//           USART_IT_LBD：  	LIN 断开检测中断
//           USART_IT_TXE：  	发送数据寄存器空中断
//           USART_IT_TC：   	发送完成中断
//           USART_IT_RXNE：	接收数据寄存器非空中断
//           USART_IT_IDLE： 	空闲帧检测中断
//           USART_IT_PE：   	校验错误中断
//           USART_IT_ERR：  	错误中断(帧错误，噪声错误，上溢错误)
// 输入参数  NewState：外设USARTx的新状态
//     可以是：ENABLE或DISABLE
// 返回值  无
-----------------------------------------------------------------------------------------------------*/
void USART_ITConfig(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState)
{
  uint32_t usartreg = 0x00, itpos = 0x00, itmask = 0x00;
  uint32_t usartxbase = 0x00;
  /* 检查输入参数 */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CONFIG_IT(USART_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  /* CTS中断对UART4和UART5是无效的 */
  if (USART_IT == USART_IT_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  }   
 
  usartxbase = (uint32_t)USARTx;
 
  /* 获得USART寄存器索引 */
  usartreg = (((uint8_t)USART_IT) >> 0x05);
 
  /* 获得中断位置 */
  itpos = USART_IT & IT_Mask;
  itmask = (((uint32_t)0x01) << itpos);
 
  if (usartreg == 0x01) /* The IT is in CR1 register */
  {
    usartxbase += 0x0C;
  }
  else if (usartreg == 0x02) /* The IT is in CR2 register */
  {
    usartxbase += 0x10;
  }
  else /* 在CR3中的中断 */
  {
    usartxbase += 0x14; 
  }
  if (NewState != DISABLE)
  {
    *(__IO uint32_t*)usartxbase  |= itmask;
  }
  else
  {
    *(__IO uint32_t*)usartxbase &= ~itmask;
  }
}
/**
  * @brief  Enables or disables the USART抯 DMA interface.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_DMAReq: specifies the DMA request.
  *   This parameter can be any combination of the following values:
  *     @arg USART_DMAReq_Tx: USART DMA transmit request
  *     @arg USART_DMAReq_Rx: USART DMA receive request
  * @param  NewState: new state of the DMA Request sources.
  *   This parameter can be: ENABLE or DISABLE.
  * @note The DMA mode is not available for UART5 except in the STM32
  *       High density value line devices(STM32F10X_HD_VL).  
  * @retval None
  */
void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DMAREQ(USART_DMAReq));  
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 
  if (NewState != DISABLE)
  {
    /* Enable the DMA transfer for selected requests by setting the DMAT and/or
       DMAR bits in the USART CR3 register */
    USARTx->CR3 |= USART_DMAReq;
  }
  else
  {
    /* Disable the DMA transfer for selected requests by clearing the DMAT and/or
       DMAR bits in the USART CR3 register */
    USARTx->CR3 &= (uint16_t)~USART_DMAReq;
  }
}

/**
  * @brief  Sets the address of the USART node.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_Address: Indicates the address of the USART node.
  * @retval None
  */
void USART_SetAddress(USART_TypeDef* USARTx, uint8_t USART_Address)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_ADDRESS(USART_Address)); 
    
  /* Clear the USART address */
  USARTx->CR2 &= CR2_Address_Mask;
  /* Set the USART address node */
  USARTx->CR2 |= USART_Address;
}

/**
  * @brief  Selects the USART WakeUp method.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_WakeUp: specifies the USART wakeup method.
  *   This parameter can be one of the following values:
  *     @arg USART_WakeUp_IdleLine: WakeUp by an idle line detection
  *     @arg USART_WakeUp_AddressMark: WakeUp by an address mark
  * @retval None
  */
void USART_WakeUpConfig(USART_TypeDef* USARTx, uint16_t USART_WakeUp)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_WAKEUP(USART_WakeUp));
  
  USARTx->CR1 &= CR1_WAKE_Mask;
  USARTx->CR1 |= USART_WakeUp;
}

/**
  * @brief  Determines if the USART is in mute mode or not.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART mute mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 
  
  if (NewState != DISABLE)
  {
    /* Enable the USART mute mode  by setting the RWU bit in the CR1 register */
    USARTx->CR1 |= CR1_RWU_Set;
  }
  else
  {
    /* Disable the USART mute mode by clearing the RWU bit in the CR1 register */
    USARTx->CR1 &= CR1_RWU_Reset;
  }
}

/**
  * @brief  Sets the USART LIN Break detection length.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_LINBreakDetectLength: specifies the LIN break detection length.
  *   This parameter can be one of the following values:
  *     @arg USART_LINBreakDetectLength_10b: 10-bit break detection
  *     @arg USART_LINBreakDetectLength_11b: 11-bit break detection
  * @retval None
  */
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, uint16_t USART_LINBreakDetectLength)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_LIN_BREAK_DETECT_LENGTH(USART_LINBreakDetectLength));
  
  USARTx->CR2 &= CR2_LBDL_Mask;
  USARTx->CR2 |= USART_LINBreakDetectLength;  
}

/**
  * @brief  Enables or disables the USART抯 LIN mode.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART LIN mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the LIN mode by setting the LINEN bit in the CR2 register */
    USARTx->CR2 |= CR2_LINEN_Set;
  }
  else
  {
    /* Disable the LIN mode by clearing the LINEN bit in the CR2 register */
    USARTx->CR2 &= CR2_LINEN_Reset;
  }
}

/**
  * @brief  Transmits single data through the USARTx peripheral.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  Data: the data to transmit.
  * @retval None
  */
/*-----------------------------------------------------------------------------------------------------
// 功能描述  通过外设USARTx发送单个数据
// 输入参数  USARTx：选择USART或 UART 外设
//     可以是下列值之一：
//     USART1, USART2, USART3, UART4 或 UART5
// 输入参数  Data：待发送的数据
// 返回值  无
-----------------------------------------------------------------------------------------------------*/
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
  /* 检查输入参数 */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data)); 
 
  /* 发送数据 */
  USARTx->DR = (Data & (uint16_t)0x01FF);
}
/**
  * @brief  Returns the most recent received data by the USARTx peripheral.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @retval The received data.
  */
/*-----------------------------------------------------------------------------------------------------
// 功能描述  返回USARTx最近接收到的数据
// 输入参数  USARTx：选择USART或 UART 外设
//     可以是下列值之一：
//     USART1, USART2, USART3, UART4 或 UART5
// 返回值  接收到的数据
-----------------------------------------------------------------------------------------------------*/
uint16_t USART_ReceiveData(USART_TypeDef* USARTx)
{
  /* 检查输入参数 */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
 
  /* 接收数据 */
  return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);
}

/**
  * @brief  Transmits break characters.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @retval None
  */
void USART_SendBreak(USART_TypeDef* USARTx)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  
  /* Send break characters */
  USARTx->CR1 |= CR1_SBK_Set;
}

/**
  * @brief  Sets the specified USART guard time.
  * @param  USARTx: where x can be 1, 2 or 3 to select the USART peripheral.
  * @param  USART_GuardTime: specifies the guard time.
  * @note The guard time bits are not available for UART4 and UART5.   
  * @retval None
  */
void USART_SetGuardTime(USART_TypeDef* USARTx, uint8_t USART_GuardTime)
{    
  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));
  
  /* Clear the USART Guard time */
  USARTx->GTPR &= GTPR_LSB_Mask;
  /* Set the USART guard time */
  USARTx->GTPR |= (uint16_t)((uint16_t)USART_GuardTime << 0x08);
}

/**
  * @brief  Sets the system clock prescaler.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_Prescaler: specifies the prescaler clock.  
  * @note   The function is used for IrDA mode with UART4 and UART5.
  * @retval None
  */
void USART_SetPrescaler(USART_TypeDef* USARTx, uint8_t USART_Prescaler)
{ 
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  
  /* Clear the USART prescaler */
  USARTx->GTPR &= GTPR_MSB_Mask;
  /* Set the USART prescaler */
  USARTx->GTPR |= USART_Prescaler;
}

/**
  * @brief  Enables or disables the USART抯 Smart Card mode.
  * @param  USARTx: where x can be 1, 2 or 3 to select the USART peripheral.
  * @param  NewState: new state of the Smart Card mode.
  *   This parameter can be: ENABLE or DISABLE.     
  * @note The Smart Card mode is not available for UART4 and UART5. 
  * @retval None
  */
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the SC mode by setting the SCEN bit in the CR3 register */
    USARTx->CR3 |= CR3_SCEN_Set;
  }
  else
  {
    /* Disable the SC mode by clearing the SCEN bit in the CR3 register */
    USARTx->CR3 &= CR3_SCEN_Reset;
  }
}

/**
  * @brief  Enables or disables NACK transmission.
  * @param  USARTx: where x can be 1, 2 or 3 to select the USART peripheral. 
  * @param  NewState: new state of the NACK transmission.
  *   This parameter can be: ENABLE or DISABLE.  
  * @note The Smart Card mode is not available for UART4 and UART5.
  * @retval None
  */
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));  
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the NACK transmission by setting the NACK bit in the CR3 register */
    USARTx->CR3 |= CR3_NACK_Set;
  }
  else
  {
    /* Disable the NACK transmission by clearing the NACK bit in the CR3 register */
    USARTx->CR3 &= CR3_NACK_Reset;
  }
}

/**
  * @brief  Enables or disables the USART抯 Half Duplex communication.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART Communication.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the Half-Duplex mode by setting the HDSEL bit in the CR3 register */
    USARTx->CR3 |= CR3_HDSEL_Set;
  }
  else
  {
    /* Disable the Half-Duplex mode by clearing the HDSEL bit in the CR3 register */
    USARTx->CR3 &= CR3_HDSEL_Reset;
  }
}


/**
  * @brief  Enables or disables the USART's 8x oversampling mode.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART one bit sampling methode.
  *   This parameter can be: ENABLE or DISABLE.
  * @note
  *     This function has to be called before calling USART_Init()
  *     function in order to have correct baudrate Divider value.   
  * @retval None
  */
void USART_OverSampling8Cmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the 8x Oversampling mode by setting the OVER8 bit in the CR1 register */
    USARTx->CR1 |= CR1_OVER8_Set;
  }
  else
  {
    /* Disable the 8x Oversampling mode by clearing the OVER8 bit in the CR1 register */
    USARTx->CR1 &= CR1_OVER8_Reset;
  }
}

/**
  * @brief  Enables or disables the USART's one bit sampling methode.
  * @param  USARTx: Select the USART or the UART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the USART one bit sampling methode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_OneBitMethodCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the one bit method by setting the ONEBITE bit in the CR3 register */
    USARTx->CR3 |= CR3_ONEBITE_Set;
  }
  else
  {
    /* Disable tthe one bit method by clearing the ONEBITE bit in the CR3 register */
    USARTx->CR3 &= CR3_ONEBITE_Reset;
  }
}

/**
  * @brief  Configures the USART抯 IrDA interface.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IrDAMode: specifies the IrDA mode.
  *   This parameter can be one of the following values:
  *     @arg USART_IrDAMode_LowPower
  *     @arg USART_IrDAMode_Normal
  * @retval None
  */
void USART_IrDAConfig(USART_TypeDef* USARTx, uint16_t USART_IrDAMode)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_IRDA_MODE(USART_IrDAMode));
    
  USARTx->CR3 &= CR3_IRLP_Mask;
  USARTx->CR3 |= USART_IrDAMode;
}

/**
  * @brief  Enables or disables the USART抯 IrDA interface.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  NewState: new state of the IrDA mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
    
  if (NewState != DISABLE)
  {
    /* Enable the IrDA mode by setting the IREN bit in the CR3 register */
    USARTx->CR3 |= CR3_IREN_Set;
  }
  else
  {
    /* Disable the IrDA mode by clearing the IREN bit in the CR3 register */
    USARTx->CR3 &= CR3_IREN_Reset;
  }
}

/**
  * @brief  Checks whether the specified USART flag is set or not.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg USART_FLAG_CTS:  CTS Change flag (not available for UART4 and UART5)
  *     @arg USART_FLAG_LBD:  LIN Break detection flag
  *     @arg USART_FLAG_TXE:  Transmit data register empty flag
  *     @arg USART_FLAG_TC:   Transmission Complete flag
  *     @arg USART_FLAG_RXNE: Receive data register not empty flag
  *     @arg USART_FLAG_IDLE: Idle Line detection flag
  *     @arg USART_FLAG_ORE:  OverRun Error flag
  *     @arg USART_FLAG_NE:   Noise Error flag
  *     @arg USART_FLAG_FE:   Framing Error flag
  *     @arg USART_FLAG_PE:   Parity Error flag
  * @retval The new state of USART_FLAG (SET or RESET).
  */
/*-----------------------------------------------------------------------------------------------------
// 功能描述  检查指定的USART标志位设置与否
// 输入参数  USARTx：选择USART或 UART 外设
//     可以是下列值之一：
//     USART1，USART2，USART3，UART4 或 UART5
// 输入参数  USART_FLAG：待检查的USART标志位
//     可以是下列值之一：
//     USART_ FLAG _CTS：  	CTS 变化标志(UART4 和 UART5不可用)
//     USART_ FLAG _LBD：  	LIN 断开检测标志
//     USART_ FLAG _TXE：  	发送数据寄存器空标志
//     USART_ FLAG _TC：   	发送完成标志
//     USART_ FLAG _RXNE：	接收数据寄存器非空标志
//     USART_ FLAG _IDLE： 	空闲帧检测标志
//     USART_ FLAG _ORE：	上溢错误标志
//     USART_ FLAG _NE：   	噪声错误标志
//     USART_ FLAG _FE：   	帧错误标志
//     USART_ FLAG _PE：   	校验错误标志
// 返回值  USART_FLAG的新状态（SET或者RESET）
-----------------------------------------------------------------------------------------------------*/
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* 检查输入参数 */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_FLAG(USART_FLAG));
  /* CTS标志对UART4和UART5是无效的 */
  if (USART_FLAG == USART_FLAG_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  }  
 
  if ((USARTx->SR & USART_FLAG) != (uint16_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}
/**
  * @brief  Clears the USARTx's pending flags.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_FLAG: specifies the flag to clear.
  *   This parameter can be any combination of the following values:
  *     @arg USART_FLAG_CTS:  CTS Change flag (not available for UART4 and UART5).
  *     @arg USART_FLAG_LBD:  LIN Break detection flag.
  *     @arg USART_FLAG_TC:   Transmission Complete flag.
  *     @arg USART_FLAG_RXNE: Receive data register not empty flag.
  *   
  * @note
  *   - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun 
  *     error) and IDLE (Idle line detected) flags are cleared by software 
  *     sequence: a read operation to USART_SR register (USART_GetFlagStatus()) 
  *     followed by a read operation to USART_DR register (USART_ReceiveData()).
  *   - RXNE flag can be also cleared by a read to the USART_DR register 
  *     (USART_ReceiveData()).
  *   - TC flag can be also cleared by software sequence: a read operation to 
  *     USART_SR register (USART_GetFlagStatus()) followed by a write operation
  *     to USART_DR register (USART_SendData()).
  *   - TXE flag is cleared only by a write to the USART_DR register 
  *     (USART_SendData()).
  * @retval None
  */
/*-----------------------------------------------------------------------------------------------------
// 功能描述  清除USARTx的挂起标志
// 输入参数  USARTx：选择USART或 UART 外设
//     可以是下列值之一：
//     USART1，USART2，USART3，UART4 或 UART5
// 输入参数  USART_FLAG：待清除的USART标志
//     可以是下列值的任意组合：
//     USART_ FLAG _CTS：  	CTS 变化标志(UART4 和 UART5不可用)
//     USART_ FLAG _LBD：  	LIN 断开检测标志
//     USART_ FLAG _TC：   	发送完成标志
//     USART_ FLAG _RXNE：	接收数据寄存器非空标志
// 
// 注意
//   - PE(校验错误)，FE(帧错误)，NE(噪声错误)，ORE(上溢错误) 
//     和IDLE(空闲帧检测)标志通过软件时序被清除： 
//     一个对USART_SR 寄存器的读操作(USART_GetFlagStatus()) 
//     后跟一个对USART_DR 寄存器的读操作 (USART_ReceiveData())。
//   - RXNE 标志也可以通过读USART_DR 寄存器被清除(USART_ReceiveData())。
//   - TC 标志也可以通过软件时序被清除： 
//     一个对USART_SR 寄存器的读操作(USART_GetFlagStatus()) 
//     后跟一个对USART_DR 寄存器的写操作(USART_SendData())。
//   - TXE标志只能通过写USART_DR 寄存器被清除(USART_SendData())。
// 返回值  无
-----------------------------------------------------------------------------------------------------*/
void USART_ClearFlag(USART_TypeDef* USARTx, uint16_t USART_FLAG)
{
  /* 检查输入参数 */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CLEAR_FLAG(USART_FLAG));
  /* CTS标志对UART4和UART5是无效的 */
  if ((USART_FLAG & USART_FLAG_CTS) == USART_FLAG_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  } 
 
  USARTx->SR = (uint16_t)~USART_FLAG;
}

/**
  * @brief  Checks whether the specified USART interrupt has occurred or not.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the USART interrupt source to check.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TXE:  Tansmit Data Register empty interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *     @arg USART_IT_IDLE: Idle line detection interrupt
  *     @arg USART_IT_ORE:  OverRun Error interrupt
  *     @arg USART_IT_NE:   Noise Error interrupt
  *     @arg USART_IT_FE:   Framing Error interrupt
  *     @arg USART_IT_PE:   Parity Error interrupt
  * @retval The new state of USART_IT (SET or RESET).
  */
/*-----------------------------------------------------------------------------------------------------
// 功能描述  检查指定的USART中断发生与否
// 输入参数  USARTx：选择USART或 UART 外设
//     可以是下列值之一：
//     USART1，USART2，USART3，UART4 或 UART5
// 输入参数  USART_IT：待检查的USART中断源
//     可以是下列值之一：
//     USART_ IT _CTS：  	CTS 变化中断(UART4 和 UART5不可用)
//     USART_ IT _LBD：  	LIN 断开检测中断
//     USART_ IT _TXE：  	发送数据寄存器空中断
//     USART_ IT _TC：   	发送完成中断
//     USART_ IT _RXNE：	接收数据寄存器非空中断
//     USART_ IT _IDLE： 	空闲帧检测中断
//     USART_ IT _ORE：	上溢错误中断
//     USART_ IT _NE：   	噪声错误中断
//     USART_ IT _FE：   	帧错误中断
//     USART_ IT _PE：   	校验错误中断
// 返回值  USART_FLAG的新状态（SET或者RESET）
-----------------------------------------------------------------------------------------------------*/
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, uint16_t USART_IT)
{
  uint32_t bitpos = 0x00, itmask = 0x00, usartreg = 0x00;
  ITStatus bitstatus = RESET;
  /* 检查输入参数 */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_GET_IT(USART_IT));
  /* CYS中断对UART4和UART5是无效的 */ 
  if (USART_IT == USART_IT_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  }   
 
  /* 获得USART寄存器索引 */
  usartreg = (((uint8_t)USART_IT) >> 0x05);
  /* 获得中断位置 */
  itmask = USART_IT & IT_Mask;
  itmask = (uint32_t)0x01 << itmask;
 
  if (usartreg == 0x01) /* The IT  is in CR1 register */
  {
    itmask &= USARTx->CR1;
  }
  else if (usartreg == 0x02) /* The IT  is in CR2 register */
  {
    itmask &= USARTx->CR2;
  }
  else /* 在CR3中的中断 */
  {
    itmask &= USARTx->CR3;
  }
 
  bitpos = USART_IT >> 0x08;
  bitpos = (uint32_t)0x01 << bitpos;
  bitpos &= USARTx->SR;
  if ((itmask != (uint16_t)RESET)&&(bitpos != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
 
  return bitstatus;  
}
/**
  * @brief  Clears the USARTx抯 interrupt pending bits.
  * @param  USARTx: Select the USART or the UART peripheral. 
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3, UART4 or UART5.
  * @param  USART_IT: specifies the interrupt pending bit to clear.
  *   This parameter can be one of the following values:
  *     @arg USART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *     @arg USART_IT_LBD:  LIN Break detection interrupt
  *     @arg USART_IT_TC:   Transmission complete interrupt. 
  *     @arg USART_IT_RXNE: Receive Data register not empty interrupt.
  *   
  * @note
  *   - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun 
  *     error) and IDLE (Idle line detected) pending bits are cleared by 
  *     software sequence: a read operation to USART_SR register 
  *     (USART_GetITStatus()) followed by a read operation to USART_DR register 
  *     (USART_ReceiveData()).
  *   - RXNE pending bit can be also cleared by a read to the USART_DR register 
  *     (USART_ReceiveData()).
  *   - TC pending bit can be also cleared by software sequence: a read 
  *     operation to USART_SR register (USART_GetITStatus()) followed by a write 
  *     operation to USART_DR register (USART_SendData()).
  *   - TXE pending bit is cleared only by a write to the USART_DR register 
  *     (USART_SendData()).
  * @retval None
  */
/*-----------------------------------------------------------------------------------------------------
// 功能描述  清除USARTx的中断挂起位
// 输入参数  USARTx：选择USART或 UART 外设
//     可以是下列值之一：
//     USART1，USART2，USART3，UART4 或 UART5
// 输入参数  USART_IT：待清除的USART中断挂起位
//     可以是下列值之一：
//     USART_ IT _CTS：  	CTS 变化中断(UART4 和 UART5不可用)
//     USART_ IT _LBD：  	LIN 断开检测中断
//     USART_ IT _TC：   	发送完成中断
//     USART_ IT _RXNE：	接收数据寄存器非空中断
// 
// 注意
//   - PE(校验错误)，FE(帧错误)，NE(噪声错误)，ORE(上溢错误) 
//     和IDLE(空闲帧检测)挂起位通过软件时序被清除： 
//     一个对USART_SR 寄存器的读操作(USART_GetFlagStatus()) 
//     后跟一个对USART_DR 寄存器的读操作 (USART_ReceiveData())。
//   - RXNE 挂起位也可以通过读USART_DR 寄存器被清除(USART_ReceiveData())。
//   - TC 挂起位也可以通过软件时序被清除： 
//     一个对USART_SR 寄存器的读操作(USART_GetFlagStatus()) 
//     后跟一个对USART_DR 寄存器的写操作(USART_SendData())。
//   - TXE标志只能通过写USART_DR 寄存器被清除(USART_SendData())。
// 返回值  无
-----------------------------------------------------------------------------------------------------*/
void USART_ClearITPendingBit(USART_TypeDef* USARTx, uint16_t USART_IT)
{
  uint16_t bitpos = 0x00, itmask = 0x00;
  /* 检查输入参数 */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CLEAR_IT(USART_IT));
  /* CTS中断对UART4和UART5是无效的 */
  if (USART_IT == USART_IT_CTS)
  {
    assert_param(IS_USART_123_PERIPH(USARTx));
  }   
 
  bitpos = USART_IT >> 0x08;
  itmask = ((uint16_t)0x01 << (uint16_t)bitpos);
  USARTx->SR = (uint16_t)~itmask;
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

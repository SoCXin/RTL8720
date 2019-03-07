

#include <stdio.h>
#include "stm32f10x.h"
#include "includes.h"

#define  GPIOx                    GPIOA
#define  GPIO_RTSPin              GPIO_Pin_1
#define  GPIO_CTSPin              GPIO_Pin_0
#define  GPIO_TxPin               GPIO_Pin_2
#define  GPIO_RxPin               GPIO_Pin_3

USART_InitTypeDef USART_InitStructure;

/**
  * 配置USART2时钟
  */
void RCC_Configuration(void)
{    
  /* 使能 GPIOA 和功能复用时 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* Enable USART2 clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USART2 RTS and USART2 Tx as alternate function push-pull */
  ///GPIO_InitStructure.GPIO_Pin = GPIO_RTSPin | GPIO_TxPin;
  GPIO_InitStructure.GPIO_Pin = GPIO_TxPin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOx, &GPIO_InitStructure);

  /* Configure USART2 CTS and USART2 Rx as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_CTSPin | GPIO_RxPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void Usart2Init(void)
{
  RCC_Configuration();

  GPIO_Configuration();
  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);

  /* Enable the USART2 */
  USART_Cmd(USART2, ENABLE);

/* Communication hyperterminal-USART2 using hardware flow control -------------*/
  /* Send a buffer from USART to hyperterminal */    
}

void PutUsart2(u8 ch)
{
  USART_SendData(USART2,ch);
  while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);      
}

void Usart1Rcc(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
}

void Usart1GPIO(void)
{
  GPIO_InitTypeDef usart1;
 
  usart1.GPIO_Pin = GPIO_Pin_9;
  usart1.GPIO_Speed = GPIO_Speed_50MHz;
  usart1.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &usart1);

  usart1.GPIO_Pin = GPIO_Pin_10;
  usart1.GPIO_Speed = GPIO_Speed_50MHz;
  usart1.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &usart1);
}

void Usart1Init(void)
{
  USART_InitTypeDef usart1;
  
  Usart1Rcc();
  Usart1GPIO();
  
  usart1.USART_BaudRate = 115200;
  usart1.USART_WordLength = USART_WordLength_8b;
  usart1.USART_WordLength = USART_StopBits_1;
  usart1.USART_Parity = USART_Parity_No;
  usart1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart1.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &usart1);
  USART_Cmd(USART1, ENABLE);
}

void PutUsart1(u8 ch)
{
  USART_SendData(USART1, ch);
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
}

#define BUFFMAX_SIZE  200  //缓冲区长度

int Put_Uart(INT8U *q);	//写入串口缓冲区   字符串指针 字符串个数


 /*********************************************************
函数介绍：
入口参数：
出口参数：
**********************************************************/
struct buffer_uart{
	char uart_data[BUFFMAX_SIZE];
	int  frount;
	int  rear;
	int  cunter;	//缓冲区计数器
};
	
volatile struct buffer_uart buf = {0,0,0};
int cun;

/*********************************************************
函数介绍： 缓冲区初始化
入口参数：
出口参数：
**********************************************************/
void Init_Queue(void)
{
  buf.frount = 0;
  buf.rear   = 0;
  buf.cunter = 0;
}

/*********************************************************
函数介绍： 将字符串写入缓冲区
入口参数： char *p 指针
出口参数： 返回写入字符串个数
**********************************************************/
int Put_Uart(INT8U *q)
{	
  int i = 0;
  while((*q) != 0x00){
    while(((buf.rear+1)%BUFFMAX_SIZE == buf.frount))	;	//队列满就等待	空循环			
    buf.uart_data[buf.rear] = *q;
    buf.rear = (buf.rear + 1) % BUFFMAX_SIZE;
    buf.cunter++;
    q++;
    i++;
  }

  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);	//开缓冲区中断 
  return i;
}

int Put_Uarts(INT8U *q,INT32S number)
{	
  int i = 0;
  while(i<number){
    while(((buf.rear+1)%BUFFMAX_SIZE == buf.frount))	;	//队列满就等待	空循环			
    buf.uart_data[buf.rear] = *q;
    buf.rear = (buf.rear+1) % BUFFMAX_SIZE;
    buf.cunter++;
    q++;
    i++;
  }

  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);	//开缓冲区中断 
  return i;
}

void USART1_IRQHandler(void)		 //串口中断
{
  OS_CPU_SR    cpu_sr = 0;
  OS_ENTER_CRITICAL() ;

  if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET){
    if(buf.frount == buf.rear)	//队列空关中断
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);	//关缓冲区中断
    else{
      USART_SendData(USART1, buf.uart_data[buf.frount]);
      buf.frount = (buf.frount + 1) % BUFFMAX_SIZE;
      buf.cunter--;
      cun = buf.cunter;
    }
  }
  OS_EXIT_CRITICAL()  ; 
}

void NVIC_cfg(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
      
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;     	  //选择串口 中断
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应式中断优先级设置为0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //使能中断
  NVIC_Init(&NVIC_InitStructure);
}

/*----------------------------------------------------------------------------
  SendChar
  Write character to Serial Port.
 *----------------------------------------------------------------------------*/
int SendChar (int ch)  
{
  while (!(USART1->SR & USART_FLAG_TXE));
  USART1->DR = (ch & 0x1FF);
  return (ch);
}

/*----------------------------------------------------------------------------
  GetKey
  Read character to Serial Port.
 *----------------------------------------------------------------------------*/
int GetKey (void)  
{
  while (!(USART1->SR & USART_FLAG_RXNE));
  return ((int)(USART1->DR & 0x1FF));
}



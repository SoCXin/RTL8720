
#ifndef USART_H
#define USART_H
#include "includes.h"

void Usart2Init(void) ;	 
void Usart1Init(void);
void PutUsart1( u8 ch)  ;
void PutUsart2( u8 ch);
int Put_Uarts(u8 *q,INT32S number);
int Put_Uart(u8 *q);
void NVIC_cfg(void);

#endif


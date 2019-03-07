
#ifndef _DRIVER_USART_H_
#define _DRIVER_USART_H_

#define USART_RECDATAMAX 2
#define USART_FRMHEAD	's'
#define USART_FRMTAIL	'w'

extern u8 UsartRecData[USART_RECDATAMAX];
extern u8 UsartRecFinish;
extern void USART_Conf(void);
extern void Usart_Puts(const char * str);
extern void Usart_TransmitData(char * str , u32 len);

extern int SendChar (int ch);
extern void PrintfU(USART_TypeDef* USARTx, uint8_t *Data,...);
extern char *itoa(int value, char *string, int radix);
#endif

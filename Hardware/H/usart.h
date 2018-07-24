#ifndef __USART_H_
#define __USART_H_

#include "stm32f10x.h"
#include <stdio.h>

#define CONTROL_ON  1
#define DEBUG_PRINT 0
#define SEND_TO_ANO 1

void UART1NVIC_Configuration(void);
void USART_Config(void);
u8 fPutChar(u8 ch);
void fPutString(u8 *buf, u8 len);
void USART1_IRQHandler(void);

#endif

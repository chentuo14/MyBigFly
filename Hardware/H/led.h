#ifndef __LED_H_
#define __LED_H_

#include "stm32f10x.h"

#define  LED1_On()  	GPIO_SetBits(GPIOA, GPIO_Pin_0)
#define  LED1_Off()   	GPIO_ResetBits(GPIOA, GPIO_Pin_0)

void LED_Config(void);

#endif

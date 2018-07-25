#ifndef __LED_H_
#define __LED_H_

#include "stm32f10x.h"

#define  PB0_On()  		GPIO_SetBits(GPIOB, GPIO_Pin_0)
#define  PB0_Off()   	GPIO_ResetBits(GPIOB, GPIO_Pin_0)

#define  PB1_On()  		GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define  PB1_Off()   	GPIO_ResetBits(GPIOB, GPIO_Pin_1)

void LED_Config(void);

#endif

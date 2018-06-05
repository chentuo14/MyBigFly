#ifndef __TIMER_H_
#define __TIMER_H_

#include "stm32f10x.h"
#include "control.h"

extern struct MY_CONTROL myControl;

void TIM2_CAP_Init(u16 arr, u16 psc);
void TIM2_IRQHandler(void);

#endif

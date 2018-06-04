#ifndef _SYS_CLOCK_H_
#define _SYS_CLOCK_H_

#include "stm32f10x.h"

void SetSysClockToHSE(void);
void MYRCC_DeInit(void);
char SystemClock_HSE(u8 PLL);

void cycleCounterInit(void);
void DelayMs(uint16_t nms);
void delay_us(u32 nus);
void delay_ms(uint16_t nms);
uint32_t micros(void);

#endif

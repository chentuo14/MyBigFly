#ifndef __MS5611_H_
#define __MS5611_H_

#include "stm32f10x.h"

#define MS5611_ADDR 0x77

#define MS5611_RESET 0x1E
#define MS5611_D1 	 0x40
#define MS5611_D2    0x50
#define MS5611_D1D2_SIZE 3

#define MS5611_OSR_256		0x00
#define MS5611_OSR_512		0x02
#define MS5611_OSR_1024		0x04
#define MS5611_OSR_2048		0x06
#define MS5611_OSR_4096		0x08

//by adding ints from 0 to 6 we can read all the prom configuration values.
#define MS5611_PROM_BASE_ADDR 0xA0
#define MS5611_PROM_REG_COUNT 8
#define MS5611_PROM_REG_SIZE  2

u8 MS5611_Command(u8 command);
void MS5611_Reset(void);
u16 MS5611_Read_PROM(u8 reg);
u32 MS5611_Read_Press(u8 osr);
u32 MS5611_Read_Temp(u8 osr);
int MS5611_CRC(void);
int MS5611_Init(void);
void MS5611_GetValue(float *temperature, float *press);
void PrintTempAndPress(void);

#endif

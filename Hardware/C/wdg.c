#include "wdg.h"

void IWDG_Conig(void)
{		
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);			//解除寄存器保护
	IWDG_SetPrescaler(IWDG_Prescaler_256);					//写入初始化分频值0-7
	IWDG_SetReload(0xFFF);									//0xfff*256/40k=26s
	IWDG_ReloadCounter();									//开启寄存器保护
	IWDG_Enable();											//启动看门狗
}

void IWDG_Feed(void)
{
	IWDG->KR = 0xAAAA;
}





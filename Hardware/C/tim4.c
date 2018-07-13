
#include "tim4.h"
#include "usart.h"
#include "control.h"

extern struct MY_CONTROL myControl;

void TIM4_CAP_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	//开启TIM2和GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//PA0初始化
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;			//下拉输入
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	//TIM2定时器初始化
	TIM_TimeBaseInitStruct.TIM_Period = arr;
	TIM_TimeBaseInitStruct.TIM_Prescaler = psc;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;		
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	
	//TIM4_CH3输入捕获初始化
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStruct.TIM_ICFilter = 0x00;							//不滤波
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;		//上升沿捕获
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;				//输入器不分频
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;	//映射到IC1
	TIM_ICInit(TIM4, &TIM_ICInitStruct);
	
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStruct.TIM_ICFilter = 0x00;							//不滤波
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;		//上升沿捕获
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;				//输入器不分频
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;	//映射到IC2
	TIM_ICInit(TIM4, &TIM_ICInitStruct);
	
	//中断分组初始化
	NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStruct);
	
	TIM_ITConfig(TIM4, TIM_IT_CC3|TIM_IT_CC4, ENABLE);			//更新中断和CC1IE捕获中断
	
	TIM_Cmd(TIM4, ENABLE);
#if DEBUG_PRINT
	printf("Init TIM4 CAP success\n");
#endif
}

u8 CAPTURE_STA_TIM4CH[2] = {0};
u16 CAPTURE_VAL_TIM4CH[2];
u16 CAPTURE_UP_TIM4CH[2], CAPTURE_DOWN_TIM4CH[2];
void TIM4_IRQHandler(void)
{
	if((CAPTURE_STA_TIM4CH[0]&0x80) == 0) {						//还未捕获成功
		if(TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET) {		//捕获3发生捕获事件
			if(CAPTURE_STA_TIM4CH[0]&0x40) {					//捕获到一个下降沿
				CAPTURE_STA_TIM4CH[0] |= 0x80;					//标记成功捕获到一次高电平脉宽
				CAPTURE_DOWN_TIM4CH[0] = TIM_GetCapture3(TIM4);
				if(CAPTURE_DOWN_TIM4CH[0] >= CAPTURE_UP_TIM4CH[0]) 
					CAPTURE_VAL_TIM4CH[0] = CAPTURE_DOWN_TIM4CH[0] - CAPTURE_UP_TIM4CH[0];
				else 
					CAPTURE_VAL_TIM4CH[0] = 0xffff + CAPTURE_DOWN_TIM4CH[0] - CAPTURE_UP_TIM4CH[0];
				TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising);	//CC1P=0 设置为上升沿捕获
			} else {												//还未开始，第一次捕获上升沿
				CAPTURE_STA_TIM4CH[0] = 0;							//清空
				CAPTURE_VAL_TIM4CH[0] = 0;
				CAPTURE_UP_TIM4CH[0] = TIM_GetCapture3(TIM4);
				CAPTURE_STA_TIM4CH[0] |= 0x40;						//标记捕获到了上升沿
				TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			}	
			TIM_ClearFlag(TIM4, TIM_FLAG_CC3);								//清除状态标志
		}
	} 
	
	if((CAPTURE_STA_TIM4CH[1]&0x80) == 0) {							//还未捕获成功
		if(TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET) {			//捕获4发生捕获事件
			if(CAPTURE_STA_TIM4CH[1]&0x40) {						//捕获到一个下降沿
				CAPTURE_STA_TIM4CH[1] |= 0x80;						//标记成功捕获到一次高电平脉宽
				CAPTURE_DOWN_TIM4CH[1] = TIM_GetCapture4(TIM4);		//获取捕获2计数
				if(CAPTURE_DOWN_TIM4CH[1] >= CAPTURE_UP_TIM4CH[1])
					CAPTURE_VAL_TIM4CH[1] = CAPTURE_DOWN_TIM4CH[1] - CAPTURE_UP_TIM4CH[1];
				else
					CAPTURE_VAL_TIM4CH[1] = 0xffff + CAPTURE_DOWN_TIM4CH[1] - CAPTURE_UP_TIM4CH[1];
				TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising);	//CC1P=0 设置为上升沿捕获
			} else {												//还未开始，第一次捕获上升沿
				CAPTURE_STA_TIM4CH[1] = 0;							//清空
				CAPTURE_VAL_TIM4CH[1] = 0;
				CAPTURE_UP_TIM4CH[1] = TIM_GetCapture4(TIM4);
				CAPTURE_STA_TIM4CH[1] |= 0x40;				//标记捕获到了上升沿
				TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Falling);	//CC1P=1 设置为下降沿捕获
			}
			TIM_ClearFlag(TIM4, TIM_FLAG_CC4);								//清除状态标志	
		}
	}
	
	if(CAPTURE_STA_TIM4CH[0]&0x80) {								//成功捕获到了一次上升沿											//溢出时间总和
		myControl.remoteSwitch[0] = CAPTURE_VAL_TIM4CH[0];							//得到总的高电平时间
//		printf("TIM4 CH3:%d\t", myControl.remoteSwitch[0]);
		CAPTURE_STA_TIM4CH[0] = 0;
	} 
	if(CAPTURE_STA_TIM4CH[1]&0x80) {								//成功捕获到了一次上升沿											//溢出时间总和
		myControl.remoteSwitch[1] = CAPTURE_VAL_TIM4CH[1];							//得到总的高电平时间
//		printf("TIM4 CH4:%d\t", temp[1]);
		CAPTURE_STA_TIM4CH[1] = 0;
	} 

}


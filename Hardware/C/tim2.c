
#include "tim2.h"
#include "usart.h"

//arr:自动重装值 psc:时钟预分频数
//定时器溢出时间计算：Tout=((arr+1)*(psc+1))/Ft   us
//Ft = 定时器工作频率，单位Mhz
//TIM2在APB1上，为HCLK/2   36Mhz
void TIM2_CAP_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	//开启TIM2和GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//PA0初始化
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;			//下拉输入
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//TIM2定时器初始化
	TIM_TimeBaseInitStruct.TIM_Period = arr;
	TIM_TimeBaseInitStruct.TIM_Prescaler = psc;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;		
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	
	//TIM2_CH1输入捕获初始化
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICFilter = 0x00;							//不滤波
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;		//上升沿捕获
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;				//输入器不分频
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;	//映射到IC1
	TIM_ICInit(TIM2, &TIM_ICInitStruct);
	
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStruct.TIM_ICFilter = 0x00;							//不滤波
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;		//上升沿捕获
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;				//输入器不分频
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;	//映射到IC2
	TIM_ICInit(TIM2, &TIM_ICInitStruct);
	
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStruct.TIM_ICFilter = 0x00;							//不滤波
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;		//上升沿捕获
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;				//输入器不分频
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;	//映射到IC3
	TIM_ICInit(TIM2, &TIM_ICInitStruct);
	
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStruct.TIM_ICFilter = 0x00;							//不滤波
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;		//上升沿捕获
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;				//输入器不分频
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;	//映射到IC4
	TIM_ICInit(TIM2, &TIM_ICInitStruct);
	
	
	//中断分组初始化
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStruct);	
	
	TIM_ITConfig(TIM2, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);			//更新中断和CC1IE捕获中断
	
	TIM_Cmd(TIM2, ENABLE);
	printf("Init TIM2 CAP success\n");
}

//捕获状态
//[7]:0,没有成功捕获；1,成功捕获到一次
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出第次数
u8 CAPTURE_STA_TIM2CH[4] = {0};
u16 CAPTURE_VAL_TIM2CH[4];
u16 CAPTURE_UP_TIM2CH[4], CAPTURE_DOWN_TIM2CH[4];
void TIM2_IRQHandler(void)
{	
//	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
//		if((CAPTURE_STA_TIM2CH[1]&0x80) == 0) {			//还未捕获成功
//			if(CAPTURE_STA_TIM2CH[1]&0x40) {			//已经捕获到高电平了
//				if((CAPTURE_STA_TIM2CH[1]&0x3F)==0x3F) {			//高电平太长了
//					CAPTURE_STA_TIM2CH[1] |= 0x80;				//标记成功捕获了一次
//					CAPTURE_VAL_TIM2CH[1] = 0xFFFF;
//				} else 
//					CAPTURE_VAL_TIM2CH[1]++;
//			}
//		}
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//	}
		
	if((CAPTURE_STA_TIM2CH[0]&0x80) == 0) {					//还未捕获成功
		if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) {	//捕获1发生捕获事件
			if(CAPTURE_STA_TIM2CH[0]&0x40) {					//捕获到一个下降沿
				CAPTURE_STA_TIM2CH[0] |= 0x80;					//标记成功捕获到一次高电平脉宽
				CAPTURE_DOWN_TIM2CH[0] = TIM_GetCapture1(TIM2);
				if(CAPTURE_DOWN_TIM2CH[0] >= CAPTURE_UP_TIM2CH[0]) 
					CAPTURE_VAL_TIM2CH[0] = CAPTURE_DOWN_TIM2CH[0] - CAPTURE_UP_TIM2CH[0];
				else 
					CAPTURE_VAL_TIM2CH[0] = 0xffff + CAPTURE_DOWN_TIM2CH[0] - CAPTURE_UP_TIM2CH[0];
				TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Rising);	//CC1P=0 设置为上升沿捕获
			} else {										//还未开始，第一次捕获上升沿
				CAPTURE_STA_TIM2CH[0] = 0;					//清空
				CAPTURE_VAL_TIM2CH[0] = 0;
				CAPTURE_UP_TIM2CH[0] = TIM_GetCapture1(TIM2);
				CAPTURE_STA_TIM2CH[0] |= 0x40;				//标记捕获到了上升沿
				TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Falling);	//CC1P=1 设置为下降沿捕获
			}	
			TIM_ClearFlag(TIM2, TIM_FLAG_CC1);								//清除状态标志
		}
	} 

	if((CAPTURE_STA_TIM2CH[1]&0x80) == 0) {					//还未捕获成功
		if(TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET) {			//捕获2发生捕获事件
			if(CAPTURE_STA_TIM2CH[1]&0x40) {						//捕获到一个下降沿
				CAPTURE_STA_TIM2CH[1] |= 0x80;						//标记成功捕获到一次高电平脉宽
				CAPTURE_DOWN_TIM2CH[1] = TIM_GetCapture2(TIM2);		//获取捕获2计数
				if(CAPTURE_DOWN_TIM2CH[1] >= CAPTURE_UP_TIM2CH[1])
					CAPTURE_VAL_TIM2CH[1] = CAPTURE_DOWN_TIM2CH[1] - CAPTURE_UP_TIM2CH[1];
				else
					CAPTURE_VAL_TIM2CH[1] = 0xffff + CAPTURE_DOWN_TIM2CH[1] - CAPTURE_UP_TIM2CH[1];
				TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Rising);	//CC1P=0 设置为上升沿捕获
			} else {												//还未开始，第一次捕获上升沿
				CAPTURE_STA_TIM2CH[1] = 0;							//清空
				CAPTURE_VAL_TIM2CH[1] = 0;
				CAPTURE_UP_TIM2CH[1] = TIM_GetCapture2(TIM2);
				CAPTURE_STA_TIM2CH[1] |= 0x40;				//标记捕获到了上升沿
				TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Falling);	//CC1P=1 设置为下降沿捕获
			}
			TIM_ClearFlag(TIM2, TIM_FLAG_CC2);								//清除状态标志	
		}
	}
	
	if((CAPTURE_STA_TIM2CH[2]&0x80) == 0) {					//还未捕获成功
		if(TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET) {			//捕获2发生捕获事件
			if(CAPTURE_STA_TIM2CH[2]&0x40) {						//捕获到一个下降沿
				CAPTURE_STA_TIM2CH[2] |= 0x80;						//标记成功捕获到一次高电平脉宽
				CAPTURE_DOWN_TIM2CH[2] = TIM_GetCapture3(TIM2);		//获取捕获2计数
				if(CAPTURE_DOWN_TIM2CH[2] >= CAPTURE_UP_TIM2CH[2])
					CAPTURE_VAL_TIM2CH[2] = CAPTURE_DOWN_TIM2CH[2] - CAPTURE_UP_TIM2CH[2];
				else
					CAPTURE_VAL_TIM2CH[2] = 0xffff + CAPTURE_DOWN_TIM2CH[2] - CAPTURE_UP_TIM2CH[2];
				TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Rising);	//CC1P=0 设置为上升沿捕获
			} else {												//还未开始，第一次捕获上升沿
				CAPTURE_STA_TIM2CH[2] = 0;							//清空
				CAPTURE_VAL_TIM2CH[2] = 0;
				CAPTURE_UP_TIM2CH[2] = TIM_GetCapture3(TIM2);
				CAPTURE_STA_TIM2CH[2] |= 0x40;				//标记捕获到了上升沿
				TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Falling);	//CC1P=1 设置为下降沿捕获
			}
			TIM_ClearFlag(TIM2, TIM_FLAG_CC3);								//清除状态标志		
		}
	} 
	if((CAPTURE_STA_TIM2CH[3]&0x80) == 0) {					//还未捕获成功
		if(TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET) {			//捕获2发生捕获事件
			if(CAPTURE_STA_TIM2CH[3]&0x40) {						//捕获到一个下降沿
				CAPTURE_STA_TIM2CH[3] |= 0x80;						//标记成功捕获到一次高电平脉宽
				CAPTURE_DOWN_TIM2CH[3] = TIM_GetCapture4(TIM2);		//获取捕获2计数
				if(CAPTURE_DOWN_TIM2CH[3] >= CAPTURE_UP_TIM2CH[3])
					CAPTURE_VAL_TIM2CH[3] = CAPTURE_DOWN_TIM2CH[3] - CAPTURE_UP_TIM2CH[3];
				else
					CAPTURE_VAL_TIM2CH[3] = 0xffff + CAPTURE_DOWN_TIM2CH[3] - CAPTURE_UP_TIM2CH[3];
				TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Rising);	//CC1P=0 设置为上升沿捕获
			} else {												//还未开始，第一次捕获上升沿
				CAPTURE_STA_TIM2CH[3] = 0;							//清空
				CAPTURE_VAL_TIM2CH[3] = 0;
				CAPTURE_UP_TIM2CH[3] = TIM_GetCapture4(TIM2);
				CAPTURE_STA_TIM2CH[3] |= 0x40;				//标记捕获到了上升沿
				TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Falling);	//CC1P=1 设置为下降沿捕获
			}
			TIM_ClearFlag(TIM2, TIM_FLAG_CC4);								//清除状态标志		
		}
	}

	//处理帧数据
	if(CAPTURE_STA_TIM2CH[0]&0x80) {								//成功捕获到了一次上升沿											//溢出时间总和
		myControl.remoteControl[0] = CAPTURE_VAL_TIM2CH[0];							//得到总的高电平时间
//		printf("TIM2 CH1:%d\t", temp[0]);
		CAPTURE_STA_TIM2CH[0] = 0;
	} 
	if(CAPTURE_STA_TIM2CH[1]&0x80) {								//成功捕获到了一次上升沿											//溢出时间总和
		myControl.remoteControl[1] = CAPTURE_VAL_TIM2CH[1];							//得到总的高电平时间
//		printf("TIM2 CH2:%d\t", temp[1]);
		CAPTURE_STA_TIM2CH[1] = 0;
	} 
	if(CAPTURE_STA_TIM2CH[2]&0x80) {								//成功捕获到了一次上升沿										//溢出时间总和
		myControl.remoteControl[2] = CAPTURE_VAL_TIM2CH[2];							//得到总的高电平时间
//		printf("TIM2 CH3:%d\t", temp[2]);
		CAPTURE_STA_TIM2CH[2] = 0;
	} 
	if(CAPTURE_STA_TIM2CH[3]&0x80) {								//成功捕获到了一次上升沿											//溢出时间总和
		myControl.remoteControl[3] = CAPTURE_VAL_TIM2CH[3];							//得到总的高电平时间
//		printf("TIM2 CH4:%d\t", temp[3]);
		CAPTURE_STA_TIM2CH[3] = 0;
	}
}


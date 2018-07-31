
#include "mpu_exti.h"
#include "usart.h"
#include "mpu_userapi.h"
#include "pca9685.h"
#include "control.h"
#include "led.h"

void DMP_EXTIConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;

	/* GPIOB Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	/* Configure PB5 in input input push down mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);
 	NVIC_SetPriority(EXTI9_5_IRQn, NVIC_EncodePriority(NVIC_PriorityGroup_0, 0, 7)); 
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void)
{

	int ret = 0;
	if(EXTI_GetITStatus(EXTI_Line5)) {
		PB0_On();
		ret = mpu_mpl_get_data(&myControl.pitch, &myControl.roll, &myControl.yaw, &myControl.gyro_X, &myControl.gyro_Y);
		PB0_Off();
		if(!ret) {		//这里是成功吧对吧？
			if(!myControl.unlocked) {			//加锁状态
				Motor_Set(MOTOR_MIDVALUE, MOTOR_MIDVALUE, MOTOR_MIDVALUE, MOTOR_MIDVALUE);						
			} else if(myControl.unlocked) {		//解锁状态
#if CONTROL_ON
				attitude_control();
#else			
				PCA9685_SetPWM(0, 0, myControl.remoteControl[2]/5);
				PCA9685_SetPWM(1, 0, myControl.remoteControl[2]/5);
				PCA9685_SetPWM(2, 0, myControl.remoteControl[2]/5);
				PCA9685_SetPWM(3, 0, myControl.remoteControl[2]/5);
#endif
			}
		} else if(ret == -1) {
#if DEBUG_PRINT
			printf("-1\n");
#endif
		} else if(ret == -2) {
#if DEBUG_PRINT
			printf("-2\n");
#endif
		}
		
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}

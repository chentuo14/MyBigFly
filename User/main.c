/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include "led.h"
#include "sysclock.h"
#include "usart.h"
#include "i2c.h"
#include "mpu_userapi.h"
#include "ms5611.h"
#include "pca9685.h"
#include "tim2.h"
#include "tim4.h"
#include "control.h"
#include "mpu_exti.h"
#include "ANO_DataTransfer.h"
#include "wifi.h"

void Delay(uint16_t c);
void old_code(void);

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	SetSysClockToHSE();
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);
	LED_Config();	
	USART_Config();
	DMA1_CHANNEL2_Config();
	IIC_Init();
//	if(MS5611_Init())					//初始化5611
//		printf("MS5611 Init failed\n");
	
	PCA9685_SetPWMFreq(50);				//初始化PCA9685
	delay_ms(100);
	if(!mpu9250_initialize()) {			//初始化9250
#if DEBUG_PRINT
		printf("MPU 9250 Initialize failed.\n");
#endif
		return 0;
	}
	
	ClearStructMyControl();
	SetDefaultPID();
	TIM2_CAP_Init(0xffff, 72-1);		//TIM2 1Mhz计数
	TIM4_CAP_Init(0xffff, 72-1);		//TIM4 1Mhz计数
	/* Infinite loop */
	WIFI_UDP_INIT();
	DMP_EXTIConfig();

	while(1) {
#if SEND_TO_ANO
		My_ANO_DT_Send_STATUS_SENSER_RCDATA_MOTO(&myControl, &mySenserData);
#endif
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
		
		delay_ms(25);
	}
}

/*******************************************************************************
* Function Name : Delay
* Description :  simple delay
* Input : c
* Output : None
* Return : None
*******************************************************************************/

void Delay(uint16_t c)
{
	uint16_t a,b;
	for(; c>0; c--)
		for(a=100; a>0; a--)
			for(b=100; b>0; b--);
}


/**
  * @}
  */
void old_code(void)
{
	while (1)
	{			
		if(myControl.remoteSwitch[0] < 1400 && myControl.remoteSwitch[0] > 900) {						//K2打低，关闭电机
			while(1) {
				Motor_Set(MOTOR_MIDVALUE, MOTOR_MIDVALUE, MOTOR_MIDVALUE, MOTOR_MIDVALUE);			
#if SEND_TO_ANO
				My_ANO_DT_Send_STATUS_SENSER_RCDATA_MOTO(&myControl, &mySenserData);
#endif		
				if(myControl.remoteSwitch[0] >= 1600 && myControl.remoteSwitch[0] <= 2200)
					break;
				delay_ms(50);

			}
		}else if(myControl.remoteSwitch[0] > 1600 && myControl.remoteSwitch[0] < 2200) {				//K2打高，启动
			while(1) {
#if CONTROL_ON
				attitude_control();
#else			
				PCA9685_SetPWM(0, 0, myControl.remoteControl[2]/5);
				PCA9685_SetPWM(1, 0, myControl.remoteControl[2]/5);
				PCA9685_SetPWM(2, 0, myControl.remoteControl[2]/5);
				PCA9685_SetPWM(3, 0, myControl.remoteControl[2]/5);
#endif
				
//串口打印信息
#if DEBUG_PRINT	
				printf("attitude:pitch %f, roll %f, yaw %f, gyro_x %d gyro_y %d\n", myControl.pitch, myControl.roll, myControl.yaw,
					myControl.gyro_X, myControl.gyro_Y);
//				printf("attitude:TIM2 CH1:%d\tTIM2 CH2:%d\tTIM2 CH3:%d\tTIM2 CH4:%d\n", 
//					myControl.remoteControl[0], myControl.remoteControl[1],
//					myControl.remoteControl[2], myControl.remoteControl[3]);	
//				printf("attitude:TIM4 CH3:%d\t TIM4 CH4:%d\n", myControl.remoteSwitch[0], myControl.remoteSwitch[1]);
#endif

//串口发送给上位机
#if SEND_TO_ANO
					My_ANO_DT_Send_STATUS_SENSER_RCDATA_MOTO(&myControl, &mySenserData);
#endif 
				if(myControl.remoteSwitch[0] <= 1600 && myControl.remoteSwitch[0] >=900)
					break;
				delay_ms(25);						//100ms delay
			}
		}
#if DEBUG_PRINT
		else {
			printf("noControl:pitch %f, roll %f, yaw %f, gyro_x %d gyro_y %d\n", myControl.pitch, myControl.roll, myControl.yaw,
					myControl.gyro_X, myControl.gyro_Y);
			printf("noControl:TIM2 CH1:%d\tTIM2 CH2:%d\tTIM2 CH3:%d\tTIM2 CH4:%d\n", 
					myControl.remoteControl[0], myControl.remoteControl[1],
					myControl.remoteControl[2], myControl.remoteControl[3]);	
			printf("noControl:TIM4 CH3:%d\t TIM4 CH4:%d\n", myControl.remoteSwitch[0], myControl.remoteSwitch[1]);
			delay_ms(10);
		}			
#endif
	}	
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

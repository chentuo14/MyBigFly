#include "pca9685.h"
#include "i2c.h"
#include "sysclock.h"
#include "math.h"
#include "usart.h"
#include "control.h"


/****************************************
*@函数原型:		int PCA9685_Write_Byte(u8 reg, u8 data)
*@功    能:	    PCA9685写入一个byte
****************************************/
int PCA9685_Write_Byte(u8 reg, u8 data)
{
	IIC_Start();
	IIC_Send_Byte(PCA9685_ADDR);
	if(IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);
	if(IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(data);
	if(IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	IIC_Stop();
	return 0;
}

/****************************************
*@函数原型:		u8 PCA9685_Read_Byte(u8 reg)
*@功    能:	    PCA9685读出一个byte
****************************************/
u8 PCA9685_Read_Byte(u8 reg)
{
	u8 data;
	IIC_Start();
	IIC_Send_Byte(PCA9685_ADDR);
	if(IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);
	if(IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	} 
	IIC_Start();
	IIC_Send_Byte(PCA9685_ADDR|1);
	if(IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	data = IIC_Read_Byte(0);
	IIC_Stop();
	return data;
}

/****************************************
*@函数原型:		void PCA9685_Reset(void)
*@功    能:	    复位（放弃）
****************************************/
void PCA9685_Reset(void)
{
	u8 data;
	data = PCA9685_Read_Byte(PCA9685_MODE1);
	if(data>>7 & 0x01) {
		PCA9685_Write_Byte(PCA9685_MODE1, data&0xEF);
		delay_ms(2);
	}
	PCA9685_Write_Byte(PCA9685_MODE1, data|MODE1_RESTART);
}

/****************************************
*@函数原型:		void PCA9685_SetPWMFreq(u8 freq)
*@功    能:	    设置频率，需要在sleep模式下fre为Hz
****************************************/
void PCA9685_SetPWMFreq(u8 freq)
{
	u8 prescale,oldmode,newmode;
	double prescaleval;
	prescaleval = 25000000.0/(4096*freq*0.915);
	prescale = (u8)floor(prescaleval+0.5)-1;

	oldmode = PCA9685_Read_Byte(PCA9685_MODE1);
	newmode = (oldmode&0x7F) | 0x10; // sleep
	PCA9685_Write_Byte(PCA9685_MODE1, newmode); // go to sleep
	PCA9685_Write_Byte(PCA9685_PRE_SCALE, prescale); // set the prescaler
	oldmode = oldmode & 0xEF;					//exit to sleep
	PCA9685_Write_Byte(PCA9685_MODE1, oldmode);
	delay_ms(5);
	PCA9685_Write_Byte(PCA9685_MODE1, oldmode | 0xa1); 
	printf("Set PCA9685 Freq:%d\n", freq);
}

/****************************************
*@函数原型:		void PCA9685_SetPWM(u8 num, u16 on, u16 off)
*@功    能:	    on为0到上升沿时间，off为0到下降沿时间
****************************************/
void PCA9685_SetPWM(u8 num, u16 on, u16 off)
{
	PCA9685_Write_Byte(LEDn_ON_L+4*num, on);
	PCA9685_Write_Byte(LEDn_ON_H+4*num, on>>8);
	PCA9685_Write_Byte(LEDn_OFF_L+4*num, off);
	PCA9685_Write_Byte(LEDn_OFF_H+4*num, off>>8);
}

/****************************************
*@函数原型:		void Motor_Set(u16 motor0, u16 motor1, u16 motor2, u16 motor3)
*@功    能:	    设置四路电机速度
****************************************/
void Motor_Set(u16 motor0, u16 motor1, u16 motor2, u16 motor3)
{
	if(motor0 >= MOTOR_MAXVALUE) motor0 = MOTOR_MAXVALUE;
	if(motor1 >= MOTOR_MAXVALUE) motor1 = MOTOR_MAXVALUE;
	if(motor2 >= MOTOR_MAXVALUE) motor2 = MOTOR_MAXVALUE;
	if(motor3 >= MOTOR_MAXVALUE) motor3 = MOTOR_MAXVALUE;
	
	if(motor0 < MOTOR_MIDVALUE) motor0 = MOTOR_MIDVALUE;
	if(motor1 < MOTOR_MIDVALUE) motor1 = MOTOR_MIDVALUE;
	if(motor2 < MOTOR_MIDVALUE) motor2 = MOTOR_MIDVALUE;
	if(motor3 < MOTOR_MIDVALUE) motor3 = MOTOR_MIDVALUE;

	
	PCA9685_SetPWM(0, 0, motor0/MOTO_TO_PWM);
	PCA9685_SetPWM(1, 0, motor1/MOTO_TO_PWM);
	PCA9685_SetPWM(2, 0, motor2/MOTO_TO_PWM);
	PCA9685_SetPWM(3, 0, motor3/MOTO_TO_PWM);
}

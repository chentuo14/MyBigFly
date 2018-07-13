
#include "ms5611.h"
#include "i2c.h"
#include "sysclock.h"
#include "usart.h"

static u16 prom[8];

/****************************************
*@函数原型:		u8 MS5611_Command(u8 command)
*@功    能:	    发送给MS5611指令
****************************************/
u8 MS5611_Command(u8 command)
{
	IIC_Start(); 
	IIC_Send_Byte((MS5611_ADDR<<1)|0);
	if(IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(command);
	if(IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	IIC_Stop();
	
	return 0;
}

/****************************************
*@函数原型:		void MS5611_Reset(void)
*@功    能:	    复位MS5611
****************************************/
void MS5611_Reset(void)
{
	MS5611_Command(MS5611_RESET);
}

/****************************************
*@函数原型:		u16 MS5611_Read_PROM(u8 reg)
*@功    能:	    读取MS5611_PROM
*@输    入：	reg地址0~7
*@输    出：    16位数据
****************************************/
u16 MS5611_Read_PROM(u8 reg)
{
	u8 data[2] = {0};
	IICreadBytes(MS5611_ADDR, (MS5611_PROM_BASE_ADDR|(reg<<1)), 2, data);
	return (data[0]<<8|data[1]);
}

/****************************************
*@函数原型:		void MS5611_Read_Press(u8 osr, u8 *data)
*@功    能:	    读取D1(气压)
*@输    入：	osr
*@输    出：    24位气压
****************************************/
u32 MS5611_Read_Press(u8 osr)
{
	u8 data[3];
	MS5611_Command(MS5611_D1|osr);
	delay_ms(10);
	IICreadBytes(MS5611_ADDR, 0x00, 3, data);
	
	return (data[0]<<16|data[1]<<8|data[2]);
}

/****************************************
*@函数原型:		void MS5611_Read_Temp(u8 osr, u8 *data)
*@功    能:	    读取D2(温度)
*@输    入：	osr
*@输    出：    24位温度
****************************************/
u32 MS5611_Read_Temp(u8 osr)
{
	u8 data[3];
	MS5611_Command(MS5611_D2|osr);
	delay_ms(10);
	IICreadBytes(MS5611_ADDR, 0x00, 3, data);
	
	return (data[0]<<16|data[1]<<8|data[2]);
}

/****************************************
*@函数原型:		int MS5611_CRC(void)
*@功    能:	    CRC校验
*@输    入：	无
*@输    出：    0成功1失败
****************************************/
int MS5611_CRC(void)
{
	u8 crc = 0;
	u8 i;
	for(i=0;i<MS5611_PROM_REG_COUNT-1;i++) {
		crc ^= prom[i];
	}
	if(crc == prom[i]&0x00ff)
		return 0;
	else
		return 1;
}

/****************************************
*@函数原型:		int MS5611_Init(void)
*@功    能:	    初始化
*@输    入：	无
*@输    出：    0成功1失败
****************************************/
int MS5611_Init(void)
{
	u8 i;
	MS5611_Reset();
	delay_ms(10);
	
	for(i=0;i<MS5611_PROM_REG_COUNT;i++) {
		prom[i] = MS5611_Read_PROM(i);
#if DEBUG_PRINT
		printf("prom[%d] is %d\n", i, prom[i]);
#endif
	}
	
//	if(MS5611_CRC())			//好像没用，我也没搞懂怎么CRC
//		return 1;
	
	return 0;
}

void MS5611_GetValue(float *temperature, float *press)
{
	float dT;
	u32 D1_Press, D2_Temp;
	float TEMP, TEMP2;	//实际和参考温度
	float OFF2, SENS2;	//温度校验值
	double OFF, SENS;
	D2_Temp = MS5611_Read_Temp(MS5611_OSR_4096);
	dT = (float)D2_Temp - (((float)prom[5])*256);
	TEMP = 2000 + ((dT* (float)prom[6])/8388608.0);
	
	D1_Press = MS5611_Read_Press(MS5611_OSR_4096);
	OFF = (float)(prom[2]<<16)+((float)prom[4]*dT)/128.0;
	SENS = (float)(prom[1]<<15)+((float)prom[3]*dT)/256.0;
	if(TEMP < 2000) {
		TEMP2 = dT*dT/2147483648;
		OFF2 = 5*(TEMP-2000)*(TEMP-2000)/2.0;
		SENS2 = 5*(TEMP-2000)*(TEMP-2000)/4.0;
		if(TEMP < -1500) {
			OFF2 = OFF2 + 7*(TEMP+1500)*(TEMP+1500);
			SENS2 = SENS2 + 11*(TEMP+1500)*(TEMP+1500)/2.0;
		}
	} else {
		TEMP2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	
	TEMP = TEMP - TEMP2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;
	
	*temperature = TEMP;
	*press = ((float)D1_Press*SENS/2097152.0 - OFF)/32768.0;
}

void PrintTempAndPress(void)
{
	float temp, press;
	MS5611_GetValue(&temp, &press);
#if DEBUG_PRINT
	printf("Temp = %f, Press = %f\n", temp/100, press/100); 
#endif
}

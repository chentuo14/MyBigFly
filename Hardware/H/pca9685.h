#ifndef __PCA9685_H_
#define __PCA9685_H_

#include "stm32f10x.h"

#define PCA9685_ADDR			0x80		//7位地址

//REG ADDRESS
#define PCA9685_MODE1 			0x00
#define PCA9685_MODE2			0x01
#define PCA9685_SUBADR1			0x02
#define PCA9685_SUBADR2			0x03
#define PCA9685_SUBADR3			0x04
#define PCA9685_ALLCALLADR		0x05
#define PCA9685_LED0_ON_L       0x06
#define PCA9685_LED0_ON_H       0x07
#define PCA9685_LED0_OFF_L      0x08
#define PCA9685_LED0_OFF_H      0x09
#define PCA9685_LED1_ON_L       0x0a
#define PCA9685_LED1_ON_H       0x0b
#define PCA9685_LED1_OFF_L      0x0c
#define PCA9685_LED1_OFF_H      0x0d
#define PCA9685_LED2_ON_L       0x0e
#define PCA9685_LED2_ON_H       0x0f
#define PCA9685_LED2_OFF_L      0x10
#define PCA9685_LED2_OFF_H      0x11
#define PCA9685_LED3_ON_L       0x12
#define PCA9685_LED3_ON_H       0x13
#define PCA9685_LED3_OFF_L      0x14
#define PCA9685_LED3_OFF_H      0x15
#define PCA9685_LED4_ON_L       0x16
#define PCA9685_LED4_ON_H       0x17
#define PCA9685_LED4_OFF_L      0x18
#define PCA9685_LED4_OFF_H      0x19
#define PCA9685_LED5_ON_L       0x1a
#define PCA9685_LED5_ON_H       0x1b
#define PCA9685_LED5_OFF_L      0x1c
#define PCA9685_LED5_OFF_H      0x1d
#define PCA9685_LED6_ON_L       0x1e
#define PCA9685_LED6_ON_H       0x1d
#define PCA9685_LED6_OFF_L      0x20
#define PCA9685_LED6_OFF_H      0x21
#define PCA9685_LED7_ON_L       0x22
#define PCA9685_LED7_ON_H       0x23
#define PCA9685_LED7_OFF_L      0x24
#define PCA9685_LED7_OFF_H      0x25
#define PCA9685_LED8_ON_L       0x26
#define PCA9685_LED8_ON_H       0x27
#define PCA9685_LED8_OFF_L      0x28
#define PCA9685_LED8_OFF_H      0x29
#define PCA9685_LED9_ON_L       0x2a
#define PCA9685_LED9_ON_H       0x2b
#define PCA9685_LED9_OFF_L      0x2c
#define PCA9685_LED9_OFF_H      0x2d
#define PCA9685_LED10_ON_L      0x2e
#define PCA9685_LED10_ON_H      0x2f
#define PCA9685_LED10_OFF_L     0x30
#define PCA9685_LED10_OFF_H     0x31
#define PCA9685_LED11_ON_L      0x32
#define PCA9685_LED11_ON_H      0x33
#define PCA9685_LED11_OFF_L     0x34
#define PCA9685_LED11_OFF_H     0x35
#define PCA9685_LED12_ON_L      0x36
#define PCA9685_LED12_ON_H      0x37
#define PCA9685_LED12_OFF_L     0x38
#define PCA9685_LED12_OFF_H     0x39
#define PCA9685_LED13_ON_L      0x3a
#define PCA9685_LED13_ON_H      0x3b
#define PCA9685_LED13_OFF_L     0x3c
#define PCA9685_LED13_OFF_H     0x3d
#define PCA9685_LED14_ON_L      0x3e
#define PCA9685_LED14_ON_H      0x3f
#define PCA9685_LED14_OFF_L     0x40
#define PCA9685_LED14_OFF_H     0x41
#define PCA9685_LED15_ON_L      0x42
#define PCA9685_LED15_ON_H      0x43
#define PCA9685_LED15_OFF_L     0x44
#define PCA9685_LED15_OFF_H     0x45
#define PCA9685_ALL_LED_ON_L    0xfa
#define PCA9685_ALL_LED_ON_H    0xfb
#define PCA9685_ALL_LED_OFF_L   0xfc
#define PCA9685_ALL_LED_OFF_H   0xfd
#define PCA9685_PRE_SCALE      	0xfe
#define PCA9685_TESTMODE		0xff

//MODE1
#define MODE1_RESTART			0x80		//重启1，无效0
#define MODE1_EXTCLK			0x40		//外部晶振
#define MODE1_AI				0x20		//寄存器自动增加位
#define MODE1_SLEEP				0x10		//睡眠模式
#define MODE1_SUB1				0x08		//0不响应SUB1，1响应SUB1
#define MODE1_SUB2				0x04		//0不响应SUB2，1响应SUB2
#define MODE1_SUB3				0x02		//0不响应SUB3，1响应SUB3
#define MODE1_ALLCALL			0x01		//0不响应所有I2C总线，1响应多有

//MODE2
#define MODE2_INVRT				0x10		//0输出不反转，1输出反转
#define MODE2_OCH				0x08		//0输出改变在STOP命令，1输出改变在ACK命令
#define MODE2_OUTDRV			0x04		//0开漏，1推完
#define MODE2_OUTNE1			0x02
#define MODE2_OUTNE0			0x01		/* 00 OE = 1(output drivers not enabled), LEDn = 0;
											 * 01 OE = 1(output drivers not enabled):
											 * 		LEDn = 1 when OUTDRV = 1
											 *		LEDn = high-impedance when OUTDRV = 0(same as OUTNE[1:0] = 10)
											 * When OE = 1(output drivers not enabled), LEDn = high-impedance.
											 */

#define LEDn_ON_L		0x06
#define LEDn_ON_H		0x07
#define LEDn_OFF_L		0x08
#define LEDn_OFF_H		0x09

int PCA9685_Write_Byte(u8 reg, u8 data);
u8 PCA9685_Read_Byte(u8 reg);
void PCA9685_Reset(void);
void PCA9685_SetPWMFreq(u8 freq);
void PCA9685_SetPWM(u8 num, u16 on, u16 off);
void Motor_Set(u16 motor0, u16 motor1, u16 motor2, u16 motor3);

#endif

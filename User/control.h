#ifndef __CONTROL_H_
#define __CONTROL_H_

#include "stm32f10x.h"

#define GYRO_XISHU	(2000.0/65535.0)		//用来将GYRO原始值转换为实际角速度的比例系数
#define PITCHROLLOFF_ON 1
#define PITCH_OFF 9.30
#define ROLL_OFF  9.60

struct MY_CONTROL {
	float pitch;
	float roll;
	float yaw;
	float pitch_off;
	float roll_off;
	vs16 gyro_X;
	vs16 gyro_Y;
	vs16 gyro_Z;
	u32 remoteControl[4];			//1:左2000右999，2：上2000下999，3：油门，4：YAW左2000右999
	u32 remoteSwitch[2];
};

//油门最大值
#define MOTO_TO_PWM		5			//PWM转成PCA9685的系数
#define MOTOR_MAXVALUE	2000		//电机最大PWM
#define MOTOR_MIDVALUE	1000		//电机最小PWM

//PID积分最大值
#define PITCH_I_MAX		300
#define ROLL_I_MAX		300

void ClearStructMyControl(void);
int Direction_Control(void);
void Outter_PID(void);
void Inner_PID(void);
void Rotation_Correction(void);
void Deal_Pwm(void);
void Set_Pwm(void);

extern struct MY_CONTROL myControl;

#endif

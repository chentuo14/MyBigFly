
#include "control.h"
#include "pca9685.h"
#include "usart.h"

struct MY_CONTROL myControl;
/*********************************************************************************
全局变量定义区
************************************************************************************/
volatile float ZHONGZHI_PIT = 0.0;		//pitch角给定值，用于控制方向和保持平衡
volatile float ZHONGZHI_ROL = 0.0;		//roll角给定值，用于控制方向和保持平衡
volatile float ZHONGZHI_YAW = 0.0;		//偏航角给定值，用于修正航向角和克服自转问题

volatile float angular_speed_X = 0;		//内环x轴角速度给定值
volatile float angular_speed_Y = 0;		//内环y轴角速度给定值

//外环PID
static float pit_p=0;								//绕X轴比例系数
static float rol_p=0;								//绕Y轴比例系数
static float e_pit,e_rol;						    //X轴偏差和Y轴偏差

//内环PID
float e_I_Y,e_I_X;									//内环积分累计偏差值
static float kp1=0,ki1=0,kd1=0;						//绕X轴PID系数(pitch)
static float kp2=-4.5,ki2=0,kd2=-8;						//绕Y轴PID系数(roll),kp=25,ki=0.5,kd=22,此组参数性能较好
static float e_X[2],e_Y[2];							//本次偏差，前一次偏差
static float flag_Y=0.0,flag_X=0.0;						//积分项参不参与运算的标志

vs16 PWM_X,PWM_Y;						//内环PID运算后输出的PWM值
vs16 PWM_YAW;							//偏航修正PWM补偿
vs16 MOTO_PWM[4];						//保存输出控制四个电机的PWM值

/**************************实现函数********************************************
*函数原型:		void ClearStructMyControl(void)
*功    能:		清空结构体内容
*******************************************************************************/
void ClearStructMyControl(void)
{
	u8 i;
	myControl.pitch = 0.0;
	myControl.roll = 0.0;
	myControl.yaw = 0.0;
	
	myControl.gyro_X = 0.0;
	myControl.gyro_Y = 0.0;
	myControl.gyro_Z = 0.0;
	for(i=0;i<4;i++)
		myControl.remoteControl[i]  = 0.0;			//1:左2000右999，2：上2000下999，3：油门，4：YAW左2000右999
	for(i=0;i<2;i++)
		myControl.remoteSwitch[i]  = 0.0;
}

/**************************实现函数********************************************
*函数原型:		void Direction_Control(void)
*功    能:		前后左右方向控制
*******************************************************************************/
int Direction_Control(void)
{
	if(myControl.remoteControl[0] > 1490 && myControl.remoteControl[0] < 1510)
		myControl.remoteControl[0] = 1500;
	if(myControl.remoteControl[1] > 1490 && myControl.remoteControl[1] < 1510)
		myControl.remoteControl[1] = 1500;
	if(myControl.remoteControl[3] > 1495 && myControl.remoteControl[3] < 1505)
		myControl.remoteControl[3] = 1500;	
	//根据遥控器传过来的前后方向值，改变ZHONGZHI_PIT的给定值
	//按40度计算，500/40 = 12.5(pwm/度) 2度是25
	ZHONGZHI_PIT = (myControl.remoteControl[1]-1500)/12.5;
	
	//根据遥控器传过来的左右方向值，改变ZHONGZHI_ROL的给定值
	ZHONGZHI_ROL = (myControl.remoteControl[0]-1500)/12.5;
	
	if((myControl.pitch >= 55) || (myControl.pitch <= -55)) {							//超过度数，关闭油门
		Motor_Set(MOTOR_MIDVALUE, MOTOR_MIDVALUE, MOTOR_MIDVALUE, MOTOR_MIDVALUE);
		return -1;
	}
	
	if((myControl.roll >= 55) || (myControl.roll <= -55)) {								//超过度数，关闭油门
		Motor_Set(MOTOR_MIDVALUE, MOTOR_MIDVALUE, MOTOR_MIDVALUE, MOTOR_MIDVALUE);
		return -1;	
	}
	
	return 0;
}

/**************************实现函数********************************************
*函数原型:		void Outter_PID(void)
*功    能:		外环角度控制
*******************************************************************************/
void Outter_PID(void)
{	
	//计算X轴和Y轴角度偏差值
	e_pit = ZHONGZHI_PIT-myControl.pitch;
	e_rol = ZHONGZHI_ROL-myControl.roll;

	//外环PID运算
	angular_speed_X = pit_p*e_pit;
	angular_speed_Y = rol_p*e_rol;
}

/**************************实现函数********************************************
*函数原型:		void Inner_PID(void)
*功    能:		内环角速度控制
*******************************************************************************/
void Inner_PID(void)
{
	//计算X轴和Y轴角速度偏差
	e_X[0] = angular_speed_X - myControl.gyro_X;
	e_Y[0] = angular_speed_Y - myControl.gyro_Y;
		
//	e_X[0] = 0 - myControl.gyro_X;
//	e_Y[0] = 0 - myControl.gyro_Y;
	
	//===========================绕X轴内环PID运算============================================
	//积分分离，以便在偏差较大的时候可以快速的缩减偏差，在偏差较小的时候，才加入积分，消除误差
	if(e_X[0]>=150.0||e_X[0]<=-150.0){
		flag_X = 0.0;
	}else{
		flag_X = 1.0;
		e_I_X += e_X[0];
	}
	
	//积分限幅
	if(e_I_X>PITCH_I_MAX)
		e_I_X=PITCH_I_MAX;             					
	if(e_I_X<-PITCH_I_MAX)	
		e_I_X=-PITCH_I_MAX;              					
	
	//位置式PID运算
	PWM_X = (s16)(kp1*e_X[0]+flag_X*ki1*e_I_X+kd1*(e_X[0]-e_X[1]));
//printf("PWM_X%d, \n", PWM_X);
	//===========================绕Y轴内环PID运算========================================
	//积分分离，以便在偏差较大的时候可以快速的缩减偏差，在偏差较小的时候，才加入积分，消除误差
	if(e_Y[0]>=150.0||e_Y[0]<=-150.0){
		flag_Y = 0.0;
	}else{
		flag_Y = 1.0;
		e_I_Y += e_Y[0];
	}
	
	//积分限幅
	if(e_I_Y>ROLL_I_MAX)
		e_I_Y=ROLL_I_MAX;
	if(e_I_Y<-ROLL_I_MAX)
		e_I_Y=ROLL_I_MAX;
	
	//位置式PID运算
	PWM_Y = (s16)(kp2*e_Y[0] + flag_Y*ki2*e_I_Y + kd2*(e_Y[0]-e_Y[1]));
printf("PWM_Y:%d, e_Y[0]:%d\n", PWM_Y, e_Y[0]);
	//=======================================================================================
	//记录本次偏差
	e_X[1] = e_X[0];								//用本次偏差值替换上次偏差值
	e_Y[1] = e_Y[0];								//用本次偏差值替换上次偏差值

}

/**************************实现函数********************************************
*函数原型:		void Rotation_Correction(void)
*功    能:		自转修正补偿PD
*******************************************************************************/
void Rotation_Correction(void)
{
	static float kp1=40,kd1=60;						//自转修正系数，kp为40，kd为60较合适的参数
	static float e_Yaw[2];							//本次偏差，前一次偏差
	
	//yaw的偏差值
	e_Yaw[0] = ZHONGZHI_YAW - myControl.yaw;
//	printf("yaw_zhongzhi:%f\r\n",yaw_zhongzhi);

	//位置式PD运算
	PWM_YAW = kp1*e_Yaw[0]+kd1*(e_Yaw[0]-e_Yaw[1]);
//	printf("pwm_Yaw:%d\r\n",pwm_Yaw);
	
	//记录本次偏差
	e_Yaw[1] = e_Yaw[0];							//用本次偏差值替换上次偏差值
}

/**************************实现函数********************************************
*函数原型:		void Deal_Pwm(void)
*功    能:		pwm输出限速
*******************************************************************************/
void Deal_Pwm(void)
{
	if(myControl.remoteControl[2] <= 1020) {
		angular_speed_X = 0;
		angular_speed_Y = 0;
		ZHONGZHI_YAW = myControl.yaw;
		
		PWM_X = 0;
		PWM_Y = 0;
		PWM_YAW = 0;
		e_I_X = 0;
		e_I_Y = 0;
	}
}

/**************************实现函数********************************************
*函数原型:		void Set_Pwm(void)
*功    能:		设置电机的pwm值
*******************************************************************************/
void Set_Pwm(void)
{
	u8 i;
PWM_X = 0;
PWM_YAW = 0;
	//装配给各电机的PWM值
	MOTO_PWM[0] = myControl.remoteControl[2]+PWM_X+PWM_Y+PWM_YAW;
	MOTO_PWM[1] = myControl.remoteControl[2]-PWM_X+PWM_Y-PWM_YAW;
	MOTO_PWM[2] = myControl.remoteControl[2]-PWM_X-PWM_Y+PWM_YAW;
	MOTO_PWM[3] = myControl.remoteControl[2]+PWM_X-PWM_Y-PWM_YAW;
			
	//PWM限幅和防止出现负值
	for(i=0;i<4;i++){
		if(MOTO_PWM[i] >= MOTOR_MAXVALUE){
			MOTO_PWM[i] = MOTOR_MAXVALUE;
		}else if(MOTO_PWM[i] <= MOTOR_MIDVALUE){
			MOTO_PWM[i] = MOTOR_MIDVALUE;
		}
	}
	printf("PWM_X:%d, PWM_Y:%d, PWM_YAW:%d\r\n", PWM_X, PWM_Y, PWM_YAW);
	printf("moto_pwm1:%d\r\n", MOTO_PWM[0]);
	printf("moto_pwm2:%d\r\n", MOTO_PWM[1]);
	printf("moto_pwm3:%d\r\n", MOTO_PWM[2]);
	printf("moto_pwm4:%d\r\n", MOTO_PWM[3]);
	printf("================================\r\n");
	Motor_Set(MOTO_PWM[0], MOTO_PWM[1], MOTO_PWM[2], MOTO_PWM[3]);
}



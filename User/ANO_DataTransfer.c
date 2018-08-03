#include "ANO_DataTransfer.h"
#include "wifi.h"

///////////////////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t f;			//需要发送数据的标志
u8 data_to_send[50];	//发送数据缓存
u8 send_all[100];
u8 myDMA_Busy = 0;
static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	DMA1_Channel2_Send(data_to_send, 7);
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		myDMA_Busy = 1;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
		myDMA_Busy = 0;
	}
	else
		state = 0;
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X01)
	{
//		if(*(data_buf+4)==0X01)
//			mpu6050.Acc_CALIBRATE = 1;
//		if(*(data_buf+4)==0X02)
//			mpu6050.Gyro_CALIBRATE = 1;
//		if(*(data_buf+4)==0X03)
//		{
//			mpu6050.Acc_CALIBRATE = 1;		
//			mpu6050.Gyro_CALIBRATE = 1;			
//		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			;
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
//			Para_ResetToFactorySetup();
		}
	}

	if(*(data_buf+2)==0X10)								//PID1
    {
        myPID.rol_p  	= 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        myPID.rol_i  	= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        myPID.rol_d  	= 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        myPID.pit_p 	= 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        myPID.pit_i 	= 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        myPID.pit_d 	= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        myPID.yaw_p 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        myPID.yaw_i 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        myPID.yaw_d 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
     	myPID.x_p	= -0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
     	myPID.x_i	= -0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
     	myPID.x_d	= -0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		myPID.y_p 	= 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+11) );
		myPID.y_i   = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+13) );
		myPID.y_d   = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+15) );
        myPID.z_p 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        myPID.z_i 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        myPID.z_d 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
		ANO_DT_Send_Check(*(data_buf+2),sum);
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
}

/**************************实现函数********************************************
*函数原型:		void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
*功    能:		发送姿态到匿名上位机
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed, u8 sw)
{
	u8 _cnt = 0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);	
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;

	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	if(sw == SW_ON)	
		serial3_send_buff(data_to_send, _cnt);
}

/**************************实现函数********************************************
*函数原型:		void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
*功    能:		发送传感器数据到上位机
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z, u8 sw)
{
	u8 _cnt = 0;
	vs16 _temp;
	
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x02;
	data_to_send[_cnt++] = 0;
	
	_temp = a_x;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	data_to_send[3] = _cnt - 4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	if(sw == SW_ON)
		serial3_send_buff(data_to_send, _cnt);
}

/**************************实现函数********************************************
*函数原型:		void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
*功    能:		发送遥控器信号到上位机
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6, u8 sw)
{
	u8 _cnt = 0;
	vs16 _temp;
	
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x03;
	data_to_send[_cnt++] = 0;
	
	_temp = thr;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = yaw;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = rol;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = pit;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	_temp = aux1;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = aux2;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = aux3;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = aux4;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	_temp = aux5;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = aux6;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	data_to_send[3] = _cnt - 4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	if(sw == SW_ON)
		serial3_send_buff(data_to_send, _cnt);
}

/**************************实现函数********************************************
*函数原型:		void My_ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit)
*功    能:		发送遥控器4路信号到上位机
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void My_ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit, u8 sw)
{
//	if(thr>3000 || yaw>3000 || rol>3000 || pit>3000)
//		return;
	ANO_DT_Send_RCData(thr, yaw, rol, pit, 0, 0, 0, 0, 0, 0, sw);
}

/**************************实现函数********************************************
*函数原型:		void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
*功    能:		发送电机PWM到上位机
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8, u8 sw)
{
	u8 _cnt = 0;
	vs16 _temp;
	
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x06;
	data_to_send[_cnt++] = 0;
	
	_temp = m_1;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = m_2;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = m_3;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = m_4;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	_temp = m_5;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = m_6;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = m_7;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = m_8;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	data_to_send[3] = _cnt - 4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	if(sw == SW_ON)
		serial3_send_buff(data_to_send, _cnt);
}

/**************************实现函数********************************************
*函数原型:		void My_ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4)
*功    能:		发送遥控器4路信号到上位机
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void My_ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4, u8 sw)
{
	ANO_DT_Send_MotoPWM(m_1 - 1000, m_2 - 1000, m_3 - 1000, m_4 - 1000, 0, 0, 0, 0, SW_ON);
}

/**************************实现函数********************************************
*函数原型:		void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
*功    能:		发送电机PWM到上位机
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt = 0;
	vs16 _temp;
	
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0x10+group-1;				//从16开始，0，1，2--16，17，18
	data_to_send[_cnt++] = 0;
	
	_temp = p1_p*1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p1_i*1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);	
	_temp = p1_d*1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = p2_p*1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p2_i*1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p2_d*1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	
	_temp = p3_p*1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p3_i*1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);
	_temp = p3_d*1000;
	data_to_send[_cnt++] = BYTE1(_temp);
	data_to_send[_cnt++] = BYTE0(_temp);

	data_to_send[3] = _cnt - 4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	serial3_send_buff(data_to_send, _cnt);
//	DMA1_Channel2_Send(data_to_send, _cnt);
}

void My_ANO_DT_Send_STATUS_SENSER_RCDATA_MOTO(struct MY_CONTROL *send_Control, struct Senser_Data *send_sensor)
{
	u8 i, offset;
/******************************STATUS**********************************/
	ANO_DT_Send_Status(send_Control->roll, send_Control->pitch, send_Control->yaw, 0, 0, send_Control->unlocked, SW_OFF);
	offset = 0;
	for(i=0;i<STATUS_LENGTH;i++) {
		send_all[offset+i] = data_to_send[i];
		data_to_send[i] = 0;
	}
	
/******************************SENSER**********************************/
	ANO_DT_Send_Senser(send_sensor->a_x, send_sensor->a_y, send_sensor->a_z, 
		send_sensor->g_x, send_sensor->g_y, send_sensor->g_z,
		send_sensor->m_x, send_sensor->m_y, send_sensor->m_z, SW_OFF);
	offset = offset + STATUS_LENGTH;
	for(i=0;i<SENSER_LENGTH;i++) {
		send_all[offset+i] = data_to_send[i];
		data_to_send[i] = 0;
	}

/******************************RCDATA**********************************/
	My_ANO_DT_Send_RCData(send_Control->remoteControl[0], send_Control->remoteControl[1],
		send_Control->remoteControl[2], send_Control->remoteControl[3], SW_OFF);
	offset = offset + SENSER_LENGTH;
	for(i=0;i<RCDATA_LENGTH;i++) {
		send_all[offset+i] = data_to_send[i];
		data_to_send[i] = 0;
	}
	
/******************************MOTO**********************************/
	My_ANO_DT_Send_MotoPWM(send_Control->MOTO_PWM[0], send_Control->MOTO_PWM[1],
		send_Control->MOTO_PWM[2], send_Control->MOTO_PWM[3], SW_OFF);
	offset = offset + RCDATA_LENGTH;
	for(i=0;i<MOTO_LENGTH;i++) {
		send_all[offset+i] = data_to_send[i];
		data_to_send[i] = 0;
	}
	
	DMA1_Channel2_Send(send_all, offset+MOTO_LENGTH);
}

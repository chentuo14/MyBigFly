
#include "i2c.h"
#include "sysclock.h"
#include "usart.h"

/**********************************************************************
*函数原型:		void IIC_Init(void)
*功    能:		初始化I2C对应的接口引脚
*******************************************************************************/
void IIC_Init(void)
{			
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			     
 	//配置PB6 PB7开漏输出 刷新频率50Mhz
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //应用配置到GPIOB
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    printf("IIC bus init success...\r\n");
}

/*******************************************************************************
*函数原型:		void IIC_Start(void)
*功    能:		产生IIC起始信号
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(2);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(2);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据
}

/*******************************************************************************
*函数原型:		void IIC_Stop(void)
*功    能:	    产生IIC停止信号
*******************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(2);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	delay_us(2);							   	
}

/******************************************************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功    能:	    等待应答信号到来
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入 
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
		delay_us(1);
	}
	IIC_SCL=0;//时钟输出0	   
	return 0;  
} 

/******************************************************************************
*函数原型:		void IIC_Ack(void)
*功    能:	    产生ACK应答
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}

/******************************************************************************
*函数原型:		void IIC_NAck(void)
*功    能:	    产生NACK应答
*******************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(1);
	IIC_SCL=1;
	delay_us(1);
	IIC_SCL=0;
}			

/******************************************************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功    能:	    IIC发送一个字节
*******************************************************************************/
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 

/******************************************************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功    能:	    读1个字节，ack=1时，发送ACK，ack=0，发送NACK
*******************************************************************************/
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(2); 
    }					 
    if (ack)
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck();//发送ACK  
    return receive;
}

/******************************************************************************
*函数原型:		unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
*功    能:	    读取指定设置指定寄存器的值
*输    入：		I2C_Addr 目标设备地址
				addr	 寄存器地址
*返    回：		读出来的值
*******************************************************************************/
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	

	IIC_Send_Byte(I2C_Addr);	   	//发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(addr);  			//发送地址
	IIC_Wait_Ack();	  
	//IIC_Stop();//产生一个停止条件
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1);    	//进入接收模式			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
    IIC_Stop();//产生一个停止条件

	return res;
}

/******************************************************************************
*函数原型:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功    能:	    读取指定设备 指定寄存器的length个值
*输    入：		dev 目标设备地址
				reg	 寄存器地址
				length 要读的字节数
				*data  读出的数据将要存放的指针
*返    回：		读出来的值
*******************************************************************************/
//u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
//{
//    u8 count = 0;
//	u8 temp;
//	IIC_Start();
//	IIC_Send_Byte(dev);	   //发送写命令
//	IIC_Wait_Ack();
//	IIC_Send_Byte(reg);   //发送地址
//    IIC_Wait_Ack();	  
//	IIC_Start();
//	IIC_Send_Byte(dev+1);  //进入接收模式
//	IIC_Wait_Ack();
//	
//    for(count=0;count<length;count++){
//		 
//		 if(count!=(length-1))
//		 	temp = IIC_Read_Byte(1);  //带ACK的读数据
//		 else  
//			temp = IIC_Read_Byte(0);	 //最后一个字节NACK

//		data[count] = temp;
//	}
//    IIC_Stop();//产生一个停止条件
//    return count;
//}

//dev为7位的地址
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    u8 count = 0;
	u8 temp;
	IIC_Start();
	IIC_Send_Byte((dev<<1)|0);	   //发送写命令
	if(IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);   //发送地址
    IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte((dev<<1)|1);  //进入接收模式
	IIC_Wait_Ack();
    for(count=0;count<length;count++){	 
		 if(count!=(length-1))
		 	temp = IIC_Read_Byte(1);  //带ACK的读数据
		 else  
			temp = IIC_Read_Byte(0);	 //最后一个字节NACK

		data[count] = temp;
	}
    IIC_Stop();//产生一个停止条件
    return 0;
}


/******************************************************************************
*函数原型:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功    能:	    将多个字节写入指定设备 指定寄存器
*输    入：		dev 目标设备地址
				reg	 寄存器地址
				length 要读的字节数
				*data  读出的数据将要存放的指针
*返    回：		返回是否成功
*******************************************************************************/
//u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
//{
//  
// 	u8 count = 0;
//	IIC_Start();
//	IIC_Send_Byte(dev);	   //发送写命令
//	IIC_Wait_Ack();
//	IIC_Send_Byte(reg);   //发送地址
//    IIC_Wait_Ack();	  
//	for(count=0;count<length;count++){
//		IIC_Send_Byte(data[count]); 
//		IIC_Wait_Ack(); 
//	 }
//	IIC_Stop();//产生一个停止条件

//    return 1; //status == 0;
//}

//dev为7位的地址
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
{
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte((dev<<1)|0);	   //发送写命令
	if(IIC_Wait_Ack()) {
		IIC_Stop();
		return 1;
	}
	IIC_Send_Byte(reg);   //发送地址
    IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		if(IIC_Wait_Ack()) {
			IIC_Stop();
			return 1;
		}
	 }
	IIC_Stop();//产生一个停止条件

    return 0; //status == 0;
}


#include "wifi.h"
#include "sysclock.h"
#include "usart.h"
#include "ANO_DataTransfer.h"

//串口接收DMA缓存
u8 Uart_Send_Buffer[100] = {0};


/**************************实现函数********************************************
*函数原型:		void U1NVIC_Configuration(void)
*功    能:		串口3中断配置
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void UART3NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**************************实现函数********************************************
*函数原型:		void USART3_Config(void)
*功    能:		初始化串口3，PB10,PB11
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void USART3_Config()
{
	//设置USART引脚输入输出模式
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	//使能串口的RCC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                            //推挽复用
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                      //浮空输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	//初始化USART
	USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
    
	/* 使能串口1接收中断 */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);            					//开中断
	
	USART_Cmd(USART3, ENABLE);
	
	UART3NVIC_Configuration();
}

/**************************实现函数********************************************
*函数原型:		void serial3_send_char(u8 temp)
*功    能:		串口3发送字节
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void serial3_send_char(u8 temp)
{
	USART_SendData(USART3, (u8)temp);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

/**************************实现函数********************************************
*函数原型:		void serial3_send_buff(u8 buff[], u32 len)
*功    能:		串口3发送字符串
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void serial3_send_buff(u8 buff[], u32 len)
{
	u32 i;
	for(i=0;i<len;i++)
	    serial3_send_char(buff[i]);
}

/**************************实现函数********************************************
*函数原型:		void USART3_IRQHandler(void)
*功    能:		串口1中断处理函数
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void USART3_IRQHandler(void)
{
//	uint8_t ch;
// 	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)            //发送中断
// 	{
// 		
// 	}
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)     //接收中断
	{
		char ch = USART_ReceiveData(USART3);
		ANO_DT_Data_Receive_Prepare(ch);
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);       //这样写只能接收一个字节
	}	
}

void DMA1_CHANNEL2_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure; 

    //---------------------串口功能配置---------------------
    //打开串口对应的外设时钟  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);   
    //启动DMA时钟
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    //DMA发送中断设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //DMA1通道2配置
    DMA_DeInit(DMA1_Channel2);
    //外设地址
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DR);
    //内存地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Uart_Send_Buffer;
    //dma传输方向单向
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    //设置DMA在传输时缓冲区的长度
    DMA_InitStructure.DMA_BufferSize = 100;
    //设置DMA的外设递增模式，一个外设
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    //设置DMA的内存递增模式
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    //外设数据字长
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    //内存数据字长
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
    //设置DMA的传输模式
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    //设置DMA的优先级别
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    //设置DMA的2个memory中的变量互相访问
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel2,&DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);
     
    //使能通道4
    //DMA_Cmd(DMA1_Channel4, ENABLE);
    
    //设置IO口时钟      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);    
    //串口1的管脚初始化---------------------------------------------  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                       //管脚9  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                //选择GPIO响应速度  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                  //复用推挽输出  
    GPIO_Init(GPIOB, &GPIO_InitStructure);                           //TX初始化  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;                       //管脚10  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;                //选择GPIO响应速度  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;            //浮空输入  
    GPIO_Init(GPIOB, &GPIO_InitStructure);                           //RX初始化  
    
    //初始化参数----------------------------------------------------  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;    
    USART_InitStructure.USART_BaudRate = 115200; 
    //初始化串口 
    USART_Init(USART3,&USART_InitStructure);  
    //TXE发送中断,TC传输完成中断,RXNE接收中断,PE奇偶错误中断,可以是多个   
    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);  
     
    //配置UART3中断--------------------------------------------------  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;               //通道设置为串口1中断  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //中断占先等级0  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;              //中断响应优先级0  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //打开中断  
    NVIC_Init(&NVIC_InitStructure);                                 //初始化  
     
    //采用DMA方式发送
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
    //启动串口  
    USART_Cmd(USART3, ENABLE);   
}

void DMA1_Channel2_Send(u8 *buff, uint16_t size)
{
    uint16_t i;
    for(i=0;i<size;i++) {
        Uart_Send_Buffer[i] = buff[i];
    }
    
    DMA_SetCurrDataCounter(DMA1_Channel2, size); 
    DMA_Cmd(DMA1_Channel2, ENABLE); 
}

void DMA1_Channel2_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA1_FLAG_TC2)==SET)
    {
        DMA_Cmd (DMA1_Channel2,DISABLE);	//关闭DMA通道
        DMA_ClearFlag(DMA1_FLAG_TC2);		//清中断标志，否则会一直中断
//        DMA_SetCurrDataCounter(DMA1_Channel4, 2);//重置传输数目，当再次达到这个数目就会进中断
//        DMA_Cmd(DMA1_Channel4,ENABLE);//开启DMA通道
    }
}

void WIFI_UDP_INIT(void)
{
	u8 *send1 = (u8 *)WIFI_CWMODE;
	u8 *send2 = (u8 *)WIFI_CONN;
	u8 *send3 = (u8 *)WIFI_UDP;
	u8 *send4 = (u8 *)WIFI_CIPMODE;
	u8 *send5 = (u8 *)WIFI_SEND;

	delay_ms(5000);
	serial3_send_buff(send1, WIFI_CWMODE_LENGTH);
	delay_ms(3000);
//	serial3_send_buff(send2, WIFI_CONN_LENGTH);
//	delay_ms(8000);
	serial3_send_buff(send3, WIFI_UDP_LENGTH);
	delay_ms(3000);
	serial3_send_buff(send4, WIFI_CIPMODE_LENGTH);
	delay_ms(3000);
	serial3_send_buff(send5, WIFI_SEND_LENGTH);
	delay_ms(3000);	
}

void WIFI_STOP(void)
{
	u8 *send1 = (u8 *)WIFI_CLOSE1;
	u8 *send2 = (u8 *)WIFI_CLOSE2;
	serial3_send_buff(send1, sizeof(send1));
	delay_ms(500);
	serial3_send_buff(send2, sizeof(send2));
	delay_ms(500);	
}

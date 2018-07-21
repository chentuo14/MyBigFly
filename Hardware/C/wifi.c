#include "wifi.h"
#include "sysclock.h"
#include "usart.h"

/**************************实现函数********************************************
*函数原型:		void U1NVIC_Configuration(void)
*功    能:		串口1中断配置
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void UART3NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
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
//		fPutChar(ch);
//		serial3_send_char(ch);
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);       //这样写只能接收一个字节
	}	
}

void WIFI_UDP_INIT(void)
{
	u8 *send1 = (u8 *)WIFI_CWMODE;
	u8 *send2 = (u8 *)WIFI_CONN;
	u8 *send3 = (u8 *)WIFI_UDP;
	u8 *send4 = (u8 *)WIFI_CIPMODE;
	u8 *send5 = (u8 *)WIFI_SEND;
	u8 *strReturn = (u8 *)"\r\n";

	delay_ms(5000);
	serial3_send_buff(send1, WIFI_CWMODE_LENGTH);
	delay_ms(3000);
	serial3_send_buff(send2, WIFI_CONN_LENGTH);
	delay_ms(8000);
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

#include "usart.h"
#include "mpu_userapi.h"

//////////////////////////////////////////////////////////////////
//加入以下代码，支持printf函数，而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数             
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef' d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送，直到发送完毕
	USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/**************************实现函数********************************************
*函数原型:		void U1NVIC_Configuration(void)
*功    能:		串口1中断配置
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void UART1NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**************************实现函数********************************************
*函数原型:		void USART_Config(void)
*功    能:		初始化串口1，PA9,PA10
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void USART_Config()
{
	//设置USART引脚输入输出模式
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                            //推挽复用
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                      //浮空输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	//初始化USART
	USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
	/* 使能串口1接收中断 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);            					//开中断
	
	USART_Cmd(USART1, ENABLE);
	
}

/****************************************************************************
* Function Name : fPutString
* Description : Send a string.
* Input : None
* Output : None
* Return : None
****************************************************************************/
void fPutString(u8 *buf, u8 len)
{
    u8 i;
    for(i=0;i<len;i++)
    {
        fPutChar(*buf++);
    }
}

/****************************************************************************
* Function Name : fPutChar
* Description : Send a byte
* Input : None
* Output : None
* Return : None
****************************************************************************/
u8 fPutChar(u8 ch)
{
    USART_SendData(USART1, (u8) ch);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
        
    }
    return ch;
}

/**************************实现函数********************************************
*函数原型:		void USART1_IRQHandler(void)
*功    能:		串口1处理函数
*******************************************************************************/
void usart1_handle(void)
{
	float pitch, yaw, roll;
	vs16 gyro_x, gyro_y;
	pitch = yaw = roll = 0;
    char c = USART_ReceiveData(USART1);
	
	switch(c) {
		case 'x':
			mpu_mpl_get_data(&pitch, &roll, &yaw, &gyro_x, &gyro_y);
			printf("pitch %f, roll %f, yaw %f\n", pitch, roll, yaw);
		default:
			break;
	}
}

/**************************实现函数********************************************
*函数原型:		void USART1_IRQHandler(void)
*功    能:		串口1中断处理函数
*输入参数：		无
*输出参数：		无
*******************************************************************************/
void USART1_IRQHandler(void)
{
//	uint8_t ch;
// 	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)            //发送中断
// 	{
// 		
// 	}
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)     //接收中断
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);       //这样写只能接收一个字节
//		printf("It happens receive IRQ \n");
//		usart1_handle();
		
		handle_input();
	}	
}

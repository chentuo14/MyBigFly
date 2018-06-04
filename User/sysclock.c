
#include "sysclock.h"
#include "led.h"

/**
  * @brief  Selects HSE as System clock source and configure HCLK, PCLK2
  *         and PCLK1 prescalers. 
  * @param  None
  * @retval None
  */
void SetSysClockToHSE(void)
{
	ErrorStatus HSEStartUpStatus;
	/* SYSCLK, HCLK, PCLK2 and PCLK1 configuration -----------------------------*/   
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);		//SYSCLK = 8M
	
	/* Disenable LSE */
	RCC_LSEConfig(RCC_LSE_OFF);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if (HSEStartUpStatus == SUCCESS)
	{
		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1); 	//AHB
	  
		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1); 	//High Speed APB

		/* PCLK1 = HCLK */
		RCC_PCLK1Config(RCC_HCLK_Div2);		//Low Speed APB

		/* Flash 0 wait state */
		FLASH_SetLatency(FLASH_Latency_2);
		/*Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		
		/* PLLCLK = 8MHz*9 = 72MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		
		/* Select HSE as system clock source */
//		RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);
		
		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {

		}
		
		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		
		/* Wait till PLL is used as system clock source */
		while (RCC_GetSYSCLKSource() != 0x08)
		{
			
		}
	} else {
		/* If HSE fails to start-up, the application will have wrong clock configuration.
		User can add here some code to deal with this error */    

		/* Go to infinite loop */
		while (1)
		{
		}
	}
}

char SysClock;

void MYRCC_DeInit(void)
{
    RCC->APB1RSTR = 0x00000000;//¸´Î»½áÊø
    RCC->APB2RSTR = 0x00000000;

    RCC->AHBENR = 0x00000014;  //Ë¯ÃßÄ£Ê½ÉÁ´æºÍSRAMÊ±ÖÓÊ¹ÄÜ.ÆäËû¹Ø±Õ.
    RCC->APB2ENR = 0x00000000; //ÍâÉèÊ±ÖÓ¹Ø±Õ.
    RCC->APB1ENR = 0x00000000;
    RCC->CR |= 0x00000001;     //Ê¹ÄÜÄÚ²¿¸ßËÙÊ±ÖÓHSION
    RCC->CFGR &= 0xF8FF0000;   //¸´Î»SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]
    RCC->CR &= 0xFEF6FFFF;     //¸´Î»HSEON,CSSON,PLLON
    RCC->CR &= 0xFFFBFFFF;     //¸´Î»HSEBYP
    RCC->CFGR &= 0xFF80FFFF;   //¸´Î»PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE
    RCC->CIR = 0x00000000;     //¹Ø±ÕËùÓÐÖÐ¶Ï
}

char SystemClock_HSE(u8 PLL)
{
    unsigned char temp=0;
    MYRCC_DeInit();		    //¸´Î»²¢ÅäÖÃÏòÁ¿±í
    RCC->CR|=1<<16;       //Íâ²¿¸ßËÙÊ±ÖÓÊ¹ÄÜHSEON
    while(!(RCC->CR>>17));//µÈ´ýÍâ²¿Ê±ÖÓ¾ÍÐ÷
    RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
    PLL-=2;//µÖÏû2¸öµ¥Î»
    RCC->CFGR|=PLL<<18;   //ÉèÖÃPLLÖµ 2~16
    RCC->CFGR|=1<<16;	    //PLLSRC ON
    FLASH->ACR|=0x32;	    //FLASH 2¸öÑÓÊ±ÖÜÆÚ
    RCC->CR|=0x01000000;  //PLLON
    while(!(RCC->CR>>25));//µÈ´ýPLLËø¶¨
    RCC->CFGR|=0x00000002;//PLL×÷ÎªÏµÍ³Ê±ÖÓ
    while(temp!=0x02)     //µÈ´ýPLL×÷ÎªÏµÍ³Ê±ÖÓÉèÖÃ³É¹¦
    {
        temp=RCC->CFGR>>2;
        temp&=0x03;
    }

    SysClock=(PLL+2)*8;
    return SysClock;
}

/***********SysTick*****************/
// cycles per microsecond
static volatile uint32_t usTicks = 0;
// current uptime for 1kHz systick timer. Will rollover after 49 days. hopefully we won't care.
uint32_t sysTickUptime = 0;

void cycleCounterInit(void)
{
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	usTicks = clocks.SYSCLK_Frequency/1000000;
}

// SysTick
void SysTick_Handler(void)
{
	sysTickUptime++;
}
void DelayMs(uint16_t nms)
{
    uint32_t t0=micros();
    while(micros() - t0 < nms * 1000);
}

void delay_us(u32 nus)
{
	uint32_t t0=micros();
	while(micros() - t0 < nus);
}

void delay_ms(uint16_t nms)
{
	uint32_t t0=micros();
	while(micros() - t0 < nms * 1000);
}

// Return system uptime in microseconds (rollover in 70minutes)
// return us
uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

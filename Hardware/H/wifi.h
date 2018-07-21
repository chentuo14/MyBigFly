#ifndef __USART2_H_
#define __USART2_H_

#include "stm32f10x.h"
#include <stdio.h>

/***************************************************
 * 1.设置客户端模式
 * AT+CWMODE=1
 * 2.连接路由器
 * AT+CWJAP="Garen","12345678"
 * 3.查看连接IP
 * AT+CIFSR
 * 4.连接UDP服务器
 * AT+CIPSTART="UDP","192.168.0.88",8080,9000,0
 * 192.168.0.88是电脑 IP地址，8080表示目标（在此是电脑）UDP端口，9000表示本地端口（ 即ESP8266），0表示远端不变）。
 * 5.开启透传模式
 * AT+CIPMODE=1
 * 6.准备模块与电脑进行互传数据
 * AT+CIPSEND
 * 7.关闭透传模式（下面两行一起用,+++没有换行，第二个有）
+++
AT+SAVETRANSLINK=0
*****************************************************/


#define WIFI_CWMODE "AT+CWMODE=1\r\n"
#define WIFI_CONN "AT+CWJAP=\"CMCC-WEIHUA\",\"1112223334\"\r\n"
#define WIFI_UDP  "AT+CIPSTART=\"UDP\",\"192.168.1.28\",8080,9000,0\r\n"
#define WIFI_CIPMODE "AT+CIPMODE=1\r\n"
#define WIFI_SEND "AT+CIPSEND\r\n"
#define WIFI_CLOSE1 "+++"
#define WIFI_CLOSE2 "AT+SAVETRANSLINK=0\r\n"

#define WIFI_CWMODE_LENGTH 13
#define WIFI_CONN_LENGTH 37
#define WIFI_UDP_LENGTH  46
#define WIFI_CIPMODE_LENGTH 14
#define WIFI_SEND_LENGTH 12
#define WIFI_CLOSE1_LENGTH 3
#define WIFI_CLOSE2_LENGTH 20

void UART3NVIC_Configuration(void);
void USART3_Config();
void serial3_send_char(u8 temp);
void serial3_send_buff(u8 buff[], u32 len);
void WIFI_UDP_INIT(void);


#endif


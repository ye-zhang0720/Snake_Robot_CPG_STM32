//
//  usart.h
//  AX12A_Drive_C
//
//  Created by 西北偏北 on 2020/8/3.
//  Copyright ? 2020 西北偏北. All rights reserved.
//


#ifndef __USART_H
#define __USART_H	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define USART_REC_LEN  			20  	//定义最大接收字节数 200
#define EN_USART3_RX 			1		//使能（1）/禁止（0）串口3接收
#define EN_USART2_RX 			1		//使能（1）/禁止（0）串口2接收



extern u8  USART2_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART2_RX_STA;         		//接收状态标记	





void uart_init(u32 bound);

unsigned char USART1_Send_Data(unsigned char *buffer, unsigned char size);   //串口1发送数据
//unsigned char* USART1_read_Data(void);  //串口接收数据返回
//void setnByteToBeRead(unsigned char a);   //设置接收数据长度
//_Bool available(void);   //串口接收完成



void uart2_init(u32 bound);
unsigned char USART2_Send_Data(unsigned char *buffer, unsigned char size);

#endif



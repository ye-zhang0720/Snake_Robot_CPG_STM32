//
//  usart.h
//  AX12A_Drive_C
//
//  Created by 西北偏北 on 2020/8/3.
//  Copyright ? 2020 西北偏北. All rights reserved.
//


#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 



#define USART_REC_LEN  			20  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

#define readOnly(x)	x->CR1 |= 4;	x->CR1 &= 0xFFFFFFF7;		//??x?????,CR1->RE=1, CR1->TE=0
#define sendOnly(x)	x->CR1 |= 8;	x->CR1 &= 0xFFFFFFFB;		//??x?????,CR1->RE=0, CR1->TE=1


extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 

void uart_init(u32 bound);

unsigned char USART1_Send_Data(unsigned char *buffer, unsigned char size);   //串口1发送数据

void setnByteToBeRead(unsigned char a);   //设置接收数据长度
unsigned char* USART1_read_Data(void);  //串口接收数据返回
_Bool available(void);   //串口接收完成

#endif



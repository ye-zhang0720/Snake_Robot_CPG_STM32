//
//  usart.h
//  AX12A_Drive_C
//
//  Created by ����ƫ�� on 2020/8/3.
//  Copyright ? 2020 ����ƫ��. All rights reserved.
//


#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 



#define USART_REC_LEN  			20  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

#define readOnly(x)	x->CR1 |= 4;	x->CR1 &= 0xFFFFFFF7;		//??x?????,CR1->RE=1, CR1->TE=0
#define sendOnly(x)	x->CR1 |= 8;	x->CR1 &= 0xFFFFFFFB;		//??x?????,CR1->RE=0, CR1->TE=1


extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 

void uart_init(u32 bound);

unsigned char USART1_Send_Data(unsigned char *buffer, unsigned char size);   //����1��������

void setnByteToBeRead(unsigned char a);   //���ý������ݳ���
unsigned char* USART1_read_Data(void);  //���ڽ������ݷ���
_Bool available(void);   //���ڽ������

#endif



//
//  usart.h
//  AX12A_Drive_C
//
//  Created by ����ƫ�� on 2020/8/3.
//  Copyright ? 2020 ����ƫ��. All rights reserved.
//


#ifndef __USART_H
#define __USART_H	
#include "stm32f4xx_conf.h"
#include "sys.h" 

#define USART_REC_LEN  			20  	//�����������ֽ��� 200
#define EN_USART3_RX 			1		//ʹ�ܣ�1��/��ֹ��0������3����
#define EN_USART2_RX 			1		//ʹ�ܣ�1��/��ֹ��0������2����



extern u8  USART2_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART2_RX_STA;         		//����״̬���	





void uart_init(u32 bound);

unsigned char USART1_Send_Data(unsigned char *buffer, unsigned char size);   //����1��������
//unsigned char* USART1_read_Data(void);  //���ڽ������ݷ���
//void setnByteToBeRead(unsigned char a);   //���ý������ݳ���
//_Bool available(void);   //���ڽ������



void uart2_init(u32 bound);
unsigned char USART2_Send_Data(unsigned char *buffer, unsigned char size);

#endif



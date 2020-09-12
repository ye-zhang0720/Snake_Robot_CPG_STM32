//
//  usart.c
//  AX12A_Drive_C
//
//  Created by ����ƫ�� on 2020/8/3.
//  Copyright ? 2020 ����ƫ��. All rights reserved.
//

#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 

	

unsigned char nByteToBeRead;                   // Used to check when Rx packet is complete


#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//__use_no_semihosting was requested, but _ttywrch was 
void _ttywrch(int ch)
{
    ch = ch;
}
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
	USART2->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u8 USART_RX_STA=0;       //����״̬���	

//��ʼ��IO ����1  ���ʹ�ô���
//bound:������
void uart_init(u32 bound){

	
	GPIO_InitTypeDef GPIO_InitStructure;  
	USART_InitTypeDef USART_InitStructure;   
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); 
	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;      //PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);  


	USART_InitStructure.USART_BaudRate = bound;  
 	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  
	USART_InitStructure.USART_Parity = USART_Parity_No;  
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
	//USART_InitStructure.USART_Mode = USART_Mode_Rx ;
	USART_Init(USART1,&USART_InitStructure);  
	USART_HalfDuplexCmd(USART1, ENABLE);  	

	USART_Cmd(USART1,ENABLE);  
 	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	
	
	readOnly(USART1);	  //����1ֻ��
	
}


void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntEnter();    
#endif
	
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
	{

		Res = USART_ReceiveData(USART1);
		USART_RX_BUF[USART_RX_STA] = Res;
		USART_RX_STA++;
	} 
	
	
#ifdef OS_TICKS_PER_SEC	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntExit();  											 
#endif
} 
#endif	

 
unsigned char USART1_Send_Data(unsigned char *buffer, unsigned char size)
{
	
	sendOnly(USART1);

	for (int i=0; i<size; i++) {
		USART_SendData(USART1, buffer[i]);         //send massage through USART1
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);     //wait for sending finish
	}

	readOnly(USART1);	
	return size;
}


void setnByteToBeRead(unsigned char a)
{
    nByteToBeRead = a;
}

unsigned char* USART1_read_Data()
{
	return USART_RX_BUF;
}

_Bool available(void)
{
	if(nByteToBeRead == USART_RX_STA)
	{
		//USART1_Send_Data(&USART_RX_STA,1);
		USART_RX_STA = 0;
		return 1;
	}
		return 0;
}


//����2 ����ͨ�Ŵ���


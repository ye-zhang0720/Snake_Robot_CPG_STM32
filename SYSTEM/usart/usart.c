//
//  usart.c
//  AX12A_Drive_C
//
//  Created by 西北偏北 on 2020/8/3.
//  Copyright ? 2020 西北偏北. All rights reserved.
//

#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 

	

unsigned char nByteToBeRead;                   // Used to check when Rx packet is complete


#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//__use_no_semihosting was requested, but _ttywrch was 
void _ttywrch(int ch)
{
    ch = ch;
}
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
	USART2->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u8 USART_RX_STA=0;       //接收状态标记	

//初始化IO 串口1  电机使用串口
//bound:波特率
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
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
	
	readOnly(USART1);	  //串口1只读
	
}


void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntEnter();    
#endif
	
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
	{

		Res = USART_ReceiveData(USART1);
		USART_RX_BUF[USART_RX_STA] = Res;
		USART_RX_STA++;
	} 
	
	
#ifdef OS_TICKS_PER_SEC	 	//如果时钟节拍数定义了,说明要使用ucosII了.
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


//串口2 电脑通信串口


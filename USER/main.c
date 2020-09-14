//
//  main.c
//  AX12A_Drive_C
//
//  Created by ����ƫ�� on 2020/8/3.
//  Copyright ? 2020 ����ƫ��. All rights reserved.
//

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "Dynamixel.h"
#include "Matrix.h"
#include "Hopf_CPG.h"
#include "memory_manage.h"	  
#include "Motor.h"

#define pi 3.1415926

int main(void)
{ 
	unsigned short temp = 0;
	u8 a[] = {0x0D,0x0A};   //���з�
	u16 times=0;  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		//��ʱ��ʼ�� 
	uart_init(115200);	//���ڳ�ʼ��������Ϊ115200
	uart2_init(115200); //���ڳ�ʼ��������Ϊ115200
	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
	
	my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ��
	my_mem_init(SRAMCCM);		//��ʼ��CCM�ڴ��  
	
	//CPG����
  //��ʼֵ
  int n = 7;
  float lambda = 1;          //������
  float omega = 0.5*pi;      //��Ƶ��
  float sigma = 1;           //�ֲ����
  float step = 0.01;         //����
	
	 //��ϲ�������
  double g = 2;     //ȫ�����ϵ��
  Matrix *W = InitMatrix(W,n,n);
    
  for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			if (i == j) {
                ValueOneMatrix(W, 0, i, j);
            }else if (i == j + 1){
                ValueOneMatrix(W, 0.5, i, j);
            }else if (j == i + 1){
                ValueOneMatrix(W, 0.5, i, j);
            }
        }
    }
    
   Matrix *Rho = InitMatrix(Rho,1,n);
   OnesMatrix(Rho);      //��wȫ����ֵ1

   //��j����������i����������λƫ�Ʋ�
   float Phicha = pi *3 / n;
   Matrix *Phi = InitMatrix(Phi,n,n);
	
	      for (int i = 0; i < n; i++)
      {
          for (int j = 0; j < n; j++)
          {
              if ( i == j)
              {
                  ValueOneMatrix(Phi, 0, i, j);
              }else {
                  ValueOneMatrix(Phi, Phicha * (j - i), i, j);
              }
          }
          
      }


	Matrix *X = InitMatrix(X,n,2);
  for (int i = 0; i < X->row; i++)        //�趨��ʼλ�á�u,v��= ��0��0.01��
  {
		ValueOneMatrix(X, 0, i, 0);
    ValueOneMatrix(X, 0.01, i, 1);
  }
      

  //�����Ķ���(0,0)
  Matrix *center = InitMatrix(center,1,2);
  ZerosMatrix(center);
      
  double u = 0;
  double v = 0;
	
	//�м��������
  Matrix *X_now;	
	Matrix *coup;
	Matrix *hopf;
	
	
	
	printf("�ڴ�ʹ���ʣ�%d\n", my_mem_perused(SRAMIN));
	USART2_Send_Data(a, 2);
	
	
	//printf("%d   %d",X->row,X->column);
	//USART2_Send_Data(a, 2);
	//PrintMatrix(W);
	while(1)
	{
		//USART2�������ݷ���
		if(USART2_RX_STA&0x8000)
		{					   
//			len=USART2_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
//			printf("\r\n�����͵���ϢΪ:\r\n");
//			for(t=0;t<len;t++)
//			{
//				USART_SendData(USART2, USART2_RX_BUF[t]);         //�򴮿�1��������
//				while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
//			}
//			printf("\r\n\r\n");//���뻻��
//			USART2_RX_STA=0;
		}
		
		
		if(times%300 == 0)
		{
			//ping(1);
			
			for(int j = 0; j < n; j++)
      {
				u = GetValue(X, j, 0) - center->data[0];
        v = GetValue(X, j, 1) - center->data[1];
				//�ͷ��ڴ�
				FreeMatrix(coup);
				FreeMatrix(hopf);
				FreeMatrix(X_now);
				
				coup = Coupling(g, W, Rho, Phi, X, j);
				hopf = Hopf(u ,v , step, lambda, omega, sigma, Rho->data[j], coup);			
        X_now = AddMatrix(hopf, center);
        ValueOneMatrix(X, X_now->data[0], j, 0);
        ValueOneMatrix(X, X_now->data[1], j, 1);

				
        MotorMove(j, AngletoPostion(GetValue(X, 2*j, 0)));
        //printf("%lf\t ",GetValue(X, j, 0));
      }	
		}
		
		
		if(times%500 == 0){
			
		}
		
//		if(times%1000 == 0){
//			p = mymalloc(SRAMIN,2048);
//			myfree(SRAMIN,p);
//		}
		
		if(times%30==0)
		{
			printf("�ڴ�ʹ���ʣ�%d\n", my_mem_perused(SRAMIN));
			USART2_Send_Data(a, 2);
			temp = readDxlResult();
	
			
//			if(temp != 0xffff)
//			{
//				unsigned char tmp[2];
//				tmp[0] = (unsigned char)temp%0xff;
//				tmp[1] = (unsigned char)temp/0xff;
//				USART1_Send_Data(tmp,2);
//			}
		}
		if(times%30==0) LED0=!LED0;//��˸LED,��ʾϵͳ��������.
		delay_ms(10);   
		times++;
	}
}


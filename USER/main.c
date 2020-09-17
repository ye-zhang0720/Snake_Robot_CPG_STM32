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
	u8 a[] = {0x0D,0x0A};   //���з�
	u16 times=0;  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		//��ʱ��ʼ�� 
	ServoUSART_Init(115200);	//���ڳ�ʼ��������Ϊ115200
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
	
	
	
	printf("\n\n\n�ڴ�ʹ���ʣ�%d\n", my_mem_perused(SRAMIN));
	USART2_Send_Data(a, 2);
	
	u16 len = 0;
	
			u8 freq = 5;
	 u8 t=0;
	
				while(!pingServo (1))			               //������ɹ�Ping,������ѭ��
			{
					printf ("Ping failed\n");   
			}
					printf("Ping OK\n");
	
	while(1)
	{

		//USART1�������ݷ���
//		if(USART_RX_STA&0x8000)
//		{				
//			
//			len=USART_RX_STA&0x7FFF;//�õ��˴ν��յ������ݳ���
////			printf("\r\n�õ�������Ϊ:\r\n");
//			for(int t=0;t<len;t++)
//			{
//				printf("%x",USART_RX_BUF[t]);
//			}
//			printf("\r\n\r\n");//���뻻��
//			USART_RX_STA=0;
//		}
		

		if(times%300 == 0)
		{
			
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

				
        setServoAngle(j, GetValue(X, 2*j, 0)*180/pi);

      }	
		}
		

		if(times%500 == 0){
			

		}

		if(times%300==0)
		{
//			printf("�ڴ�ʹ���ʣ�%d\n", my_mem_perused(SRAMIN));
//			USART2_Send_Data(a, 2);
			
//		printf("%hd",USART_RX_STA);
//		USART2_Send_Data(a, 2);
		}
		if(times%30==0) LED0=!LED0;//��˸LED,��ʾϵͳ��������.
		delay_ms(10);   
		times++;
	}
}


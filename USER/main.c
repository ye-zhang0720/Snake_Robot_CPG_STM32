//
//  main.c
//  AX12A_Drive_C
//
//  Created by 西北偏北 on 2020/8/3.
//  Copyright ? 2020 西北偏北. All rights reserved.
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
	u8 a[] = {0x0D,0x0A};   //换行符
	u16 times=0;  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		//延时初始化 
	ServoUSART_Init(115200);	//串口初始化波特率为115200
	uart2_init(115200); //串口初始化波特率为115200
	LED_Init();		  		//初始化与LED连接的硬件接口
	
	my_mem_init(SRAMIN);		//初始化内部内存池
	my_mem_init(SRAMCCM);		//初始化CCM内存池  
	
	//CPG定义
  //初始值
  int n = 7;
  float lambda = 1;          //吸引率
  float omega = 0.5*pi;      //震荡频率
  float sigma = 1;           //分叉参数
  float step = 0.01;         //步长
	
	 //耦合参数定义
  double g = 2;     //全局耦合系数
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
   OnesMatrix(Rho);      //将w全部赋值1

   //第j个振荡器到第i个振荡器的相位偏移差
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
  for (int i = 0; i < X->row; i++)        //设定初始位置【u,v】= 【0，0.01】
  {
		ValueOneMatrix(X, 0, i, 0);
    ValueOneMatrix(X, 0.01, i, 1);
  }
      

  //震荡中心定义(0,0)
  Matrix *center = InitMatrix(center,1,2);
  ZerosMatrix(center);
      
  double u = 0;
  double v = 0;
	
	//中间变量定义
  Matrix *X_now;	
	Matrix *coup;
	Matrix *hopf;
	
	
	
	printf("\n\n\n内存使用率：%d\n", my_mem_perused(SRAMIN));
	USART2_Send_Data(a, 2);
	
	u16 len = 0;
	
			u8 freq = 5;
	 u8 t=0;
	
				while(!pingServo (1))			               //如果不成功Ping,进入死循环
			{
					printf ("Ping failed\n");   
			}
					printf("Ping OK\n");
	
	while(1)
	{

		//USART1接收数据分析
//		if(USART_RX_STA&0x8000)
//		{				
//			
//			len=USART_RX_STA&0x7FFF;//得到此次接收到的数据长度
////			printf("\r\n得到的数据为:\r\n");
//			for(int t=0;t<len;t++)
//			{
//				printf("%x",USART_RX_BUF[t]);
//			}
//			printf("\r\n\r\n");//插入换行
//			USART_RX_STA=0;
//		}
		

		if(times%300 == 0)
		{
			
			for(int j = 0; j < n; j++)
      {
				u = GetValue(X, j, 0) - center->data[0];
        v = GetValue(X, j, 1) - center->data[1];
				//释放内存
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
//			printf("内存使用率：%d\n", my_mem_perused(SRAMIN));
//			USART2_Send_Data(a, 2);
			
//		printf("%hd",USART_RX_STA);
//		USART2_Send_Data(a, 2);
		}
		if(times%30==0) LED0=!LED0;//闪烁LED,提示系统正在运行.
		delay_ms(10);   
		times++;
	}
}


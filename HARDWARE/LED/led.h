#ifndef __LED_H
#define __LED_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 


//LED端口定义
#define LED0 PFout(9)	// LED2
#define LED1 PFout(10)	// LED3	 

void LED_Init(void);//初始化		 				    
#endif

#ifndef __Motor_H
#define __Motor_H	 

#include "Dynamixel.h"

	 			

int AngletoPostion(double angle);
void MotorMove(int id, int postion);
void MotorMoveBySpeed(int id, int postion,int MotorSpeed);
#endif



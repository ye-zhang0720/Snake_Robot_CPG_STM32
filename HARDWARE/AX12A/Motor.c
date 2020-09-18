#include "Motor.h"


float Motor_Status[MotorNum][2]={0};     //��һ�����أ��ڶ��нǶ�, id��0��ʼ���



void setMotorTorque(const unsigned int servoId, const float torque)
{
	Motor_Status[servoId][0] = torque;
}

void setMotorAngle(const unsigned int servoId, const float angle)
{
	Motor_Status[servoId][1] = angle;
}

float getMotorTorque(const unsigned int servoId)
{
	return Motor_Status[servoId][0];
}

float getMotorAngle(const unsigned int servoId)
{
	return Motor_Status[servoId][1];
}


#ifndef __MOTOR_H
#define __MOTOR_H


#define MotorNum 7          //关节数量


//////////////////////////////////////////////////////////////////////////////////	 


void setMotorTorque(const unsigned int servoId, const float torque);
void setMotorAngle(const unsigned int servoId, const float angle);
float getMotorTorque(const unsigned int servoId);
float getMotorAngle(const unsigned int servoId);

#endif



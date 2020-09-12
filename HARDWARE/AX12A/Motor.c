#include "Motor.h"

//电机信息
const int GoalPostionUpLim = 300;   //电机角度上限（度）
const int GoalPostionDownLim = 0;   //电机角度下限（度）
const int GPcodeUpLim = 1023;      //电机角度编码上限（度）
const int GPcodeDownLim = 0;        //电机角度编码下限（度）


//将偏转角转换为电机角度
int AngletoPostion(double angle) {
  //形参 单位度
  int temp = (int)(GPcodeUpLim - GPcodeDownLim + 1) / (GoalPostionUpLim - GoalPostionDownLim) * angle;
  return (temp + (GPcodeUpLim - GPcodeDownLim + 1) / 2);
}

//电机转动
void MotorMove(int id, int postion) {
  //while (!dxlCom.isBusy());
  //while (!pingMotors(id));
  setGoalPositionAtSpeed(id, postion,500);
}
//电机转动
void MotorMoveBySpeed(int id, int postion,int MotorSpeed) {
  //while (!dxlCom.isBusy());
  //while (!pingMotors(id));
  setGoalPositionAtSpeed(id, postion,MotorSpeed);
}



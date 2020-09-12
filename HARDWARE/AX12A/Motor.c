#include "Motor.h"

//�����Ϣ
const int GoalPostionUpLim = 300;   //����Ƕ����ޣ��ȣ�
const int GoalPostionDownLim = 0;   //����Ƕ����ޣ��ȣ�
const int GPcodeUpLim = 1023;      //����Ƕȱ������ޣ��ȣ�
const int GPcodeDownLim = 0;        //����Ƕȱ������ޣ��ȣ�


//��ƫת��ת��Ϊ����Ƕ�
int AngletoPostion(double angle) {
  //�β� ��λ��
  int temp = (int)(GPcodeUpLim - GPcodeDownLim + 1) / (GoalPostionUpLim - GoalPostionDownLim) * angle;
  return (temp + (GPcodeUpLim - GPcodeDownLim + 1) / 2);
}

//���ת��
void MotorMove(int id, int postion) {
  //while (!dxlCom.isBusy());
  //while (!pingMotors(id));
  setGoalPositionAtSpeed(id, postion,500);
}
//���ת��
void MotorMoveBySpeed(int id, int postion,int MotorSpeed) {
  //while (!dxlCom.isBusy());
  //while (!pingMotors(id));
  setGoalPositionAtSpeed(id, postion,MotorSpeed);
}



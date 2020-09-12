#ifndef __MATRIX_H
#define __MATRIX_H

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "memory_manage.h"	  

typedef struct
{
    int column,row;        //rowΪ��,columnΪ��
    double *data;
}Matrix;

Matrix* InitMatrix(Matrix *matrix,int row,int column);        //��ʼ������
void ValueMatrix(Matrix *matrix,double* array[]);                //��һ������ֵ
int SizeMatrix(Matrix *matrix);                                //���һ������Ĵ�С
void FreeMatrix(Matrix *matrix);                            //�ͷ�һ������
void CopyMatrix(Matrix *matrix_A, Matrix *matrix_B);        //����һ�������ֵ����A������B
void PrintMatrix(Matrix *matrix);                            //��ӡһ������
double GetValue(Matrix *matrix, int i, int j);              //��ȡ�����ֵ
void ValueOneMatrix(Matrix *matrix, double x, int i, int j);    //��i��j��ֵx

//����Ļ�������
void NumMulMatrix(Matrix *matrix_A, double a);           //�����һ����
void NumAddMatrix(Matrix *matrix_A, double a);           //�����һ����
void NumCutMatrix(Matrix *matrix_A, double a);           //�����һ����
Matrix* AddMatrix(Matrix *matrix_A,Matrix *matrix_B);        //����ļӷ�
Matrix* CutMatrix(Matrix *matrix_A,Matrix *matrix_B);        //����ļ���
Matrix* MulMatrix(Matrix *matrix_A,Matrix *matrix_B);        //����ĳ˷�
void TransMatrix(Matrix *matrix);                            //ת��
void OnesMatrix(Matrix *matrix);                            //����ȫ��1�ľ���
void ZerosMatrix(Matrix *matrix);                           //����ȫ��0�ľ���
void XMatrix(Matrix *matrix, double x);                     //����ȫ��x�ľ���
void IdentityMatrix(Matrix *matrix);                        //���ɵ�λ����
Matrix* Inv(Matrix *matrix);    //���ò�����Ԫ�ĸ�˹��ȥ������A�������B
Matrix* cross(Matrix *A, Matrix *B);                  //��������������

#endif


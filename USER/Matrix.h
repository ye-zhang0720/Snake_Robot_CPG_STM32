#ifndef __MATRIX_H
#define __MATRIX_H

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "memory_manage.h"	  

typedef struct
{
    int column,row;        //row为行,column为列
    double *data;
}Matrix;

Matrix* InitMatrix(Matrix *matrix,int row,int column);        //初始化矩阵
void ValueMatrix(Matrix *matrix,double* array[]);                //给一个矩阵赋值
int SizeMatrix(Matrix *matrix);                                //获得一个矩阵的大小
void FreeMatrix(Matrix *matrix);                            //释放一个矩阵
void CopyMatrix(Matrix *matrix_A, Matrix *matrix_B);        //复制一个矩阵的值，从A拷贝到B
void PrintMatrix(Matrix *matrix);                            //打印一个矩阵
double GetValue(Matrix *matrix, int i, int j);              //读取矩阵的值
void ValueOneMatrix(Matrix *matrix, double x, int i, int j);    //给i，j赋值x

//矩阵的基本运算
void NumMulMatrix(Matrix *matrix_A, double a);           //矩阵乘一个数
void NumAddMatrix(Matrix *matrix_A, double a);           //矩阵加一个数
void NumCutMatrix(Matrix *matrix_A, double a);           //矩阵减一个数
Matrix* AddMatrix(Matrix *matrix_A,Matrix *matrix_B);        //矩阵的加法
Matrix* CutMatrix(Matrix *matrix_A,Matrix *matrix_B);        //矩阵的减法
Matrix* MulMatrix(Matrix *matrix_A,Matrix *matrix_B);        //矩阵的乘法
void TransMatrix(Matrix *matrix);                            //转置
void OnesMatrix(Matrix *matrix);                            //生成全是1的矩阵
void ZerosMatrix(Matrix *matrix);                           //生成全是0的矩阵
void XMatrix(Matrix *matrix, double x);                     //生成全是x的矩阵
void IdentityMatrix(Matrix *matrix);                        //生成单位矩阵
Matrix* Inv(Matrix *matrix);    //采用部分主元的高斯消去法求方阵A的逆矩阵B
Matrix* cross(Matrix *A, Matrix *B);                  //求向量的向量积

#endif


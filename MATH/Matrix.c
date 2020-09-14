#include "Matrix.h"
#include "usart.h"	
#include "memory_manage.h"	 

Matrix* InitMatrix(Matrix *matrix,int row,int column)                //初始化一个矩阵
{
    if (column>0 && row>0)
    {
        matrix = (Matrix*)mymalloc(SRAMIN,sizeof(Matrix));
        matrix->column = column;
        matrix->row = row;
        matrix->data = (double*)mymalloc(SRAMIN,sizeof(double)*column*row);
        ZerosMatrix(matrix);
        return matrix;
    }
    else
        return NULL;
}

void ValueMatrix(Matrix *matrix,double* array[])         //给矩阵赋值
{
    if (matrix->data != NULL)
    {
        mymemcpy(matrix->data, array, matrix->column*matrix->row*sizeof(double));
    }
}

int SizeMatrix(Matrix *matrix)
{
    return matrix->column*matrix->row;
}

void FreeMatrix(Matrix *matrix)
{
	myfree(SRAMIN, matrix->data);
    myfree(SRAMIN,matrix);        //释放掉矩阵
//    if (matrix->data == NULL)
//        printf("释放成功\n");
//    else
//        printf("释放失败\n");
}

//从A拷贝到B
void CopyMatrix(Matrix *matrix_A, Matrix *matrix_B)
{
    matrix_B->column = matrix_A->column;
    matrix_B->row = matrix_A->row;
    mymemcpy(matrix_B->data, matrix_A->data, SizeMatrix(matrix_A)*sizeof(double));
}

//打印矩阵
void PrintMatrix(Matrix *matrix)
{
	u8 a[] = {0x0D,0x0A}; 
    if(!matrix)
    {
        return;
    }
    int i = 0;
    for (i=0;i<SizeMatrix(matrix);i++)
    {
        printf("%lf\t", matrix->data[i]);
        if ((i+1)%matrix->column == 0)
            USART2_Send_Data(a, 2);
    }
    
    
}

//读取矩阵值
double GetValue(Matrix *matrix, int i, int j)
{
    if (i > matrix->row || j > matrix->column)
    {
//        printf("无法读取矩阵\n");
        return 0;
    }else
    {
        return matrix->data[i*matrix->column + j];
    }
    
}

//给i，j赋值x
void ValueOneMatrix(Matrix *matrix, double x, int i, int j)
{
    if (i >= matrix->row || j >= matrix->column)
    {
//        printf("无法读取矩阵\n");
    }else
    {
        matrix->data[i*matrix->column + j] = x;
    }
}

//加法
Matrix* AddMatrix(Matrix *matrix_A,Matrix *matrix_B)
{
    int i,j;
    if (matrix_A->column == matrix_B->column && matrix_A->row == matrix_B->row)
    {
        Matrix *matrix_C = InitMatrix(matrix_C,matrix_A->row,matrix_A->column);
        for (i=0;i<matrix_A->row;i++)
        {
            for (j=0;j<matrix_A->column;j++)
            {
                matrix_C->data[i*matrix_C->column + j] = \
                matrix_A->data[i*matrix_A->column + j] + matrix_B->data[i*matrix_A->column + j];
            }
        }
        return matrix_C;
    }
    else
    {
//        printf("不可相加\n");
        return NULL;
    }
}

//减法
Matrix* CutMatrix(Matrix *matrix_A,Matrix *matrix_B)
{
    int i,j;
    if (matrix_A->column == matrix_B->column && matrix_A->row == matrix_B->row)
    {
        Matrix *matrix_C = InitMatrix(matrix_C,matrix_A->row,matrix_A->column);
        for (i=0;i<matrix_A->row;i++)
        {
            for (j=0;j<matrix_A->column;j++)
            {
                matrix_C->data[i*matrix_C->column + j] = \
                matrix_A->data[i*matrix_A->column + j] - matrix_B->data[i*matrix_A->column + j];
            }
        }
        return matrix_C;
    }
    else
    {
//        printf("不可相减\n");
        return NULL;
    }
}

//乘一个数
void NumMulMatrix(Matrix *matrix_A, double a)
{
    int i;
    for (i = 0; i < matrix_A->row * matrix_A->column; i++)
    {
        matrix_A->data[i] = matrix_A->data[i] * a;
    }
}

//加一个数
void NumAddMatrix(Matrix *matrix_A, double a)
{
    int i;
    for (i = 0; i < matrix_A->row * matrix_A->column; i++)
    {
        matrix_A->data[i] = matrix_A->data[i] + a;
    }
}

//减一个数
void NumCutMatrix(Matrix *matrix_A, double a)
{
    int i;
    for (i = 0; i < matrix_A->row * matrix_A->column; i++)
    {
        matrix_A->data[i] = matrix_A->data[i] - a;
    }
}


//乘法
Matrix* MulMatrix(Matrix *matrix_A,Matrix *matrix_B)
{
    //    printf("\n\n\n");
    //    PrintMatrix(matrix_A);
    //    printf("\n\n\n");
    //    PrintMatrix(matrix_B);
    
    int i,j,k;
    if (matrix_A->column == matrix_B->row)        //列==行
    {
        Matrix *matrix_C = InitMatrix(matrix_C,matrix_A->row,matrix_B->column);
        
        for (i=0;i<matrix_A->row;i++)
        {
            for (j=0;j<matrix_B->column;j++)
            {
                for (k=0;k<matrix_A->column;k++)
                {
                    matrix_C->data[i*matrix_C->column + j] += matrix_A->data[i*matrix_A->column + k] * matrix_B->data[k*matrix_B->column + j];
                }
            }
        }
        return matrix_C;
    }
    else
    {
//        printf("不可相乘\n");
        return NULL;
    }
}

//矩阵转置
void TransMatrix(Matrix *matrix)
{
    int i,j;
    if (matrix->column == matrix->row)
    {
        Matrix *matrixTemp = InitMatrix(matrixTemp, matrix->row,matrix->column);           //创建一个临时矩阵
        CopyMatrix(matrix,matrixTemp);    //将目标矩阵的data复制给临时矩阵
        
        for (i=0;i<matrix->column;i++)
        {
            for (j=0;j<matrix->row;j++)
            {
                matrix->data[i*matrix->column + j] = matrixTemp->data[j*matrix->column + i];
            }
        }
				FreeMatrix(matrixTemp);
    }
    else
    {
        Matrix *matrixTemp = InitMatrix(matrixTemp, matrix->row,matrix->column);           //创建一个临时矩阵
        CopyMatrix(matrix,matrixTemp);    //将目标矩阵的data复制给临时矩阵
        
        matrix->column = matrixTemp->row;
        matrix->row = matrixTemp->column;
        
        for (i=0;i<matrix->row;i++)
        {
            for (j=0;j<matrix->column;j++)
            {
                matrix->data[i*matrix->column + j] = matrixTemp->data[j*matrixTemp->column + i];
                
            }
        }
				FreeMatrix(matrixTemp);
    }
}

//生成全是1的矩阵
void OnesMatrix(Matrix *matrix)
{
    int i;
    for (i = 0; i < matrix->column * matrix->row; i++)
    {
        matrix->data[i] = 1;
    }
}

//生成全是0的矩阵
void ZerosMatrix(Matrix *matrix)
{
    int i;
    for (i = 0; i < matrix->column * matrix->row; i++)
    {
        matrix->data[i] = 0;
    }
}

//生成全是x的矩阵
void XMatrix(Matrix *matrix, double x)
{
    int i;
    for (i = 0; i < matrix->column * matrix->row; i++)
    {
        matrix->data[i] = x;
    }
}


//生成单位矩阵
void IdentityMatrix(Matrix *matrix)
{
    for (int i = 0; i < matrix->row; i++) {
        for (int j = 0; j < matrix->column; j++) {
            ValueOneMatrix(matrix, ((i == j) ? (double)1 : 0), i, j);
        }
    }
}



//------------------------------------------------------------------
//功能: 采用部分主元的高斯消去法求方阵A的逆矩阵B
//入口参数: 输入方阵
//返回值:
//------------------------------------------------------------------- 


Matrix* Inv(Matrix* A)
{
    int i, j, k;
    double max, temp;
    
    if (A->column != A->row) {
        return NULL;
    }
    
    int n = A->row;   //矩阵阶数
    Matrix *B = InitMatrix(B, A->row, A->column);      //结果矩阵
    Matrix *t = InitMatrix(t, A->row, A->column);                //临时矩阵
    //将A矩阵存放在临时矩阵中
    
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            ValueOneMatrix(t, GetValue(A, i, j), i, j);
        }
    }
    
    //初始化B矩阵为单位阵
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            ValueOneMatrix(B, ((i == j) ? (double)1 : 0), i, j);
        }
    }
    
    for (i = 0; i < n; i++)
    {
        //寻找主元
        max = GetValue(t, i, i);
        k = i;
        for (j = i + 1; j < n; j++)
        {
            if (fabs(GetValue(t, j, i)) > fabs(max))
            {
                max = GetValue(t, j, i);
                k = j;
            }
        }
        //如果主元所在行不是第i行，进行行交换
        if (k != i)
        {
            for (j = 0; j < n; j++)
            {
                temp = GetValue(t, i, j);
                ValueOneMatrix(t, GetValue(t, k, j), i, j);
                ValueOneMatrix(t, temp, k, j);

                //B伴随交换
                temp = GetValue(B, i, j);
                ValueOneMatrix(B, GetValue(B, k, j), i, j);
                ValueOneMatrix(B, temp, k, j);

            }
        }
        //判断主元是否为0, 若是, 则矩阵A不是满秩矩阵,不存在逆矩阵
        if (GetValue(t, i, i) == 0)
        {
//            printf("There is no inverse matrix!");
            return 0;
        }
        
        //消去A的第i列除去i行以外的各行元素
        temp = GetValue(t, i, i);
        for (j = 0; j < n; j++)
        {
            ValueOneMatrix(t, GetValue(t, i, j)/temp, i, j);   //主对角线上的元素变为1
            ValueOneMatrix(B, GetValue(B, i, j)/temp, i, j);   //伴随计算

        }
        for (j = 0; j < n; j++)        //第0行->第n行
        {
            if (j != i)                //不是第i行
            {
                temp = GetValue(t, j, i);
                for (k = 0; k < n; k++)        //第j行元素 - i行元素*j列i行元素
                {
                    ValueOneMatrix(t, GetValue(t, j, k) - GetValue(t, i, k)*temp, j, k);
                    ValueOneMatrix(B, GetValue(B, j, k) - GetValue(B, i, k)*temp, j, k);
                }
            }
        }
    }
		FreeMatrix(t);
    return B;
}


//求向量的向量积
Matrix* cross(Matrix *A, Matrix *B)
{
    if((A->row == 3 && A->column  == 1)  && (B->row == 3 && B->column  == 1))
    {

        Matrix *C = InitMatrix(C, 3, 1);       //结果矩阵
        ValueOneMatrix(C, A->data[1]*B->data[2] - A->data[2]*B->data[1], 0, 0);
        ValueOneMatrix(C, A->data[2]*B->data[0] - A->data[0]*B->data[2], 1, 0);
        ValueOneMatrix(C, A->data[0]*B->data[1] - A->data[1]*B->data[0], 2, 0);
        return C;
    }else if ( (A->row == 1 && A->column  == 3) && (B->row == 1 && B->column  == 3) || ((A->row == 1 && A->column  == 3)  && (B->row == 3 && B->column  == 1)) || ((A->row == 3 && A->column  == 1)  && (B->row == 1 && B->column  == 3)) )
    {
        
        Matrix *C = InitMatrix(C, 1, 3);       //结果矩阵
        ValueOneMatrix(C, A->data[1]*B->data[2] - A->data[2]*B->data[1], 0, 0);
        ValueOneMatrix(C, A->data[2]*B->data[0] - A->data[0]*B->data[2], 0, 1);
        ValueOneMatrix(C, A->data[0]*B->data[1] - A->data[1]*B->data[0], 0, 2);
        return C;
    }
//    printf("矩阵不合法\n");
    return NULL;
}

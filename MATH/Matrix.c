#include "Matrix.h"
#include "usart.h"	
#include "memory_manage.h"	 

Matrix* InitMatrix(Matrix *matrix,int row,int column)                //��ʼ��һ������
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

void ValueMatrix(Matrix *matrix,double* array[])         //������ֵ
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
    myfree(SRAMIN,matrix);        //�ͷŵ�����
//    if (matrix->data == NULL)
//        printf("�ͷųɹ�\n");
//    else
//        printf("�ͷ�ʧ��\n");
}

//��A������B
void CopyMatrix(Matrix *matrix_A, Matrix *matrix_B)
{
    matrix_B->column = matrix_A->column;
    matrix_B->row = matrix_A->row;
    mymemcpy(matrix_B->data, matrix_A->data, SizeMatrix(matrix_A)*sizeof(double));
}

//��ӡ����
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

//��ȡ����ֵ
double GetValue(Matrix *matrix, int i, int j)
{
    if (i > matrix->row || j > matrix->column)
    {
//        printf("�޷���ȡ����\n");
        return 0;
    }else
    {
        return matrix->data[i*matrix->column + j];
    }
    
}

//��i��j��ֵx
void ValueOneMatrix(Matrix *matrix, double x, int i, int j)
{
    if (i >= matrix->row || j >= matrix->column)
    {
//        printf("�޷���ȡ����\n");
    }else
    {
        matrix->data[i*matrix->column + j] = x;
    }
}

//�ӷ�
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
//        printf("�������\n");
        return NULL;
    }
}

//����
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
//        printf("�������\n");
        return NULL;
    }
}

//��һ����
void NumMulMatrix(Matrix *matrix_A, double a)
{
    int i;
    for (i = 0; i < matrix_A->row * matrix_A->column; i++)
    {
        matrix_A->data[i] = matrix_A->data[i] * a;
    }
}

//��һ����
void NumAddMatrix(Matrix *matrix_A, double a)
{
    int i;
    for (i = 0; i < matrix_A->row * matrix_A->column; i++)
    {
        matrix_A->data[i] = matrix_A->data[i] + a;
    }
}

//��һ����
void NumCutMatrix(Matrix *matrix_A, double a)
{
    int i;
    for (i = 0; i < matrix_A->row * matrix_A->column; i++)
    {
        matrix_A->data[i] = matrix_A->data[i] - a;
    }
}


//�˷�
Matrix* MulMatrix(Matrix *matrix_A,Matrix *matrix_B)
{
    //    printf("\n\n\n");
    //    PrintMatrix(matrix_A);
    //    printf("\n\n\n");
    //    PrintMatrix(matrix_B);
    
    int i,j,k;
    if (matrix_A->column == matrix_B->row)        //��==��
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
//        printf("�������\n");
        return NULL;
    }
}

//����ת��
void TransMatrix(Matrix *matrix)
{
    int i,j;
    if (matrix->column == matrix->row)
    {
        Matrix *matrixTemp = InitMatrix(matrixTemp, matrix->row,matrix->column);           //����һ����ʱ����
        CopyMatrix(matrix,matrixTemp);    //��Ŀ������data���Ƹ���ʱ����
        
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
        Matrix *matrixTemp = InitMatrix(matrixTemp, matrix->row,matrix->column);           //����һ����ʱ����
        CopyMatrix(matrix,matrixTemp);    //��Ŀ������data���Ƹ���ʱ����
        
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

//����ȫ��1�ľ���
void OnesMatrix(Matrix *matrix)
{
    int i;
    for (i = 0; i < matrix->column * matrix->row; i++)
    {
        matrix->data[i] = 1;
    }
}

//����ȫ��0�ľ���
void ZerosMatrix(Matrix *matrix)
{
    int i;
    for (i = 0; i < matrix->column * matrix->row; i++)
    {
        matrix->data[i] = 0;
    }
}

//����ȫ��x�ľ���
void XMatrix(Matrix *matrix, double x)
{
    int i;
    for (i = 0; i < matrix->column * matrix->row; i++)
    {
        matrix->data[i] = x;
    }
}


//���ɵ�λ����
void IdentityMatrix(Matrix *matrix)
{
    for (int i = 0; i < matrix->row; i++) {
        for (int j = 0; j < matrix->column; j++) {
            ValueOneMatrix(matrix, ((i == j) ? (double)1 : 0), i, j);
        }
    }
}



//------------------------------------------------------------------
//����: ���ò�����Ԫ�ĸ�˹��ȥ������A�������B
//��ڲ���: ���뷽��
//����ֵ:
//------------------------------------------------------------------- 


Matrix* Inv(Matrix* A)
{
    int i, j, k;
    double max, temp;
    
    if (A->column != A->row) {
        return NULL;
    }
    
    int n = A->row;   //�������
    Matrix *B = InitMatrix(B, A->row, A->column);      //�������
    Matrix *t = InitMatrix(t, A->row, A->column);                //��ʱ����
    //��A����������ʱ������
    
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            ValueOneMatrix(t, GetValue(A, i, j), i, j);
        }
    }
    
    //��ʼ��B����Ϊ��λ��
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            ValueOneMatrix(B, ((i == j) ? (double)1 : 0), i, j);
        }
    }
    
    for (i = 0; i < n; i++)
    {
        //Ѱ����Ԫ
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
        //�����Ԫ�����в��ǵ�i�У������н���
        if (k != i)
        {
            for (j = 0; j < n; j++)
            {
                temp = GetValue(t, i, j);
                ValueOneMatrix(t, GetValue(t, k, j), i, j);
                ValueOneMatrix(t, temp, k, j);

                //B���潻��
                temp = GetValue(B, i, j);
                ValueOneMatrix(B, GetValue(B, k, j), i, j);
                ValueOneMatrix(B, temp, k, j);

            }
        }
        //�ж���Ԫ�Ƿ�Ϊ0, ����, �����A�������Ⱦ���,�����������
        if (GetValue(t, i, i) == 0)
        {
//            printf("There is no inverse matrix!");
            return 0;
        }
        
        //��ȥA�ĵ�i�г�ȥi������ĸ���Ԫ��
        temp = GetValue(t, i, i);
        for (j = 0; j < n; j++)
        {
            ValueOneMatrix(t, GetValue(t, i, j)/temp, i, j);   //���Խ����ϵ�Ԫ�ر�Ϊ1
            ValueOneMatrix(B, GetValue(B, i, j)/temp, i, j);   //�������

        }
        for (j = 0; j < n; j++)        //��0��->��n��
        {
            if (j != i)                //���ǵ�i��
            {
                temp = GetValue(t, j, i);
                for (k = 0; k < n; k++)        //��j��Ԫ�� - i��Ԫ��*j��i��Ԫ��
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


//��������������
Matrix* cross(Matrix *A, Matrix *B)
{
    if((A->row == 3 && A->column  == 1)  && (B->row == 3 && B->column  == 1))
    {

        Matrix *C = InitMatrix(C, 3, 1);       //�������
        ValueOneMatrix(C, A->data[1]*B->data[2] - A->data[2]*B->data[1], 0, 0);
        ValueOneMatrix(C, A->data[2]*B->data[0] - A->data[0]*B->data[2], 1, 0);
        ValueOneMatrix(C, A->data[0]*B->data[1] - A->data[1]*B->data[0], 2, 0);
        return C;
    }else if ( (A->row == 1 && A->column  == 3) && (B->row == 1 && B->column  == 3) || ((A->row == 1 && A->column  == 3)  && (B->row == 3 && B->column  == 1)) || ((A->row == 3 && A->column  == 1)  && (B->row == 1 && B->column  == 3)) )
    {
        
        Matrix *C = InitMatrix(C, 1, 3);       //�������
        ValueOneMatrix(C, A->data[1]*B->data[2] - A->data[2]*B->data[1], 0, 0);
        ValueOneMatrix(C, A->data[2]*B->data[0] - A->data[0]*B->data[2], 0, 1);
        ValueOneMatrix(C, A->data[0]*B->data[1] - A->data[1]*B->data[0], 0, 2);
        return C;
    }
//    printf("���󲻺Ϸ�\n");
    return NULL;
}

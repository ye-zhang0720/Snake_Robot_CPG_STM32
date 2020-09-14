#include "Hopf_CPG.h"
#include "Matrix.h"
#include "memory_manage.h"	 

Matrix* Hopf(double u, double v, float steps, float lambda, float omega, float sigma, float rho, Matrix* CouplingPar)
{
    
    double r = u*u + v*v;
    rho = rho*rho;
    double temp = r/rho - sigma;
    
    double du = -omega*v - lambda*temp*u + GetValue(CouplingPar, 0, 0);
    double dv = omega*u - lambda*temp*v + GetValue(CouplingPar, 0, 1);
    
    Matrix *result = InitMatrix(result,1,2);
    ValueOneMatrix(result, du*steps+u, 0, 0);
    ValueOneMatrix(result, dv*steps+v, 0, 1);
    return result;
}

Matrix* Coupling(double g, Matrix *W, Matrix *Rho, Matrix *Phi, Matrix *X, int i)
{
    //g 全局耦合系数
    //W 振荡器j到i的链接权值
    //Rho 第i个和第j个振荡器稳态时的震荡半径
    //Phi 第j个振荡器到第i个振荡器的相位偏移差
    //X     %当前状态
    //i 当前节点编号
    Matrix *Xi = InitMatrix(Xi,2,1);
    Matrix *Xj = InitMatrix(Xj,2,1);   //初始化xj
    Matrix *R = InitMatrix(R,2,2);     //初始化R
		Matrix *tmp2, *tmp3;
		//定义返回值
    Matrix *rel = InitMatrix(rel,2,1);   //初始化xj
	
    int a = X->row;
    
    double r[4] = {0};
    //读取xi的值
    //temp[0] = GetValue(X, i, 0);
    //temp[1] = GetValue(X, i, 1);
    //ValueMatrix(Xi,temp);    //给Xi赋值
    
		ValueOneMatrix(Xi,GetValue(X, i, 0),0,0);
		ValueOneMatrix(Xi,GetValue(X, i, 1),1,0);

    

    

    for(int j = 0; j < a; j++){
        r[0] = cos(GetValue(Phi, i, j));
        r[1] = 0-sin(GetValue(Phi, i, j));
        r[2] = 0-r[1];
        r[3] = r[0];
        
        //读取xj的值
        //temp[0] = GetValue(X, j, 0);
        //temp[1] = GetValue(X, j, 1);
        
			ValueOneMatrix(R,r[0],0,0);
			ValueOneMatrix(R,r[1],0,1);
			ValueOneMatrix(R,r[2],1,0);
			ValueOneMatrix(R,r[3],1,1);
			
			ValueOneMatrix(Xj,GetValue(X, j, 0),0,0);
			ValueOneMatrix(Xj,GetValue(X, j, 1),1,0);
        //ValueMatrix(R,r);    //给R赋值
        //ValueMatrix(Xj,temp);    //给Xj赋值
			
        tmp2 = MulMatrix(R,Xj);
        NumMulMatrix(tmp2, GetValue(Rho,0,i) / GetValue(Rho,0,j));
        tmp3 = CutMatrix(tmp2, Xi);
				FreeMatrix(tmp2);
        NumMulMatrix(tmp3, GetValue(W,i,j));
        tmp2 = AddMatrix(rel, tmp3);
				FreeMatrix(tmp3);
				FreeMatrix(rel);
				CopyMatrix(tmp2, rel);
				FreeMatrix(tmp2);
    }
    NumMulMatrix(rel, g);
    TransMatrix(rel);
		
		//释放内存
		FreeMatrix(Xi);
		FreeMatrix(Xj);
		FreeMatrix(R);
		FreeMatrix(tmp2);

    return rel;
}


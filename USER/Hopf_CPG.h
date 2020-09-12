#ifndef __HOPF_CPG_H
#define __HOPF_CPG_H

#include "Matrix.h"
#include "memory_manage.h"	  

Matrix* Hopf(double u, double v, float steps, float lambda, float omega, float sigma, float rho, Matrix* CouplingPar);

Matrix* Coupling(double g, Matrix *W, Matrix *Rho, Matrix *Phi, Matrix *X, int i);


#endif

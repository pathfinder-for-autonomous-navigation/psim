/*
 * arrayProduct.c - example in MATLAB External Interfaces
 *
 * Multiplies an input scalar (multiplier)
 * times a 1xN matrix (inMatrix)
 * and outputs a 1xN matrix (outMatrix)
 *
 * The calling syntax is:
 *
 *		outMatrix = arrayProduct(multiplier, inMatrix)
 *
 * This is a MEX file for MATLAB.
*/

#include "mex.h"
#include "../../../src/inl/geomag.hpp"

void calc_geomag(double dyear, double *x, double *b)
{
  geomag::Vector in;
  geomag::Vector out;
  in.x= (float)x[0];
  in.y= (float)x[1];
  in.z= (float)x[2];
  out= geomag::GeoMag(dyear,in, geomag::WMM2020);
  b[0]=out.x;
  b[1]=out.y;
  b[2]=out.z;
}

/* The gateway function */
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
double dyear;      /* input scalar */
double *inMatrix;       /* 3x1 input matrix */
double *outMatrix;      /* 3x1 output matrix */
/* get the value of the scalar input  */
dyear = mxGetScalar(prhs[0]);
/* create a pointer to the real data in the input matrix  */
inMatrix = mxGetDoubles(prhs[1]);

/* create the output matrix */
plhs[0] = mxCreateDoubleMatrix(3,1,mxREAL);

/* get a pointer to the real data in the output matrix */
outMatrix = mxGetDoubles(plhs[0]);

/* call the computational routine */
calc_geomag(dyear, inMatrix, outMatrix);

}

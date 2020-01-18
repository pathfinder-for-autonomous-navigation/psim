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
#include "../../../geograv/include/geograv.hpp"
#include "../../../geograv/include/EGM96.hpp"

constexpr geograv::Coeff<30> EGM96_trunc(EGM96);

void calc_geograv(double *x, double *out)
{
  geograv::Vector in;
  geograv::Vector g;
  in.x= x[0];
  in.y= x[1];
  in.z= x[2];
  double pot=geograv::GeoGrav(in, g,EGM96_trunc,true);
  out[3]=pot;
  out[0]=g.x;
  out[1]=g.y;
  out[2]=g.z;
}

/* The gateway function */
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[])
{
double *inMatrix;       /* 3x1 input matrix */
double *outMatrix;      /* 4x1 output matrix */

/* create a pointer to the real data in the input matrix  */
inMatrix = mxGetDoubles(prhs[0]);

/* create the output matrix */
plhs[0] = mxCreateDoubleMatrix(4,1,mxREAL);

/* get a pointer to the real data in the output matrix */
outMatrix = mxGetDoubles(plhs[0]);


/* call the computational routine */
calc_geograv(inMatrix, outMatrix);

}

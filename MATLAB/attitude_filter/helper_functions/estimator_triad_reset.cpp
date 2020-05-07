#include <mex.h>
#include <gnc/attitude_estimator.hpp>
#include <lin.hpp>

void extract_and_update(double *x, double *out)
{
  lin
  geograv::Vector g;
  in.x= x[0];
  in.y= x[1];
  in.z= x[2];
  double pot=geograv::GeoGrav(in, g,GGM05S_trunc,true);
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

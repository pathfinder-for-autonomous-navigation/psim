
#include "mex.h"

#include <gnc/attitude_estimator.hpp>

#include <lin/core.hpp>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, mxArray const *prhs[]) {
  // Verify output arguments
  if (nlhs != 4) mexErrMsgTxt("Expects four output arguemnts");

  // Verify inputs arguments
  if (nrhs != 8) mexErrMsgTxt("Expect seven input arguments");

  // Inputs
  struct {
    double t;
    lin::Vector4d q_body_eci;
    lin::Vector3d gyro_bias;
    lin::Matrix6<6, 6> P;
    double dt;
    lin::Vector3d r_ecef;
    lin::Vector3d s_body;
    lin::Vector3d b_body;
    lin::Vector3d w_body;
  } in;

  // Populate inputs

  // Take a UKF step
  gnc::AttitudeEstimator estimator;
  estimator.reset(in.t, in.q_body_eci, in.gyro_bias, in.P);
  estimator.update(in.t + in.dt, in.r_ecef, in.s_body, in.b_body, in.w_body);


}

//
// test/attitude_estimation_test/test.cpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#include "../test.hpp"

#include <gnc_attitude_estimation.hpp>
#include <cmath>

void test_nan_intputs() {
  gnc::AttitudeEstimatorState state;
  gnc::AttitudeEstimatorData data;
  gnc::AttitudeEstimate estimate;
  gnc::estimate_attitude(state, data, estimate);
  // Assert all NaN outputs
  TEST_ASSERT_TRUE(std::isnan(estimate.q_body_eci(0)));
  TEST_ASSERT_TRUE(std::isnan(estimate.w_body(0)));
}

void test() {
  RUN_TEST(test_nan_intputs);
}

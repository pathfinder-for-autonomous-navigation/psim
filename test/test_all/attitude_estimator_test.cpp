/** @file test_all/attitude_estimator.cpp
 *  @author Kyle Krol */

#include "test.hpp"
#include "attitude_estimator_test.hpp"

#include <gnc/attitude_estimator.hpp>

void test_attitude_estimator_nan_inputs() {
  gnc::AttitudeEstimatorState state;
  gnc::AttitudeEstimatorData data;
  gnc::AttitudeEstimate estimate;
  gnc::estimate_attitude(state, data, estimate);
  // Assert all NaN outputs
  TEST_ASSERT_TRUE(std::isnan(estimate.q_body_eci(0)));
  TEST_ASSERT_TRUE(std::isnan(estimate.w_body(0)));
}

void attitude_estimator_test() {
  RUN_TEST(test_attitude_estimator_nan_inputs);
}

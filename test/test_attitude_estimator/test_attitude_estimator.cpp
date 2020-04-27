
#include <gnc/attitude_estimator.hpp>

#include <lin/core.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>

#include <unity.h>
#undef isnan
#undef isinf

static void test_attitude_estimator_state_constructor() {
  gnc::AttitudeEstimatorState state;

  TEST_ASSERT_TRUE(lin::all(lin::isnan(state.q_body_eci)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(state.gyro_bias)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(state.P)));
  TEST_ASSERT_TRUE(lin::isnan(state.t));
  TEST_ASSERT_FALSE(state.is_valid);
}

static void test_attitude_estimator_data_constructor() {
  gnc::AttitudeEstimatorData data;

  TEST_ASSERT_TRUE(lin::all(lin::isnan(data.r_ecef)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(data.b_body)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(data.s_body)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(data.w_body)));
  TEST_ASSERT_TRUE(lin::isnan(data.t));
}

static void test_attitude_estimate_constructor() {
  gnc::AttitudeEstimate estimate;

  TEST_ASSERT_TRUE(lin::all(lin::isnan(estimate.q_body_eci)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(estimate.gyro_bias)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(estimate.P)));
  TEST_ASSERT_FALSE(estimate.is_valid);
}

static int test() {
  UNITY_BEGIN()
  RUN_TEST(test_attitude_estimator_state_constructor);
  RUN_TEST(test_attitude_estimator_data_constructor);
  RUN_TEST(test_attitude_estimate_constructor);
  return UNITY_END();
}

#ifdef DESKTOP
int main(int argc, char *argv[]) {
  return test();
}
#else
#include <Arduino.h>
void setup() {
  delay(10000);
  Serial.begin(9600);
  test();
}

void loop() {}
#endif

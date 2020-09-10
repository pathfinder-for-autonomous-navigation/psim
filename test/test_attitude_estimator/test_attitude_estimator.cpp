
#include <gnc/attitude_estimator.hpp>
#include <gnc/constants.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>

#include <unity.h>
#undef isnan
#undef isinf
#undef isfinite

#define TEST_ASSERT_LIN_NEAR_ABS(tol, e, a) \
    TEST_ASSERT_TRUE(lin::all(tol > lin::abs(e - a))); static_assert(true, "")
#define TEST_ASSERT_LIN_FRO_NEAR_REL(tol, e, a) \
    TEST_ASSERT_TRUE(tol > lin::abs(lin::fro(e) - lin::fro(a)) / lin::fro(e)); static_assert(true, "")

static void test_state_constructor() {
  gnc::AttitudeEstimatorState state;

  TEST_ASSERT_TRUE(lin::all(lin::isnan(state.x)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(state.x)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(state.P)));
  TEST_ASSERT_TRUE(lin::isnan(state.t));
  TEST_ASSERT_FALSE(state.is_valid);
}

static void test_data_constructor() {
  gnc::AttitudeEstimatorData data;

  TEST_ASSERT_TRUE(lin::all(lin::isnan(data.r_ecef)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(data.b_body)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(data.s_body)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(data.w_body)));
  TEST_ASSERT_TRUE(lin::isnan(data.t));
}

static void test_estimate_constructor() {
  gnc::AttitudeEstimate estimate;

  TEST_ASSERT_TRUE(lin::all(lin::isnan(estimate.q_body_eci)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(estimate.gyro_bias)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(estimate.P)));
  TEST_ASSERT_FALSE(estimate.is_valid);
}

static void test_simple_reset() {
  lin::internal::RandomsGenerator randoms;
  double t = randoms.next();
  lin::Vector4d q = lin::rands<lin::Vector4d>(4, 1, randoms);
  q = q / lin::norm(q);

  gnc::AttitudeEstimatorState state;

  gnc::attitude_estimator_reset(state, t, q);
  TEST_ASSERT_TRUE(state.is_valid);
  TEST_ASSERT_DOUBLE_WITHIN(1.0e-6, t, state.t);
  TEST_ASSERT_LIN_NEAR_ABS(1.0e-5f, q, state.q);

  gnc::attitude_estimator_reset(state, gnc::constant::nan, q);
  TEST_ASSERT_FALSE(state.is_valid);
}

static void test_triad_reset() {
  // TODO : Implement this
  // https://github.com/pathfinder-for-autonomous-navigation/psim/issues/187
}

static void test_update() {
  // TODO
  // https://github.com/pathfinder-for-autonomous-navigation/psim/issues/197
}

static int test() {
  UNITY_BEGIN();
  RUN_TEST(test_state_constructor);
  RUN_TEST(test_data_constructor);
  RUN_TEST(test_estimate_constructor);
  RUN_TEST(test_simple_reset);
  RUN_TEST(test_triad_reset);
  RUN_TEST(test_update);
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

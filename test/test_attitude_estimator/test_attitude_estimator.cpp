
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
  // Inputs tested with the MATLAB filter
  double dt = 0.1;
  lin::Vector3f w_body { -0.024014867906579, 0.027691160749801, 0.014046367732340 };
  lin::Vector3f b_body {  0.000001674731376, 0.000018282146914, 0.000022276563655 };
  lin::Vector3f r_ecef { 1.0e6 * -6.690748340286341, 1.0e6 * -0.708202263245960, 1.0e6 * 0.012794219705281 };
  lin::Vector4f q_init { 0.214170254370675, 0.051976879477451, 0.033112924164660, 0.974850265628446 };
  float s_phi = -0.653906823853647, s_theta = 1.520169057482305;
  lin::Vector3f s_body { std::sin(s_theta) * std::cos(s_phi), std::sin(s_theta) * std::sin(s_phi), std::cos(s_theta) };

  // Initialize and ensure that state is valid
  gnc::AttitudeEstimatorState state;
  gnc::attitude_estimator_reset(state, 0.0, q_init);
  TEST_ASSERT_TRUE(state.is_valid);

  // Set sensor readings
  gnc::AttitudeEstimatorData data;
  data.t = dt;
  data.r_ecef = r_ecef;
  data.b_body = b_body;
  data.s_body = s_body;
  data.w_body = w_body;

  gnc::AttitudeEstimate estimate;
  gnc::attitude_estimator_update(state, data, estimate);

  // Expected output from the MATLAB filter
  lin::Vector4f q_out { 0.265929905107455, 0.014020477923108, .017499187636831, 0.963731544674345 };
  lin::Vector3f gyr_bias_out { -0.001772074342230, 0.001526741703701, 0.000031638711634 };
  lin::Matrixf<6, 6> P_out {
      0.000763825058840, -0.000124845790786, -0.000019683986452,  0.000006386112004, -0.000000679193028,  0.000003146367224,
    -0.000124845790786,  0.000813516433867,  0.000737086978426, -0.000000438648340,  0.000003579035280, -0.000019614885086,
    -0.000019683986452,  0.000737086978426,  0.001354560190315, -0.000002776949153, -0.000014384374830, -0.000015421716731,
      0.000006386112004, -0.000000438648340, -0.000002776949153,  0.004891715532545,  0.000000043138377,  0.000000007334188,
    -0.000000679193028,  0.000003579035280, -0.000014384374830,  0.000000043138377,  0.004891764283089,  0.000000394646413,
      0.000003146367224, -0.000019614885086, -0.000015421716731,  0.000000007334188,  0.000000394646413,  0.004892241707372
  };

  TEST_ASSERT_TRUE(estimate.is_valid);
  TEST_ASSERT_TRUE(state.is_valid);
  TEST_ASSERT_LIN_FRO_NEAR_REL(1.0e-3f, q_out, estimate.q_body_eci);
  TEST_ASSERT_LIN_FRO_NEAR_REL(1.0e-3f, gyr_bias_out, estimate.gyro_bias);
  TEST_ASSERT_LIN_FRO_NEAR_REL(1.0e-2f, P_out, estimate.P); // Makes sense this would be the least accurate
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

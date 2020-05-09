
#include <gnc/constants.hpp>
#include <gnc/orbit_controller.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>

#include <unity.h>
#undef isnan
#undef isinf
#undef isfinite

static void test_state_constructor() {
  gnc::OrbitControllerState state;

  TEST_ASSERT_TRUE(lin::isnan(state.t_last_firing));
}

static void test_data_constructor() {
  gnc::OrbitControllerData data;

  TEST_ASSERT_TRUE(lin::isnan(data.t));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(data.r_ecef)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(data.v_ecef)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(data.dr_ecef)));
  TEST_ASSERT_TRUE(lin::all(lin::isnan(data.dv_ecef)));
}

static void test_actuation_constructor() {
  gnc::OrbitActuation actuation;

  TEST_ASSERT_TRUE(lin::all(lin::isnan(actuation.J_eci)));
  TEST_ASSERT_TRUE(lin::isnan(actuation.phase_till_next_node));
}

static int test() {
  UNITY_BEGIN();
  RUN_TEST(test_state_constructor);
  RUN_TEST(test_data_constructor);
  RUN_TEST(test_actuation_constructor);
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

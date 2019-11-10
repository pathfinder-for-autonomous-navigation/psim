//
// test/test.hpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#ifdef PAN_PSIM_TEST_TEST_HPP_
#error test.hpp should not be included twice
#endif
#define PAN_PSIM_TEST_TEST_HPP_

#define UNITY_EXCLUDE_MATH_H
#include <unity.h>

#define TEST_ASSERT_DOUBLE_VEC_NEAR(t, u, v) \
    TEST_ASSERT_DOUBLE_WITHIN(t, 0.0, lin::fro(u - v))

#define TEST_ASSERT_FLOAT_VEC_NEAR(t, u, v) \
    TEST_ASSERT_DOUBLE_WITHIN(t, 0.0, lin::fro(u - v))

void test();

int _test() {
  UNITY_BEGIN();
  test();
  return UNITY_END();
}

#ifdef TEST_DESKTOP
int main() {
  return _test();
}
#else
#include <Arduino.h>
void setup() {
  delay(2000);
  Serial.begin(9600);
  _test();
}
void loop() {
  delay(1000);
  Serial.println("Unit tests complete");
}
#endif

/** @file test_all/test.cpp
 *  @author Kyle Krol */

#include "test.hpp"

#include "attitude_estimator_test.hpp"
#include "containers_test.hpp"
#include "environment_test.hpp"
#include "ode_test.hpp"
#include "utilities_test.hpp"

int test() {
  UNITY_BEGIN();
  attitude_estimator_test();
  containers_test();
  environment_test();
  ode_test();
  utilities_test();
  return UNITY_END();
}

#ifdef TEST_DESKTOP
int main() { return test(); }
#else
#include <Arduino.h>
void setup() {
  delay(2000);
  Serial.begin(9600);
  test();
}
void loop() { }
#endif

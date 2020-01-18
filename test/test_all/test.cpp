//
// test/test_all/test.cpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#include "test.hpp"

#include "attitude_estimation_test.hpp"
#include "containers_test.hpp"
#include "environment_test.hpp"
#include "ode_test.hpp"
#include "utilities_test.hpp"

int test() {
  UNITY_BEGIN();
  attitude_estimation_test();
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

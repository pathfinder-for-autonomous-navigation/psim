//
// test/utilities_test.cpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#include <gnc_ode.hpp>

#include <gtest/gtest.h>

#define PAN_GNC_ODE_TEST_VARS(n) \
    unsigned int const nt = 4; \
    unsigned int const ne = 2; \
    double const t[nt] = { 0.0, 0.1, 0.2, 0.3 }; \
    double bf[n * ne]; \
    double _y[nt * ne]; \
    double *y[nt]; \
    for (unsigned int i = 0; i < nt; i++) \
      y[i] = _y + ne * i; \
    y[0][0] = 1.0; \
    y[0][1] = 0.0;

static void fsho(double t, double const *y, double *dy) {
  dy[0] = y[1];
  dy[1] = -y[0];
}

TEST(OdeTest, Ode1ShoTest) {
  PAN_GNC_ODE_TEST_VARS(1)
  gnc::ode1(t, nt, y, ne, bf, fsho);
  // Check against MATLAB output
  ASSERT_FLOAT_EQ(y[1][0],  1.000000000000000);
  ASSERT_FLOAT_EQ(y[1][1], -0.100000000000000);
  ASSERT_FLOAT_EQ(y[2][0],  0.990000000000000);
  ASSERT_FLOAT_EQ(y[2][1], -0.200000000000000);
  ASSERT_FLOAT_EQ(y[3][0],  0.970000000000000);
  ASSERT_FLOAT_EQ(y[3][1], -0.299000000000000);
}

TEST(OdeTest, Ode2ShoTest) {
  PAN_GNC_ODE_TEST_VARS(3)
  gnc::ode2(t, nt, y, ne, bf, fsho);
  // Check against MATLAB output
  ASSERT_FLOAT_EQ(y[1][0],  0.995000000000000);
  ASSERT_FLOAT_EQ(y[1][1], -0.100000000000000);
  ASSERT_FLOAT_EQ(y[2][0],  0.980025000000000);
  ASSERT_FLOAT_EQ(y[2][1], -0.199000000000000);
  ASSERT_FLOAT_EQ(y[3][0],  0.955224875000000);
  ASSERT_FLOAT_EQ(y[3][1], -0.296007500000000);
}

TEST(OdeTest, Ode3ShoTest) {
  PAN_GNC_ODE_TEST_VARS(4)
  gnc::ode3(t, nt, y, ne, bf, fsho);
  // Check against MATLAB output
  ASSERT_FLOAT_EQ(y[1][0],  0.995000000000000);
  ASSERT_FLOAT_EQ(y[1][1], -0.099833333333333);
  ASSERT_FLOAT_EQ(y[2][0],  0.980058305555556);
  ASSERT_FLOAT_EQ(y[2][1], -0.198668333333333);
  ASSERT_FLOAT_EQ(y[3][0],  0.955324292083333);
  ASSERT_FLOAT_EQ(y[3][1], -0.295517479171296);
}

TEST(OdeTest, Ode4ShoTest) {
  PAN_GNC_ODE_TEST_VARS(5)
  gnc::ode4(t, nt, y, ne, bf, fsho);
  // Check against MATLAB output
  ASSERT_FLOAT_EQ(y[1][0],  0.995004166666667);
  ASSERT_FLOAT_EQ(y[1][1], -0.099833333333333);
  ASSERT_FLOAT_EQ(y[2][0],  0.980066597239583);
  ASSERT_FLOAT_EQ(y[2][1], -0.198669165277778);
  ASSERT_FLOAT_EQ(y[3][0],  0.955336542863976);
  ASSERT_FLOAT_EQ(y[3][1], -0.295519962530663);
  // ASSERT_FLOAT_EQ(y[3][1], -0.295520962530663); Fails as expected
}

//
// test/test_all/ode_test.cpp
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
#include "ode_test.hpp"

#include <gnc_ode.hpp>
#include <cmath>

#define PAN_GNC_ODEX_TEST_VARS(n) \
    unsigned int const nt = 4; \
    unsigned int const ne = 2; \
    double const t[nt] = { 0.0L, 0.1L, 0.2L, 0.3L }; \
    double bf[n * ne]; \
    double _y[nt * ne]; \
    double *y[nt]; \
    for (unsigned int i = 0; i < nt; i++) \
      y[i] = _y + ne * i; \
    y[0][0] = 1.0L; \
    y[0][1] = 0.0L;

#define PAN_GNC_ODEXX_TEST_VARS(n) \
    unsigned int const ne = 2; \
    double const ti = 0.0; \
    double const yi[ne] = { 1.0, 0.0 }; \
    double yf[ne]; \
    double bf[n * ne];

static void fsho(double t, double const *y, double *dy) {
  dy[0] = y[1];
  dy[1] = -y[0];
}

void test_ode_ode1_sho() {
  PAN_GNC_ODEX_TEST_VARS(1)
  gnc::ode1(t, nt, y, ne, bf, fsho);
  // Check against MATLAB output
  TEST_ASSERT_EQUAL_DOUBLE( 1.000000000000000, y[1][0]);
  TEST_ASSERT_EQUAL_DOUBLE(-0.100000000000000, y[1][1]);
  TEST_ASSERT_EQUAL_DOUBLE( 0.990000000000000, y[2][0]);
  TEST_ASSERT_EQUAL_DOUBLE(-0.200000000000000, y[2][1]);
  TEST_ASSERT_EQUAL_DOUBLE( 0.970000000000000, y[3][0]);
  TEST_ASSERT_EQUAL_DOUBLE(-0.299000000000000, y[3][1]);
}

void test_ode_ode2_sho() {
  PAN_GNC_ODEX_TEST_VARS(3)
  gnc::ode2(t, nt, y, ne, bf, fsho);
  // Check against MATLAB output
  TEST_ASSERT_EQUAL_DOUBLE( 0.995000000000000, y[1][0]);
  TEST_ASSERT_EQUAL_DOUBLE(-0.100000000000000, y[1][1]);
  TEST_ASSERT_EQUAL_DOUBLE( 0.980025000000000, y[2][0]);
  TEST_ASSERT_EQUAL_DOUBLE(-0.199000000000000, y[2][1]);
  TEST_ASSERT_EQUAL_DOUBLE( 0.955224875000000, y[3][0]);
  TEST_ASSERT_EQUAL_DOUBLE(-0.296007500000000, y[3][1]);
}

void test_ode_ode3_sho() {
  PAN_GNC_ODEX_TEST_VARS(4)
  gnc::ode3(t, nt, y, ne, bf, fsho);
  // Check against MATLAB output
  TEST_ASSERT_EQUAL_DOUBLE( 0.995000000000000, y[1][0]);
  TEST_ASSERT_EQUAL_DOUBLE(-0.099833333333333, y[1][1]);
  TEST_ASSERT_EQUAL_DOUBLE( 0.980058305555556, y[2][0]);
  TEST_ASSERT_EQUAL_DOUBLE(-0.198668333333333, y[2][1]);
  TEST_ASSERT_EQUAL_DOUBLE( 0.955324292083333, y[3][0]);
  TEST_ASSERT_EQUAL_DOUBLE(-0.295517479171296, y[3][1]);
}

void test_ode_ode4_sho() {
  PAN_GNC_ODEX_TEST_VARS(5)
  gnc::ode4(t, nt, y, ne, bf, fsho);
  // Check against MATLAB output
  TEST_ASSERT_EQUAL_DOUBLE( 0.995004166666667, y[1][0]);
  TEST_ASSERT_EQUAL_DOUBLE(-0.099833333333333, y[1][1]);
  TEST_ASSERT_EQUAL_DOUBLE( 0.980066597239583, y[2][0]);
  TEST_ASSERT_EQUAL_DOUBLE(-0.198669165277778, y[2][1]);
  TEST_ASSERT_EQUAL_DOUBLE( 0.955336542863976, y[3][0]);
  TEST_ASSERT_EQUAL_DOUBLE(-0.295519962530663, y[3][1]);
}

void test_ode_ode23_sho() {
  PAN_GNC_ODEXX_TEST_VARS(6);
  int code;
  // Check against cos(1.0) and assert the error code is OK
  code = gnc::ode23(ti, 1.0, yi, yf, ne, bf, 1e-4, 1e-6, 1e-6, 1000, fsho);
  TEST_ASSERT_EQUAL_INT(gnc::ODE_ERR_OK, code);
  TEST_ASSERT_DOUBLE_WITHIN(1e-3, std::cos(1.0), yf[0]);
  // Check against cos(5.0) and assert the error code is OK
  code = gnc::ode23(ti, 5.0, yi, yf, ne, bf, 1e-4, 1e-6, 1e-6, 1000, fsho);
  TEST_ASSERT_EQUAL_INT(gnc::ODE_ERR_OK, code);
  TEST_ASSERT_DOUBLE_WITHIN(1e-3, std::cos(5.0), yf[0]);
  // Check against cos(6.5) and assert the error code is OK
  code = gnc::ode23(ti, 6.5, yi, yf, ne, bf, 1e-4, 1e-6, 1e-6, 1000, fsho);
  TEST_ASSERT_EQUAL_INT(gnc::ODE_ERR_OK, code);
  TEST_ASSERT_DOUBLE_WITHIN(1e-3, std::cos(6.5), yf[0]);
  // Check against cos(7.5) and assert the error code is OK
  code = gnc::ode23(ti, 7.5, yi, yf, ne, bf, 1e-4, 1e-6, 1e-6, 2000, fsho);
  TEST_ASSERT_EQUAL_INT(gnc::ODE_ERR_OK, code);
  TEST_ASSERT_DOUBLE_WITHIN(2e-3, std::cos(7.5), yf[0]);
}

void test_ode_ode45_sho() {
  PAN_GNC_ODEXX_TEST_VARS(9);
  int code;
  // Check against cos(1.0) and assert the error code is OK
  code = gnc::ode45(ti, 1.0, yi, yf, ne, bf, 1e-4, 1e-6, 1e-6, 1000, fsho);
  TEST_ASSERT_EQUAL_INT(gnc::ODE_ERR_OK, code);
  TEST_ASSERT_DOUBLE_WITHIN(1e-3, std::cos(1.0), yf[0]);
  // Check against cos(5.0) and assert the error code is OK
  code = gnc::ode45(ti, 5.0, yi, yf, ne, bf, 1e-4, 1e-6, 1e-6, 1000, fsho);
  TEST_ASSERT_EQUAL_INT(gnc::ODE_ERR_OK, code);
  TEST_ASSERT_DOUBLE_WITHIN(1e-3, std::cos(5.0), yf[0]);
  // Check against cos(6.5) and assert the error code is OK
  code = gnc::ode45(ti, 6.5, yi, yf, ne, bf, 1e-4, 1e-6, 1e-6, 1000, fsho);
  TEST_ASSERT_EQUAL_INT(gnc::ODE_ERR_OK, code);
  TEST_ASSERT_DOUBLE_WITHIN(1e-3, std::cos(6.5), yf[0]);
  // Check against cos(7.5) and assert the error code is OK
  code = gnc::ode45(ti, 7.5, yi, yf, ne, bf, 1e-4, 1e-6, 1e-6, 2000, fsho);
  TEST_ASSERT_EQUAL_INT(gnc::ODE_ERR_OK, code);
  TEST_ASSERT_DOUBLE_WITHIN(2e-3, std::cos(7.5), yf[0]);
}

void ode_test() {
  RUN_TEST(test_ode_ode1_sho);
  RUN_TEST(test_ode_ode2_sho);
  RUN_TEST(test_ode_ode3_sho);
  RUN_TEST(test_ode_ode4_sho);
  RUN_TEST(test_ode_ode23_sho);
  RUN_TEST(test_ode_ode45_sho);
}

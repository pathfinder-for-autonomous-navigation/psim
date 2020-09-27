/** @file test_all/ode_test.cpp
 *  @author Kyle Krol */

#include "test.hpp"
#include "ode_test.hpp"

#include <gnc/ode.hpp>
#include <gnc/ode1.hpp>
#include <gnc/ode2.hpp>
#include <gnc/ode3.hpp>
#include <gnc/ode4.hpp>

#include <cmath>

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
  // Values were comapred to MATLAB's ode2 implementation
  auto const dx = [](double t, lin::Vector2d const &x, void *) -> lin::Vector2d {
    return {x(1), -x(0)};
  };
  gnc::Ode1<double, 2> ode;
  lin::Vector2d x = {1.0, 0.0};
  x = ode(0.0, 0.1, x, nullptr, dx);
  TEST_ASSERT_EQUAL_DOUBLE( 1.000000000000000, x(0));
  TEST_ASSERT_EQUAL_DOUBLE(-0.100000000000000, x(1));
  x = ode(0.1, 0.1, x, nullptr, dx);
  TEST_ASSERT_EQUAL_DOUBLE( 0.990000000000000, x(0));
  TEST_ASSERT_EQUAL_DOUBLE(-0.200000000000000, x(1));
  x = ode(0.2, 0.1, x, nullptr, dx);
  TEST_ASSERT_EQUAL_DOUBLE( 0.970000000000000, x(0));
  TEST_ASSERT_EQUAL_DOUBLE(-0.299000000000000, x(1));
}

void test_ode_ode2_sho() {
  // Values were comapred to MATLAB's ode2 implementation
  auto const dx = [](double t, lin::Vector2d const &x, void *) -> lin::Vector2d {
    return {x(1), -x(0)};
  };
  gnc::Ode2<double, 2> ode;
  lin::Vector2d x = {1.0, 0.0};
  x = ode(0.0, 0.1, x, nullptr, dx);
  TEST_ASSERT_EQUAL_DOUBLE( 0.995000000000000, x(0));
  TEST_ASSERT_EQUAL_DOUBLE(-0.100000000000000, x(1));
  x = ode(0.1, 0.1, x, nullptr, dx);
  TEST_ASSERT_EQUAL_DOUBLE( 0.980025000000000, x(0));
  TEST_ASSERT_EQUAL_DOUBLE(-0.199000000000000, x(1));
  x = ode(0.2, 0.1, x, nullptr, dx);
  TEST_ASSERT_EQUAL_DOUBLE( 0.955224875000000, x(0));
  TEST_ASSERT_EQUAL_DOUBLE(-0.296007500000000, x(1));
}

void test_ode_ode3_sho() {
  // Values were comapred to MATLAB's ode2 implementation
  auto const dx = [](double t, lin::Vector2d const &x, void *) -> lin::Vector2d {
    return {x(1), -x(0)};
  };
  gnc::Ode3<double, 2> ode;
  lin::Vector2d x = {1.0, 0.0};
  x = ode(0.0, 0.1, x, nullptr, dx);
  TEST_ASSERT_EQUAL_DOUBLE( 0.995000000000000, x(0));
  TEST_ASSERT_EQUAL_DOUBLE(-0.099833333333333, x(1));
  x = ode(0.1, 0.1, x, nullptr, dx);
  TEST_ASSERT_EQUAL_DOUBLE( 0.980058305555556, x(0));
  TEST_ASSERT_EQUAL_DOUBLE(-0.198668333333333, x(1));
  x = ode(0.2, 0.1, x, nullptr, dx);
  TEST_ASSERT_EQUAL_DOUBLE( 0.955324292083333, x(0));
  TEST_ASSERT_EQUAL_DOUBLE(-0.295517479171296, x(1));
}

void test_ode_ode4_sho() {
  // Values were comapred to MATLAB's ode2 implementation
  auto const dx = [](double t, lin::Vector2d const &x, void *) -> lin::Vector2d {
    return {x(1), -x(0)};
  };
  gnc::Ode4<double, 2> ode;
  lin::Vector2d x = {1.0, 0.0};
  x = ode(0.0, 0.1, x, nullptr, dx);
  TEST_ASSERT_EQUAL_DOUBLE( 0.995004166666667, x(0));
  TEST_ASSERT_EQUAL_DOUBLE(-0.099833333333333, x(1));
  x = ode(0.1, 0.1, x, nullptr, dx);
  TEST_ASSERT_EQUAL_DOUBLE( 0.980066597239583, x(0));
  TEST_ASSERT_EQUAL_DOUBLE(-0.198669165277778, x(1));
  x = ode(0.2, 0.1, x, nullptr, dx);
  TEST_ASSERT_EQUAL_DOUBLE( 0.955336542863976, x(0));
  TEST_ASSERT_EQUAL_DOUBLE(-0.295519962530663, x(1));
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

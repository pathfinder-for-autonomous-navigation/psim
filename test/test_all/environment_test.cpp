/** @file test_all/environmental_test.cpp
 *  @author Kyle Krol */

#include "test.hpp"
#include "environment_test.hpp"

#include <gnc/constants.hpp>
#include <gnc/environment.hpp>
#include <lin.hpp>

void test_environment_earth_attitude() {
  lin::Vector3d w;
  lin::Vector4d q;
  // Calculate and check for t=0.0 (comparing against MATLAB)
  gnc::env::earth_attitude(0.0, q);
  gnc::env::earth_angular_rate(0.0, w);
  TEST_ASSERT_DOUBLE_VEC_NEAR(1e-7, q, lin::Vector4d({
    0.000917219458782,
    0.000038843277811,
    0.998612074831165,
    0.052660053181324
  }));
  TEST_ASSERT_DOUBLE_VEC_NEAR(1e-10, w, lin::Vector3d({
    0.0,
    0.0,
    0.729211585530000e-4
  }));
  // Calculate and check for t=100000.0 (comparing against MATLAB)
  gnc::env::earth_attitude(100000.0, q);
  gnc::env::earth_angular_rate(100000.0, w);
  TEST_ASSERT_DOUBLE_VEC_NEAR(1e-7, q, lin::Vector4d({
    -0.000822043660271,
     0.000409431230373,
    -0.899670695955541,
     0.436568202517272
  }));
  TEST_ASSERT_DOUBLE_VEC_NEAR(1e-10, w, lin::Vector3d({
    0.0,
    0.0,
    0.729211585530000e-4
  }));
  // Calculate and check for t=200000.0 (comparing against MATLAB)
  gnc::env::earth_attitude(200000.0, q);
  gnc::env::earth_angular_rate(200000.0, w);
  TEST_ASSERT_DOUBLE_VEC_NEAR(1e-7, q, lin::Vector4d({
     0.000521964887389,
    -0.000755997323587,
     0.576590854598569,
    -0.817032522249877
  }));
  TEST_ASSERT_DOUBLE_VEC_NEAR(1e-10, w, lin::Vector3d({
    0.0,
    0.0,
    0.729211585530000e-4
  }));
}

void test_environment_sun_vector() {
  lin::Vector3f s;
  // Calculate and check for t=0.0 (comparing against MATLAB)
  gnc::env::sun_vector(0.0L, s);
  TEST_ASSERT_FLOAT_VEC_NEAR(1e-5, s, lin::Vector3f({
    -0.799739280399221,
     0.463585679812799,
     0.381451701869374
  }));
  // Calculate and check for t=100000.0 (comparing against MATLAB)
  gnc::env::sun_vector(100000.0L, s);
  TEST_ASSERT_FLOAT_VEC_NEAR(1e-5, s, lin::Vector3f({
    -0.808776458089373,
     0.445760043285879,
     0.383638664175262
  }));
  // Calculate and check for t=200000.0 (comparing against MATLAB)
  gnc::env::sun_vector(200000.0L, s);
  TEST_ASSERT_FLOAT_VEC_NEAR(1e-5, s, lin::Vector3f({
    -0.817481265280114,
     0.427765782728485,
     0.385669309179135
  }));
}

void test_environment_magnetic_field() {
  lin::Vector3f b, r;
  // Calculate and check for t=0.0 (comparing against MATLAB)
  r = { 5.430929361985142e6f, 2.949986114465339e6f, 2.949986114465338e6f };
  gnc::env::magnetic_field(0.0, r, b);
  TEST_ASSERT_FLOAT_VEC_NEAR(1e-9f, b, lin::Vector3f({
    -0.258627060247818e-4f,
    -0.123760992210009e-4f,
     0.152310085468343e-4f
  }));
  // Calculate and check for t=100000.0 (comparing against MATLAB)
  r = { 2.949986114465339e6f, 5.430929361985142e6f, 2.949986114465338e6f };
  gnc::env::magnetic_field(100000.0, r, b);
  TEST_ASSERT_FLOAT_VEC_NEAR(1e-9f, b, lin::Vector3f({
    -0.159026585606625e-4f,
    -0.279433261312079e-4f,
     0.147381606439012e-4f
  }));
  // Calculate and check for t=200000.0 (comparing against MATLAB)
  r = { 2.949986114465338e6f, 2.949986114465339e6f, -5.430929361985142e6f };
  gnc::env::magnetic_field(200000.0, r, b);
  TEST_ASSERT_FLOAT_VEC_NEAR(1e-9f, b, lin::Vector3f({
     0.245297651417786e-4f,
     0.101511859611492e-4f,
    -0.167983453138731e-4f
  }));
}

void environment_test() {
  RUN_TEST(test_environment_earth_attitude);
  RUN_TEST(test_environment_sun_vector);
  RUN_TEST(test_environment_magnetic_field);
}

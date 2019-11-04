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

#include <gnc_constants.hpp>
#include <gnc_environment.hpp>
#include <lin.hpp>

#include <gtest/gtest.h>

// Tests are broken otherwise
static_assert(gnc::constant::init_gps_week_number == 2045,
    "Tests are only applicable if the GPS week number is 2045!");
static_assert(gnc::constant::init_gps_time_of_week == 0,
    "Tests are only applicable if the GPS time of the week is 0!");
static_assert(gnc::constant::init_gps_nanoseconds == 0,
    "Tests are only applicable if the GPS nanoseconds is 0!");

#define ASSERT_VEC_NEAR(tol, a, b) ASSERT_NEAR(lin::fro(a - b), 0.0, tol)

TEST(EnvironmentTest, EarthAttitudeTest) {
  lin::Vector3d w;
  lin::Vector4d q;
  // Calculate and check for t=0.0 (comparing against MATLAB)
  gnc::env::earth_attitude(0.0, q);
  gnc::env::earth_angular_rate(0.0, w);
  ASSERT_VEC_NEAR(1e-7, q, lin::Vector4d({
    0.000917219458782,
    0.000038843277811,
    0.998612074831165,
    0.052660053181324
  }));
  ASSERT_VEC_NEAR(1e-10, w, lin::Vector3d({
    0.0,
    0.0,
    0.729211585530000e-4
  }));
  // Calculate and check for t=100000.0 (comparing against MATLAB)
  gnc::env::earth_attitude(100000.0, q);
  gnc::env::earth_angular_rate(100000.0, w);
  ASSERT_VEC_NEAR(1e-7, q, lin::Vector4d({
    -0.000822043660271,
     0.000409431230373,
    -0.899670695955541,
     0.436568202517272
  }));
  ASSERT_VEC_NEAR(1e-10, w, lin::Vector3d({
    0.0,
    0.0,
    0.729211585530000e-4
  }));
  // Calculate and check for t=200000.0 (comparing against MATLAB)
  gnc::env::earth_attitude(200000.0, q);
  gnc::env::earth_angular_rate(200000.0, w);
  ASSERT_VEC_NEAR(1e-7, q, lin::Vector4d({
     0.000521964887389,
    -0.000755997323587,
     0.576590854598569,
    -0.817032522249877
  }));
  ASSERT_VEC_NEAR(1e-10, w, lin::Vector3d({
    0.0,
    0.0,
    0.729211585530000e-4
  }));
}

TEST(EnvironmentTest, SunVectorTest) {
  lin::Vector3d s;
  // Calculate and check for t=0.0 (comparing against MATLAB)
  gnc::env::sun_vector(0.0, s);
  ASSERT_VEC_NEAR(1e-7, s, lin::Vector3d({
     0.997341623364311,
    -0.066845783473014,
    -0.029005646638551
  }));
  // Calculate and check for t=100000.0 (comparing against MATLAB)
  gnc::env::sun_vector(100000.0, s);
  ASSERT_VEC_NEAR(1e-7, s, lin::Vector3d({
     0.998604978312986,
    -0.048435925195477,
    -0.021025185825087
  }));
  // Calculate and check for t=200000.0 (comparing against MATLAB)
  gnc::env::sun_vector(200000.0, s);
  ASSERT_VEC_NEAR(1e-7, s, lin::Vector3d({
     0.999464270282190,
    -0.030018263209437,
    -0.013041330575474,
  }));
}

TEST(EnvironmentTest, MagneticFieldTest) {
  lin::Vector3f b, r;
  // Calculate and check for t=0.0 (comparing against MATLAB)
  r = { 5.430929361985142e6f, 2.949986114465339e6f, 2.949986114465338e6f };
  gnc::env::magnetic_field(0.0, r, b);
  ASSERT_VEC_NEAR(1e-5f, b, lin::Vector3f({
    -0.258582567767007e-4f,
    -0.123709833133034e-4f,
     0.152313341459376e-4f
  }));
  // Calculate and check for t=100000.0 (comparing against MATLAB)
  r = { 2.949986114465339e6f, 5.430929361985142e6f, 2.949986114465338e6f };
  gnc::env::magnetic_field(100000.0, r, b);
  ASSERT_VEC_NEAR(1e-5f, b, lin::Vector3f({
    -0.159017272380879e-4f,
    -0.279396062978776e-4f,
     0.147361051858752e-4f
  }));
  // Calculate and check for t=200000.0 (comparing against MATLAB)
  r = { 2.949986114465338e6f, 2.949986114465339e6f, -5.430929361985142e6f };
  gnc::env::magnetic_field(200000.0, r, b);
  ASSERT_VEC_NEAR(1e-5f, b, lin::Vector3f({
     0.245254032051889e-4f,
     0.101543964774464e-4f,
    -0.167963662534021e-4f
  }));
}

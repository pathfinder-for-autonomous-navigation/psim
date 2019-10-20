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

#include <lin.hpp>
#include <gnc_utilities.hpp>

#include <gtest/gtest.h>

#define ASSERT_VEC_NEAR(a, b) ASSERT_NEAR(lin::fro(a - b), 0.0, 1e-4)

using namespace gnc;  // Only b/c this repo pretty much defines all of ::gnc

TEST(UtilitiesTest, QuaternionConjugateTest) {
  lin::Vector4f q = {2.0f, 1.0f, -2.5f, 1.0f};
  lin::Vector4f q_conj;
  utl::quat_conj(q, q_conj);
  ASSERT_FLOAT_EQ(q_conj(0), -2.0f);
  ASSERT_FLOAT_EQ(q_conj(1), -1.0f);
  ASSERT_FLOAT_EQ(q_conj(2), 2.5f);
  ASSERT_FLOAT_EQ(q_conj(3), 1.0f);
}

TEST(UtilitiesTest, QuaternionCrossMultiplicationTest) {
  lin::Vector4f q1 = {1.0f, 2.0f, 3.0f, 4.0f};
  lin::Vector4f q2 = {5.0f, 2.0f, 23.0f, -4.0f};
  lin::Vector4f q3;
  utl::quat_cross_mult(q1, q2, q3);
  // Expected answer was calculated in MATLAB
  lin::Vector4f ans = {-24.0f, 8.0f, 88.0f, -94.0f};
  ASSERT_VEC_NEAR(q3, ans);
}

TEST(UtilitiesTest, RotateFrameTest) {
  lin::Vector4f q1 = {1.0f, 2.0f, 3.0f, 4.0f};
  q1 = q1 / lin::norm(q1);
  lin::Vector3f v = {3.0f, 50.0f, -23.0f};
  utl::rotate_frame(q1, v);
  // Expected answer was calculated in MATLAB
  lin::Vector3f ans = { 54.73333f, -0.66667f, -6.46667f};
  ASSERT_VEC_NEAR(v, ans);
}

// TODO : Add testcases for the other switch statement cases in dcm_to_quat
TEST(UtilitiesTest, DCMToQuaternionCase3Test) {
  lin::Vector4f q;
  lin::Matrix3x3f Q = {
     0.1333333f, 0.9333333f, -0.3333333f,
    -0.6666667f, 0.3333333f,  0.6666667f,
     0.7333333f, 0.1333333f,  0.6666667f
  };
  utl::dcm_to_quat(Q, q);
  // Expected answer was calculated in MATLAB
  lin::Vector4f ans = {0.1825742f, 0.3651484f, 0.5477226f, 0.7302967f};
  ASSERT_VEC_NEAR(q, ans);
}

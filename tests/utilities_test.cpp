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

TEST(UtilitiesTest, QuaternionConjugateTest) {
  lin::Vector4f q = {2.0f, 1.0f, -2.5f, 1.0f};
  lin::Vector4f q_conj;
  gnc::utl::quat_conj(q, q_conj);
  ASSERT_FLOAT_EQ(q_conj(0), -2.0f);
  ASSERT_FLOAT_EQ(q_conj(1), -1.0f);
  ASSERT_FLOAT_EQ(q_conj(2), 2.5f);
  ASSERT_FLOAT_EQ(q_conj(3), 1.0f);
}

TEST(UtilitiesTest, QuaternionCrossMultiplicationTest) {
  lin::Vector4f q1 = {1.0f, 2.0f, 3.0f, 4.0f};
  lin::Vector4f q2 = {5.0f, 2.0f, 23.0f, -4.0f};
  lin::Vector4f q3;
  gnc::utl::quat_cross_mult(q1, q2, q3);
  // Expected answer was calculated in MATLAB
  lin::Vector4f ans = {-24.0f, 8.0f, 88.0f, -94.0f};
  ASSERT_VEC_NEAR(q3, ans);
}

TEST(UtilitiesTest, RotateFrameTest) {
  lin::Vector4f q1 = {1.0f, 2.0f, 3.0f, 4.0f};
  q1 = q1 / lin::norm(q1);
  lin::Vector3f v = {3.0f, 50.0f, -23.0f};
  gnc::utl::rotate_frame(q1, v);
  // Expected answer was calculated in MATLAB
  lin::Vector3f ans = { 54.73333f, -0.66667f, -6.46667f};
  ASSERT_VEC_NEAR(v, ans);
}

TEST(UtilitiesTest, DCMToQuaternionCase0Test) {
  lin::Vector4f q;
  lin::Matrix3x3f Q = {
    0.2449f,  0.8723f,  0.4233f,
    0.4809f, -0.4883f,  0.7282f,
    0.8419f,  0.0252f, -0.5390f
  };
  gnc::utl::dcm_to_quat(Q, q);
  // Expected answer was calculated in MATLAB
  lin::Vector4f ans = { 0.7537f, 0.4488f, 0.4197f, 0.2332f };
  ASSERT_VEC_NEAR(q, ans);
}

TEST(UtilitiesTest, DCMToQuaternionCase1Test) {
  lin::Vector4f q;
  lin::Matrix3x3f Q = {
    0.0283f,  0.9952f, -0.0933f,
    0.6724f,  0.0501f,  0.7385f,
    0.7396f, -0.0836f, -0.6678f
  };
  gnc::utl::dcm_to_quat(Q, q);
  // Expected answer was calculated in MATLAB
  lin::Vector4f ans = { 0.6415f, 0.6499f, 0.2519f, 0.3204f };
  ASSERT_VEC_NEAR(q, ans);
}

TEST(UtilitiesTest, DCMToQuaternionCase2Test) {
  lin::Vector4f q;
  lin::Matrix3x3f Q = {
    0.4773f,  0.6765f, -0.5608f,
    0.8208f, -0.5712f,  0.0096f,
    0.3139f,  0.4649f,  0.8279f
  };
  gnc::utl::dcm_to_quat(Q, q);
  // Expected answer was calculated in MATLAB
  lin::Vector4f ans = { -0.1235f, 0.2372f, 0.9609f, -0.0721f };
  ASSERT_VEC_NEAR(q, ans);
}

// TODO : Add testcases for the other switch statement cases in dcm_to_quat
TEST(UtilitiesTest, DCMToQuaternionCase3Test) {
  lin::Vector4f q;
  lin::Matrix3x3f Q = {
     0.1333333f, 0.9333333f, -0.3333333f,
    -0.6666667f, 0.3333333f,  0.6666667f,
     0.7333333f, 0.1333333f,  0.6666667f
  };
  gnc::utl::dcm_to_quat(Q, q);
  // Expected answer was calculated in MATLAB
  lin::Vector4f ans = {0.1825742f, 0.3651484f, 0.5477226f, 0.7302967f};
  ASSERT_VEC_NEAR(q, ans);
}

TEST(UtilitiesTest, TriadTest) {
  // Randomly generated DCM in MATLAB using MGS
  lin::Matrix3x3f Q = {
    0.2449f,  0.8723f,  0.4233f,
    0.4809f, -0.4883f,  0.7282f,
    0.8419f,  0.0252f, -0.5390f
  };
  lin::Vector4f q, ans = {
    0.7537f,
    0.4488f,
    0.4197f,
    0.2332f
  };
  // Generate 'base frame' and 'body frame' readings
  lin::Vector3f n1 = { 0.1394f, 0.4143f, 0.8994f };
  lin::Vector3f n2 = { 0.5633f, 0.6822f, 0.4662f };
  lin::Vector3f b1 = Q * n1;
  lin::Vector3f b2 = Q * n2;
  // Perform triad
  gnc::utl::triad(n1, n2, b1, b2, q);
  ASSERT_VEC_NEAR(q, ans);
}

TEST(UtilitiesTest, VecToRotQuatTest) {
  lin::Vector4f q;
  lin::Vector3f u = {
    0.6651f,
    0.7395f,
    0.1037f
  };
  lin::Vector3f v = {
    0.8190f,
    0.5670f,
    0.0875f
  };
  gnc::utl::vec_rot_to_quat(u, v, q);
  // Expected answer was calculated in MATLAB
  lin::Vector4f ans = {0.0030f, 0.0135f, -0.1150f, 0.9933f};
  ASSERT_VEC_NEAR(q, ans);
}

TEST(UtilitiesTest, VecToRotQuatAntiparallelTest) {
  lin::Vector4f q;
  gnc::utl::vec_rot_to_quat({0.0f, 1.0f, 0.0f}, {0.0f, -1.0f, 0.0f}, q);
  // Expected answer for the antiparallel test
  lin::Vector4f ans = {0.0f, 0.0f, 1.0f, 0.0f};
  ASSERT_VEC_NEAR(q, ans);
}

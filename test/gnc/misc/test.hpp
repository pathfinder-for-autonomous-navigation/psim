/** @file test_all/test.hpp
 *  @author Kyle Krol */

#ifndef TEST_ALL_TEST_HPP_
#define TEST_ALL_TEST_HPP_

#define UNITY_EXCLUDE_MATH_H
#include <unity.h>
#undef isnan
#undef isinf

#define TEST_ASSERT_DOUBLE_VEC_NEAR(t, u, v) \
    TEST_ASSERT_DOUBLE_WITHIN(t, 0.0, lin::fro(u - v))

#define TEST_ASSERT_FLOAT_VEC_NEAR(t, u, v) \
    TEST_ASSERT_DOUBLE_WITHIN(t, 0.0, lin::fro(u - v))

#endif

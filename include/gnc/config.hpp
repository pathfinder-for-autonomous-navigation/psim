/** @file gnc/config.hpp
 *  @author Kyle Krol
 *  File included in all GNC translation units to define common macros and
 *  ensure required compiler options. */

#ifndef GNC_CONFIG_HPP_
#define GNC_CONFIG_HPP_

#include <lin/core.hpp>

#include <cmath>
#include <limits>
#include <type_traits>

/* Constant tracker macro for the GNC repository. Intended to be compatible with
 * the constant tracker from flight software. */
#define GNC_TRACKED_CONSTANT(type, name, ...) type name { __VA_ARGS__ }; static_assert(true, "")

/* Conditionaly disable asserts within the GNC library by defining
 * GNC_NO_CASSERT. */
#ifndef GNC_NO_CASSERT
  #include <cassert>
  #define GNC_ASSERT(x) assert(x)
#else
  #define GNC_ASSERT(x)
#endif

/** Assert some number is near another within some tolerance. This should be
 *  used with floating point values. */
#define GNC_ASSERT_NEAR(a, b, delta) GNC_ASSERT(std::abs(a - b) < delta)

/** Assert a vector is normalized. */
#define GNC_ASSERT_NORMALIZED(a) GNC_ASSERT_NEAR(1.0f, lin::fro(a), 1.0e-2f)

/* Ensure float literals are actually floats and double literals are actually
 * doubles. */
static_assert(std::is_same<float, decltype(0.0f)>::value, "float literal not treated as a float");
static_assert(std::is_same<double, decltype(0.0)>::value, "double literal not treated as a double");

/* Ensure quiet double and float NaNs. */
static_assert(std::numeric_limits<double>::has_quiet_NaN, "double quiet NaNs not supported");
static_assert(std::numeric_limits<float>::has_quiet_NaN, "float quiet NaNs not supported");

#endif

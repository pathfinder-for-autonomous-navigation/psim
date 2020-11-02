/** @file test/psim/core/state_field_lazy_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <psim/core/state_field_lazy.hpp>
#include <psim/core/types.hpp>

TEST(StateFieldLazy, TestConstructorAndGet) {
  psim::StateFieldLazy<psim::Real> field("default", []() { return 1.0; });

  ASSERT_STREQ(field.name().c_str(), "default");
  ASSERT_STREQ(field.type().c_str(), "state_field_lazy");
  ASSERT_EQ(field.get(), 1.0);
}

TEST(StateFieldLazy, TestReset) {
  auto value = 1.0;
  psim::StateFieldLazy<psim::Real> field(
      "default", [&]() { return 2.0 * value; });

  // Ensure the first evaluation returns the expected value
  ASSERT_EQ(field.get(), 2.0 * 1.0);

  // Change underlying value but ensure the field evaluation is cached
  value = 2.0;
  ASSERT_EQ(field.get(), 2.0 * 1.0);

  // Ensure the underlying value is now updated after reset was called
  field.reset();
  ASSERT_EQ(field.get(), 2.0 * 2.0);
}

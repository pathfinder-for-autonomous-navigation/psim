/** @file test/psim/core/state_field_lazy_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <psim/core/state_field_lazy.hpp>
#include <psim/core/types.hpp>

TEST(StateFieldLazy, TestConstructor) {
  psim::StateFieldLazy<psim::Real> field([]() { return 2.0; });
  ASSERT_EQ((psim::Real) field, 2.0);
}

TEST(StateFieldLazy, TestReset) {
  auto underlying_value = 1.0;
  auto const function = [&underlying_value]() {
    return 2.0 * underlying_value;
  };

  psim::StateFieldLazy<psim::Real> field(function);

  // Ensure the first evaluation returns the expected value
  ASSERT_EQ((psim::Real) field, 2.0 * 1.0);

  // Change underlying value but ensure the field evaluation is cached
  underlying_value = 2.0;
  ASSERT_EQ((psim::Real) field, 2.0 * 1.0);

  // Ensure the underlying value is now updated after reset was called
  field.reset();
  ASSERT_EQ((psim::Real) field, 2.0 * 2.0);
}

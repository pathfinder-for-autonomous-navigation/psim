/** @file test/psim/core/state_field_valued_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <psim/core/parameter.hpp>
#include <psim/core/state_field_valued.hpp>
#include <psim/core/types.hpp>

TEST(StateFieldValued, TestConstructor) {
  // Named default constructor
  {
    psim::StateFieldValued<psim::Real> field("default");

    ASSERT_STREQ(field.name().c_str(), "default");
    ASSERT_STREQ(field.type().c_str(), "state_field_valued");
    ASSERT_EQ(field.get(), psim::Real());
  }

  // Named value constructor
  {
    psim::StateFieldValued<psim::Real> field("default", 1.0);

    ASSERT_STREQ(field.name().c_str(), "default");
    ASSERT_STREQ(field.type().c_str(), "state_field_valued");
    ASSERT_EQ(field.get(), 1.0);
  }
}

TEST(StateFieldValued, TestGet) {
  psim::StateFieldValued<psim::Real> field("default", 1.0);

  // Test read only access
  {
    auto const &value = field.get();

    ASSERT_EQ(value, 1.0);
  }

  // Test read-write access
  {
    field.get() = 2.0;
    auto const &value = field.get();

    ASSERT_EQ(value, 2.0);
  }
}

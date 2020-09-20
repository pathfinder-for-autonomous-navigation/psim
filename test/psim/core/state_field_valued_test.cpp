/** @file test/psim/core/state_field_valued_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <psim/core/parameter.hpp>
#include <psim/core/state_field_valued.hpp>
#include <psim/core/types.hpp>

TEST(StateFieldValued, TestAssignment) {
  // Underlying value assignment
  {
    psim::StateFieldValued<psim::Real> field(2.0);
    field = 1.0;

    ASSERT_EQ((psim::Real) field, 1.0);
  }
}

TEST(StateFieldValued, TestConstructor) {
  // Default construct
  {
    psim::StateFieldValued<psim::Real> field;    
    ASSERT_EQ((psim::Real) field, psim::Real());
  }

  // Underlying value constructor
  {
    psim::StateFieldValued<psim::Real> field(1.0);
    ASSERT_EQ((psim::Real) field, 1.0);
  }

  // Parameter base constructor
  {
    psim::Parameter<psim::Real> param_base(2.0);
    psim::StateFieldValued<psim::Real> field((psim::ParameterBase &) param_base);

    ASSERT_EQ((psim::Real) field, 2.0);
  }
}

TEST(StateFieldValued, TestOperatorT) {
  psim::StateFieldValued<psim::Real> field(2.0);
  (psim::Real &) field = 3.0;

  ASSERT_EQ((psim::Real const &) field, 3.0);
}

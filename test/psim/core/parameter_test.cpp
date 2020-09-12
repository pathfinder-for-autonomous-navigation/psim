/** @file test/psim/core/parameter_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <psim/core/parameter.hpp>
#include <psim/core/types.hpp>

#include <stdexcept>

TEST(Parameter, TestAssignment) {
  // Underlying value assignment
  {
    psim::Parameter<psim::Real> param(0.0);
    param = 3.0;

    ASSERT_EQ((psim::Real) param, 3.0);
  }

  // Parameter base assignment
  {
    psim::Parameter<psim::Real> param_base(0.0);
    psim::ParameterBase *param_ptr = &param_base;

    // Test a proper assignment
    {
      psim::Parameter<psim::Real> param = 4.0;
      param = *param_ptr;

      ASSERT_EQ((psim::Real) param, 0.0);
    }

    // Ensure exception on invalid assignment
    {
      psim::Parameter<psim::Integer> param;

      EXPECT_THROW(param = *param_ptr, std::runtime_error);
    }
  }
}

TEST(Parameter, TestConstructor) {
  // Default constructor
  {
    psim::Parameter<psim::Real> param;
    ASSERT_EQ((psim::Real) param, psim::Real());
  }

  // Underlying value constructor
  {
    psim::Parameter<psim::Real> param(1.0);
    ASSERT_EQ((psim::Real) param, 1.0);
  }

  // Parameter base constructor
  {
    psim::Parameter<psim::Real> param_base(2.0);
    psim::Parameter<psim::Real> param((psim::ParameterBase &) param_base);

    ASSERT_EQ((psim::Real) param, 2.0);
  }
}

TEST(Parameter, TestGet) {
  psim::Parameter<psim::Real> param;
  psim::ParameterBase *param_ptr = &param;

  // Test a proper access
  auto const value = param_ptr->get<psim::Real>();
  ASSERT_EQ(value, psim::Real());

  // Ensure exception on improper access
  EXPECT_THROW(param_ptr->get<psim::Integer>(), std::runtime_error);
}

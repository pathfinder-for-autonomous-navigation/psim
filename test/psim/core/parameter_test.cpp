/** @file test/psim/core/parameter_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <psim/core/parameter.hpp>
#include <psim/core/types.hpp>

#include <stdexcept>

TEST(Parameter, TestConstructor) {
  // Default constructor
  {
    psim::Parameter<psim::Real> param;

    ASSERT_STREQ(param.name().c_str(), "");
    ASSERT_STREQ(param.type().c_str(), "parameter");
    ASSERT_EQ(param.get(), psim::Real());
  }

  // Named default constructor
  {
    psim::Parameter<psim::Real> param("default");

    ASSERT_STREQ(param.name().c_str(), "default");
    ASSERT_STREQ(param.type().c_str(), "parameter");
    ASSERT_EQ(param.get(), psim::Real());
  }

  // Value constructor
  {
    psim::Parameter<psim::Real> param(1.0);

    ASSERT_STREQ(param.name().c_str(), "");
    ASSERT_STREQ(param.type().c_str(), "parameter");
    ASSERT_EQ(param.get(), 1.0);
  }

  // Named value constructor
  {
    psim::Parameter<psim::Real> param("default", 1.0);

    ASSERT_STREQ(param.name().c_str(), "default");
    ASSERT_STREQ(param.type().c_str(), "parameter");
    ASSERT_EQ(param.get(), 1.0);
  }
}

TEST(Parameter, TestGet) {
  psim::Parameter<psim::Real> param(1.0);

  // Test read only access
  {
    auto const &value = param.get();

    ASSERT_EQ(value, 1.0);
  }

  // Test read-write access
  {
    param.get() = 2.0;
    auto const &value = param.get();

    ASSERT_EQ(value, 2.0);
  }

  // Test dynamic access (proper and improper)
  {
    psim::ParameterBase *ptr = &param;

    ASSERT_EQ(ptr->template get<psim::Real>(), 2.0);
    EXPECT_THROW(ptr->template get<psim::Integer>(), std::runtime_error);
  }
}

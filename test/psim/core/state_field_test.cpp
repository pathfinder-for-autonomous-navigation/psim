/** @file test/psim/core/state_field_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <psim/core/state_field.hpp>
#include <psim/core/state_field_lazy.hpp>
#include <psim/core/state_field_valued.hpp>
#include <psim/core/types.hpp>

#include <stdexcept>

TEST(StateField, TestCast) {
  psim::StateFieldValued<psim::Real> field("default");

  // Test for `StateFieldBase`
  {
    psim::StateFieldBase &base_field = field;

    ASSERT_EQ(&(base_field.cast<psim::Real>()), &field);
    EXPECT_THROW(base_field.cast<psim::Integer>(), std::runtime_error);
  }

  // Test for `StateFieldBase const`
  {
    psim::StateFieldBase const &base_field = field;

    ASSERT_EQ(&(base_field.cast<psim::Real>()), &field);
    EXPECT_THROW(base_field.cast<psim::Integer>(), std::runtime_error);
  }

  // Test for `StateField`
  {
    psim::StateField<psim::Real> &base_field = field;

    ASSERT_EQ(&(base_field.cast<psim::Real>()), &field);
    // EXPECT_THROW(base_field.cast<psim::Integer>(), std::runtime_error);
    //   ^ throws a state assertion error
  }

  // Test for `StateField const`
  {
    psim::StateField<psim::Real> const &base_field = field;

    ASSERT_EQ(&(base_field.cast<psim::Real>()), &field);
    // EXPECT_THROW(base_field.cast<psim::Integer>(), std::runtime_error);
    //   ^ throws a state assertion error
  }
}

TEST(StateField, TestCastWritable) {
  // Test for non-writable state fields
  {
    psim::StateFieldLazy<psim::Real> field("default", []() { return psim::Real(); });

    // Test for `StateFieldBase`
    {
      psim::StateFieldBase &base_field = field;

      EXPECT_THROW(base_field.cast_writable<psim::Real>(), std::runtime_error);
    }

    // Test for `StateFieldBase const`
    {
      psim::StateFieldBase const &base_field = field;

      EXPECT_THROW(base_field.cast_writable<psim::Real>(), std::runtime_error);
    }

    // Test for `StateField`
    {
      psim::StateField<psim::Real> *field_ptr = &field;

      EXPECT_THROW(field_ptr->cast_writable<psim::Real>(), std::runtime_error);
    }

    // Test for `StateField const`
    {
      psim::StateField<psim::Real> const &base_field = field;

      EXPECT_THROW(base_field.cast_writable<psim::Real>(), std::runtime_error);
    }
  }

  // Test for writable state fields
  {
    psim::StateFieldValued<psim::Real> field("default");

    // Test for `StateFieldBase`
    {
      psim::StateFieldBase &base_field = field;

      ASSERT_EQ(&(base_field.cast<psim::Real>()), &field);
      EXPECT_THROW(base_field.cast<psim::Integer>(), std::runtime_error);
    }

    // Test for `StateFieldBase const`
    {
      psim::StateFieldBase const &base_field = field;

      ASSERT_EQ(&(base_field.cast<psim::Real>()), &field);
      EXPECT_THROW(base_field.cast<psim::Integer>(), std::runtime_error);
    }

    // Test for `StateField`
    {
      psim::StateField<psim::Real> &base_field = field;

      ASSERT_EQ(&(base_field.cast<psim::Real>()), &field);
      // EXPECT_THROW(base_field.cast<psim::Integer>(), std::runtime_error);
      //   ^ throws a state assertion error
    }

    // Test for `StateField const`
    {
      psim::StateField<psim::Real> const &base_field = field;

      ASSERT_EQ(&(base_field.cast<psim::Real>()), &field);
      // EXPECT_THROW(base_field.cast<psim::Integer>(), std::runtime_error);
      //   ^ throws a state assertion error
    }
  }
}

TEST(StateField, TestGet) {
  psim::StateFieldValued<psim::Real> field("default", 2.0);

  // Test for `StateFieldBase`
  {
    psim::StateFieldBase &base_field = field;

    ASSERT_EQ(base_field.get<psim::Real>(), 2.0);
    EXPECT_THROW(base_field.get<psim::Integer>(), std::runtime_error);
  }

  // Test for `StateFieldBase const`
  {
    psim::StateFieldBase const &base_field = field;

    ASSERT_EQ(base_field.get<psim::Real>(), 2.0);
    EXPECT_THROW(base_field.get<psim::Integer>(), std::runtime_error);
  }

  // Test for `StateField`
  {
    psim::StateField<psim::Real> &base_field = field;

    ASSERT_EQ(base_field.get<psim::Real>(), 2.0);
    // EXPECT_THROW(base_field.get<psim::Integer>(), std::runtime_error);
    //   ^ throws a static assertion error
  }

  // Test for `StateField const`
  {
    psim::StateField<psim::Real> const &base_field = field;

    ASSERT_EQ(base_field.get<psim::Real>(), 2.0);
    // EXPECT_THROW(base_field.get<psim::Integer>(), std::runtime_error);
    //   ^ throws a static assertion error
  }
}

TEST(StateField, TestGetWritable) {
  // Test for non-writable state fields
  {
    psim::StateFieldLazy<psim::Real> field("default", []() { return psim::Real(); });

    // Test for `StateFieldBase`
    {
      psim::StateFieldBase &base_field = field;

      EXPECT_THROW(base_field.get_writable<psim::Real>(), std::runtime_error);
    }

    // Test for `StateField`
    {
      psim::StateField<psim::Real> &base_field = field;

      EXPECT_THROW(base_field.get_writable<psim::Real>(), std::runtime_error);
    }
  }

  // Test for writable state fields
  {
    psim::StateFieldValued<psim::Real> field("default", 2.0);

    // Test for `StateFieldBase`
    {
      psim::StateFieldBase &base_field = field;

      ASSERT_EQ(base_field.get_writable<psim::Real>(), 2.0);
      EXPECT_THROW(base_field.get_writable<psim::Integer>(), std::runtime_error);
    }

    // Test for `StateField`
    {
      psim::StateField<psim::Real> &base_field = field;

      ASSERT_EQ(base_field.get_writable<psim::Real>(), 2.0);
      // EXPECT_THROW(base_field.get_writable<psim::Integer>(), std::runtime_error);
      //   ^ throws a static assertion error
    }
  }
}

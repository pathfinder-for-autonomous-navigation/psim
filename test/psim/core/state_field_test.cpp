/** @file test/psim/core/state_field_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <psim/core/state_field.hpp>
#include <psim/core/state_field_base.hpp>
#include <psim/core/state_field_lazy.hpp>
#include <psim/core/state_field_valued.hpp>
#include <psim/core/state_field_writable_base.hpp>
#include <psim/core/state_field_writable.hpp>
#include <psim/core/types.hpp>

#include <stdexcept>

TEST(StateField, TestCast) {
  psim::StateFieldValued<psim::Real> field;

  // Test for `StateFieldBase`
  {
    psim::StateFieldBase *field_ptr = &field;

    ASSERT_EQ(field_ptr->cast<psim::Real>(), &field);
    EXPECT_THROW(field_ptr->cast<psim::Integer>(), std::runtime_error);
  }

  // Test for `StateFieldBase const`
  {
    psim::StateFieldBase const *field_ptr = &field;

    ASSERT_EQ(field_ptr->cast<psim::Real>(), &field);
    EXPECT_THROW(field_ptr->cast<psim::Integer>(), std::runtime_error);
  }

  // Test for `StateField`
  {
    psim::StateField<psim::Real> *field_ptr = &field;

    ASSERT_EQ(field_ptr->cast<psim::Real>(), &field);
    // EXPECT_THROW(field_ptr->cast<psim::Integer>(), std::runtime_error);
    //   ^ throws a static assertion error
  }

  // Test for `StateField const`
  {
    psim::StateField<psim::Real> const *field_ptr = &field;

    ASSERT_EQ(field_ptr->cast<psim::Real>(), &field);
    // EXPECT_THROW(field_ptr->cast<psim::Integer>(), std::runtime_error);
    //   ^ throws a static assertion error
  }
}

TEST(StateField, TestCastWritable) {
  // Test for non-writable state fields
  {
    psim::StateFieldLazy<psim::Real> field([]() { return psim::Real(); });

    // Test for `StateFieldBase`
    {
      psim::StateFieldBase *field_ptr = &field;

      EXPECT_THROW(field_ptr->cast_writable<psim::Real>(), std::runtime_error);
    }

    // Test for `StateFieldBase const`
    {
      psim::StateFieldBase const *field_ptr = &field;

      EXPECT_THROW(field_ptr->cast_writable<psim::Real>(), std::runtime_error);
    }

    // Test for `StateField`
    {
      psim::StateField<psim::Real> *field_ptr = &field;

      EXPECT_THROW(field_ptr->cast_writable<psim::Real>(), std::runtime_error);
    }

    // Test for `StateField const`
    {
      psim::StateField<psim::Real> const *field_ptr = &field;

      EXPECT_THROW(field_ptr->cast_writable<psim::Real>(), std::runtime_error);
    }
  }

  // Test for writable state fields
  {
    psim::StateFieldValued<psim::Real> field;

    // Test for `StateFieldBase`
    {
      psim::StateFieldBase *field_ptr = &field;

      ASSERT_EQ(field_ptr->cast_writable<psim::Real>(), &field);
      EXPECT_THROW(field_ptr->cast_writable<psim::Integer>(), std::runtime_error);
    }

    // Test for `StateFieldBase const`
    {
      psim::StateFieldBase const *field_ptr = &field;

      ASSERT_EQ(field_ptr->cast_writable<psim::Real>(), &field);
      EXPECT_THROW(field_ptr->cast_writable<psim::Integer>(), std::runtime_error);
    }

    // Test for `StateField`
    {
      psim::StateField<psim::Real> *field_ptr = &field;

      ASSERT_EQ(field_ptr->cast_writable<psim::Real>(), &field);
      // EXPECT_THROW(field_ptr->cast_writable<psim::Integer>(), std::runtime_error);
      //   ^ throws a static assertion error
    }

    // Test for `StateField const`
    {
      psim::StateField<psim::Real> const *field_ptr = &field;

      ASSERT_EQ(field_ptr->cast_writable<psim::Real>(), &field);
      // EXPECT_THROW(field_ptr->cast_writable<psim::Integer>(), std::runtime_error);
      //   ^ throws a static assertion error
    }
  }
}

TEST(StateField, TestGet) {
  psim::StateFieldValued<psim::Real> field(2.0);

  // Test for `StateFieldBase`
  {
    psim::StateFieldBase *field_ptr = &field;

    ASSERT_EQ(field_ptr->get<psim::Real>(), 2.0);
    EXPECT_THROW(field_ptr->get<psim::Integer>(), std::runtime_error);
  }

  // Test for `StateFieldBase const`
  {
    psim::StateFieldBase const *field_ptr = &field;

    ASSERT_EQ(field_ptr->get<psim::Real>(), 2.0);
    EXPECT_THROW(field_ptr->get<psim::Integer>(), std::runtime_error);
  }

  // Test for `StateField`
  {
    psim::StateField<psim::Real> *field_ptr = &field;

    ASSERT_EQ(field_ptr->get<psim::Real>(), 2.0);
    // EXPECT_THROW(field_ptr->cast<psim::Integer>(), std::runtime_error);
    //   ^ throws a static assertion error
  }

  // Test for `StateField const`
  {
    psim::StateField<psim::Real> const *field_ptr = &field;

    ASSERT_EQ(field_ptr->get<psim::Real>(), 2.0);
    // EXPECT_THROW(field_ptr->cast<psim::Integer>(), std::runtime_error);
    //   ^ throws a static assertion error
  }
}

TEST(StateField, TestGetWritable) {
  // Test for non-writable state fields
  {
    psim::StateFieldLazy<psim::Real> field([]() { return psim::Real(); });

    // Test for `StateFieldBase`
    {
      psim::StateFieldBase *field_ptr = &field;

      EXPECT_THROW(field_ptr->get_writable<psim::Real>(), std::runtime_error);
    }

    // Test for `StateField`
    {
      psim::StateField<psim::Real> *field_ptr = &field;

      EXPECT_THROW(field_ptr->get_writable<psim::Real>(), std::runtime_error);
    }
  }

  // Test for writable state fields
  {
    psim::StateFieldValued<psim::Real> field(2.0);

    // Test for `StateFieldBase`
    {
      psim::StateFieldBase *field_ptr = &field;

      ASSERT_EQ(field_ptr->get_writable<psim::Real>(), 2.0);
      EXPECT_THROW(field_ptr->get_writable<psim::Integer>(), std::runtime_error);
    }

    // Test for `StateField`
    {
      psim::StateField<psim::Real> *field_ptr = &field;

      ASSERT_EQ(field_ptr->get_writable<psim::Real>(), 2.0);
      // EXPECT_THROW(field_ptr->get_writable<psim::Integer>(), std::runtime_error);
      //   ^ throws a static assertion error
    }
  }
}

TEST(StateField, TestOperatorT) {
  // Test for non-writable state fields
  {
    psim::StateFieldValued<psim::Real> field(2.0);
    ASSERT_EQ((psim::Real) field, 2.0);
  }

  // Test for writable state fields
  {
    psim::StateFieldLazy<psim::Real> field([]() { return 2.0; });
    ASSERT_EQ((psim::Real) field, 2.0);
  }
}

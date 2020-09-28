/** @file test/psim/core/state_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <psim/core/state.hpp>
#include <psim/core/state_field_valued.hpp>
#include <psim/core/types.hpp>

#include <stdexcept>

class State : public testing::Test {
 protected:
  psim::StateFieldValued<psim::Real> field0;
  psim::StateFieldValued<psim::Real> field1;
  psim::StateFieldValued<psim::Real> field2;
  psim::StateFieldValued<psim::Real> field3;
  psim::StateFieldValued<psim::Real> field4;
  psim::State state;

 public:
  State()
     : field0("field0"), field1("field1"), field2("field2"), field3("field3"),
       field4("field4") { }

  void SetUp() override {
    state.add(&field0);
    state.add(&field1);
    state.add(&field2);
    state.add_writable(&field3);
    state.add_writable(&field4);
  }
};

TEST_F(State, TestHas) {
  ASSERT_TRUE(state.has("field1"));
  ASSERT_TRUE(state.has("field4"));
  ASSERT_FALSE(state.has("field"));
}

TEST_F(State, TestHasWritable) {
  ASSERT_TRUE(state.has_writable("field4"));
  ASSERT_FALSE(state.has_writable("field1"));
  ASSERT_FALSE(state.has_writable("field"));
}

TEST_F(State, TestGet) {
  ASSERT_EQ(state.get("field0"), &field0);
  ASSERT_EQ(state.get("field4"), &field4);
  ASSERT_EQ(state.get("field5"), nullptr);
}

TEST_F(State, TestGetWritable) {
  ASSERT_EQ(state.get_writable("field0"), nullptr);
  ASSERT_EQ(state.get_writable("field4"), &field4);
  ASSERT_EQ(state.get_writable("field5"), nullptr);
}

TEST_F(State, TestBracketsOperator) {
  ASSERT_EQ(&state["field0"], &field0);
  ASSERT_EQ(&state["field4"], &field4);
  EXPECT_THROW(state["field5"], std::runtime_error);
}

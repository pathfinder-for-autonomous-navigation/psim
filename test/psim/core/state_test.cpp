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
  psim::StateFieldValued<psim::Real> fields[5];
  psim::State state;

 public:
  void SetUp() override {
    state.add("field0", &fields[0]);
    state.add("field1", &fields[1]);
    state.add("field2", &fields[2]);
    state.add_writable("field3", &fields[3]);
    state.add_writable("field4", &fields[4]);
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
  ASSERT_EQ(state.get("field0"), &fields[0]);
  ASSERT_EQ(state.get("field4"), &fields[4]);
  ASSERT_EQ(state.get("field5"), nullptr);
}

TEST_F(State, TestGetWritable) {
  ASSERT_EQ(state.get_writable("field0"), nullptr);
  ASSERT_EQ(state.get_writable("field4"), &fields[4]);
  ASSERT_EQ(state.get_writable("field5"), nullptr);
}

TEST_F(State, TestBracketsOperator) {
  ASSERT_EQ(&state["field0"], &fields[0]);
  ASSERT_EQ(&state["field4"], &fields[4]);
  EXPECT_THROW(state["field5"], std::runtime_error);
}

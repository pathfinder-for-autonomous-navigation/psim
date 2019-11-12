//
// test/test_all/containers_test.cpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#include "test.hpp"
#include "containers_test.hpp"

#include <gnc_containers.hpp>

void subtest_circular_buffer(gnc::CircularBuffer<float, 3> &buffer) {
  // Test properties on when cleared/constructed
  TEST_ASSERT_EQUAL_UINT(3, buffer.max_size());
  TEST_ASSERT_EQUAL_UINT(0, buffer.size());
  TEST_ASSERT_TRUE(buffer.empty());
  TEST_ASSERT_FALSE(buffer.full());
  // Test properties after adding one element
  buffer.push(0.0f);
  TEST_ASSERT_EQUAL_UINT(1, buffer.size());
  TEST_ASSERT_FALSE(buffer.empty());
  TEST_ASSERT_FALSE(buffer.full());
  TEST_ASSERT_EQUAL_FLOAT(buffer[0], 0.0f);
  // Test properties after adding three elements
  buffer.push(1.0f);
  buffer.push(2.0f);
  TEST_ASSERT_EQUAL_UINT(3, buffer.size());
  TEST_ASSERT_FALSE(buffer.empty());
  TEST_ASSERT_TRUE(buffer.full());
  TEST_ASSERT_EQUAL_FLOAT(buffer[0], 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(buffer[1], 1.0f);
  TEST_ASSERT_EQUAL_FLOAT(buffer[2], 2.0f);
  // Test pushing to full buffer
  buffer.push(3.0f);
  TEST_ASSERT_EQUAL_UINT(3, buffer.size());
  TEST_ASSERT_FALSE(buffer.empty());
  TEST_ASSERT_TRUE(buffer.full());
  TEST_ASSERT_EQUAL_FLOAT(buffer[0], 1.0f);
  TEST_ASSERT_EQUAL_FLOAT(buffer[1], 2.0f);
  TEST_ASSERT_EQUAL_FLOAT(buffer[2], 3.0f);
}

void test_containers_circular_buffer() {
  gnc::CircularBuffer<float, 3> buffer;
  for (unsigned int i = 0; i < 4; i++) { // Test many starting offsets
    subtest_circular_buffer(buffer);
    buffer.clear();
  }
}

void containers_test() {
  RUN_TEST(test_containers_circular_buffer);
}

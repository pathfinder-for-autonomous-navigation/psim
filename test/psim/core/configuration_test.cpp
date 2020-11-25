/** @file test/psim/core/configuration_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <psim/core/configuration.hpp>
#include <psim/core/types.hpp>

#include <stdexcept>

TEST(Configuration, TestBadName) {
  std::string const file =
      "test/psim/core/configuration_test_bad_name_config.txt";
  EXPECT_THROW(psim::Configuration _(file), std::runtime_error);
}

TEST(Configuration, TestBadValue) {
  std::string const file =
      "test/psim/core/configuration_test_bad_value_config.txt";
  EXPECT_THROW(psim::Configuration _(file), std::runtime_error);
}

TEST(Configuration, TestDuplicateName) {
  std::string const file =
      "test/psim/core/configuration_test_duplicate_name_config.txt";
  EXPECT_THROW(psim::Configuration _(file), std::runtime_error);
}

TEST(Configuration, TestFieldNotFound) {
  std::string const file = "test/psim/core/configuration_test_dne_config.txt";
  EXPECT_THROW(psim::Configuration _(file), std::runtime_error);
}

TEST(Configuration, TestGet) {
  std::string const file = "test/psim/core/configuration_test_config.txt";
  auto const config = psim::Configuration(file);

  // Test a proper access
  ASSERT_EQ(config.get("test.integer")->template get<psim::Integer>(), 1);

  // Ensure a null pointer on an invalid access
  ASSERT_EQ(config.get("test.dne"), nullptr);
}

TEST(Configuration, TestMultiFileMake) {
  std::vector<std::string> const files = {
      "test/psim/core/configuration_test_config.txt",
      "test/psim/core/configuration_test_two_config.txt"};
  auto const config = psim::Configuration(files);

  // Check Integer field(s) from both files
  {
    psim::Integer test_integer;

    test_integer = config["test.integer"].template get<psim::Integer>();
    ASSERT_EQ(test_integer, 1);

    test_integer = config["test.i"].template get<psim::Integer>();
    ASSERT_EQ(test_integer, -2);

    test_integer = config["test.i"].template get<psim::Integer>();
    ASSERT_EQ(test_integer, -2);

    test_integer = config["test.j"].template get<psim::Integer>();
    ASSERT_EQ(test_integer, 4);
  }
}

TEST(Configuration, TestSingleFileMake) {
  std::string const file = "test/psim/core/configuration_test_config.txt";
  auto const config = psim::Configuration(file);

  // Check Integer field(s)
  {
    psim::Integer test_integer;

    test_integer = config["test.integer"].template get<psim::Integer>();
    ASSERT_EQ(test_integer, 1);

    test_integer = config["test.i"].template get<psim::Integer>();
    ASSERT_EQ(test_integer, -2);
  }

  // Check Real field(s)
  {
    psim::Real test_real;

    test_real = config["test.real"].template get<psim::Real>();
    ASSERT_DOUBLE_EQ(test_real, -1.0);
  }

  // Check Vector2 field(s)
  {
    psim::Vector2 test_vector;

    test_vector = config["test.vector2"].template get<psim::Vector2>();
    ASSERT_DOUBLE_EQ(test_vector(0), 1.0);
    ASSERT_DOUBLE_EQ(test_vector(1), 2.0);
  }

  // Check Vector3 field(s)
  {
    psim::Vector3 test_vector;

    test_vector = config["test.vector3"].template get<psim::Vector3>();
    ASSERT_DOUBLE_EQ(test_vector(0), 1.0);
    ASSERT_DOUBLE_EQ(test_vector(1), -2.0);
    ASSERT_DOUBLE_EQ(test_vector(2), 3.0);
  }

  // Check Vector4 field(s)
  {
    psim::Vector4 test_vector;

    test_vector = config["test.vector4"].template get<psim::Vector4>();
    ASSERT_DOUBLE_EQ(test_vector(0), 1.0);
    ASSERT_DOUBLE_EQ(test_vector(1), 2.0);
    ASSERT_DOUBLE_EQ(test_vector(2), 3.0);
    ASSERT_DOUBLE_EQ(test_vector(3), 4.0);
  }

  // Ensure exception on an invalid access
  EXPECT_THROW(config["test.dne"], std::runtime_error);
}

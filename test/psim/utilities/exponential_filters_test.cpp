/** @file test/psim/utilities/exponential_filter_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>

#include <psim/core/configuration.hpp>
#include <psim/core/state.hpp>
#include <psim/core/state_field_valued.hpp>
#include <psim/core/types.hpp>
#include <psim/utilities/exponential_filters.hpp>

using namespace psim;

TEST(ExponentialFilterReal, TestStep) {
  Configuration const config("test/psim/utilities/exponential_filters_test.txt");
  RandomsGenerator randoms;
  State state;

  StateFieldValued<Real> value("value", 1.0);
  state.add(&value);

  ExponentialFilterReal filter(randoms, config, "value");
  filter.add_fields(state);
  filter.get_fields(state);

  for (auto i = 0; i < 2; i++) {
    filter.step();
    GTEST_ASSERT_EQ(state.get("value.filtered")->template get<Real>(), 1.0);
  }

  value.get() = 0.0;

  Real const alpha = config["value.alpha"].get<Real>();

  filter.step();
  GTEST_ASSERT_EQ(state.get("value.filtered")->template get<Real>(), 1.0 - alpha);

  filter.step();
  GTEST_ASSERT_EQ(state.get("value.filtered")->template get<Real>(), (1.0 - alpha) * (1.0 - alpha));
}

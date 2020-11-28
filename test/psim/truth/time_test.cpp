/** @file test/psim/truth/time_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <psim/core/configuration.hpp>
#include <psim/core/simulation.hpp>
#include <psim/truth/time.hpp>

TEST(Time, TestStep) {
  auto const config = psim::Configuration("test/psim/truth/time_test_config.txt");
  psim::Simulation<psim::Time> sim(config);

  // Assert intial conditions
  ASSERT_EQ(sim["truth.dt.ns"].get<psim::Integer>(), 5);
  ASSERT_EQ(sim["truth.t.ns"].get<psim::Integer>(), 0);

  // Check the time.t.ns field is updated
  sim.step();
  sim.step();
  ASSERT_EQ(sim["truth.t.ns"].get<psim::Integer>(), 10);

  // Check the lazy evaluated fiels
  ASSERT_DOUBLE_EQ(sim["truth.dt.s"].get<psim::Real>(), 5e-9);
  ASSERT_DOUBLE_EQ(sim["truth.t.s"].get<psim::Real>(), 1e-8);

  // Update time.dt.ns and try stepping again
  sim.get_writable("truth.dt.ns")->get<psim::Integer>() = 4;
  sim.step();
  ASSERT_EQ(sim["truth.t.ns"].get<psim::Integer>(), 14);
}

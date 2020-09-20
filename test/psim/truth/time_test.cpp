/** @file test/psim/truth/time_test.cpp
 *  @author Kyle Krol
 */

#include <gtest/gtest.h>

#include <psim/core/configuration.hpp>
#include <psim/core/simulation.hpp>
#include <psim/truth/time.hpp>

TEST(Time, TestStep) {
  auto const config = psim::Configuration::make("test/psim/truth/time_test_config.txt");
  auto sim = psim::Simulation::make<psim::Time>(config, "time");

  // Assert intial conditions
  ASSERT_EQ(sim["time.dt.ns"].get<psim::Integer>(), 5);
  ASSERT_EQ(sim["time.t.ns"].get<psim::Integer>(), 0);

  // Check the time.t.ns field is updated
  sim.step();
  sim.step();
  ASSERT_EQ(sim["time.t.ns"].get<psim::Integer>(), 10);

  // Check the lazy evaluated fiels
  ASSERT_DOUBLE_EQ(sim["time.dt.s"].get<psim::Real>(), 5e-9);
  ASSERT_DOUBLE_EQ(sim["time.t.s"].get<psim::Real>(), 1e-8);

  // Update time.dt.ns and try stepping again
  sim.get_writable("time.dt.ns")->get<psim::Integer>() = 4;
  sim.step();
  ASSERT_EQ(sim["time.t.ns"].get<psim::Integer>(), 14);
}

/** @file test/psim/core/simulation_test.cpp
 *  @author Kyle Krol
 */

#include "counter.hpp"

#include <gtest/gtest.h>

#include <psim/core/configuration.hpp>
#include <psim/core/simulation.hpp>
#include <psim/core/types.hpp>

TEST(Simulation, TestStep) {
  auto const config =
      psim::Configuration("test/psim/core/simulation_test_config.txt");
  auto sim = psim::Simulation::make<Counter>(config);

  // Check initial conditions
  ASSERT_EQ(sim["dn"].template get<psim::Integer>(), 1);
  ASSERT_EQ(sim["n"].template get<psim::Integer>(), 0);

  // Ensure step update the proper fields
  sim.step();
  ASSERT_EQ(sim["n"].template get<psim::Integer>(), 1);
}

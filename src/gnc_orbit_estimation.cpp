//
// src/gnc_orbit_estimation.cpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#include <gnc_environment.hpp>
#include <gnc_orbit_estimation.hpp>
#include <gnc_utilities.hpp>

namespace gnc {

void estimate_orbit(OrbitState const *state1, OrbitState const *state2,
    GpsData const *gps1, GpsData const *gps2, double t,
    OrbitStateEstimate **state1_est, OrbitStateEstimate **state2_est) {
  return;
}

void estimate_orbit_rtk(OrbitState const *state1, OrbitState const *state2,
    GpsRtkData const *gps1, double t, OrbitStateEstimate **state1_est,
    OrbitStateEstimate **state2_est) {
  return;
}
}  // namespace gnc

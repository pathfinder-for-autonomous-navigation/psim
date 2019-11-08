//
// include/gnc_orbit_estimation.hpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#ifndef PAN_PSIM_INCLUDE_GNC_ESTIMATION_HPP_
#define PAN_PSIM_INCLUDE_GNC_ESTIMATION_HPP_

#include "lin.hpp"

namespace gnc {

/** @struct OrbitState
 *  Minimal representation of a spacecraft's orbital state. It contains
 *  timestamped position and velocity vectors in ECI. The time should be seconds
 *  since the 'PAN epoch'. */
struct OrbitState {
  double t;
  lin::Vector3d r_eci;
  lin::Vector3d v_eci;
};

/** @struct OrbitStateEstimate
 *  Serves as an output from the orbit estimation function. It's an overdefined
 *  state that includes timestamped position and velocity vector in ECI and
 *  ECEF. The time should be seconds since the 'PAN epoch'. */
struct OrbitStateEstimate {
  double t;
  lin::Vector3d r_eci;
  lin::Vector3d v_eci;
  lin::Vector3d r_ecef;
  lin::Vector3d v_ecef;
};

/** @struct GpsData
 *  Serves as a wrapper around a timestamped GPS position and velocity data in
 *  ECEF. The time should be seconds since the 'PAN epoch'. */
struct GpsData {
  double t;
  lin::Vector3d r_ecef;
  lin::Vector3d v_ecef;
};

/** @fn estimate_orbit
 *  @brief Calculates orbital state estimates for both satellites.
 *  @param[in]  state1     Previous state of this satellite (nullptr if
 *                         unknown).
 *  @param[in]  state2     Previous state of that satellite (nulltpr if
 *                         unknown).
 *  @param[in]  gps1       GPS data from this satellite (nullptr if unknown).
 *  @param[in]  gps2       GPS data from that satellite (nullptr if unknown).
 *  @param[in]  t          Desired timestamp for the output estimate.
 *  @param[out] state1_est Destination for the state estimate of this satellite
 *                         (set to nullptr if undetermined).
 *  @param[out] state2_est Destination for the state estimate of that satellite
 *                         (set to nullptr if undetermined). */
void estimate_orbit(OrbitState const *state1, OrbitState const *state2,
    GpsData const *gps1, GpsData const *gps2, double t,
    OrbitStateEstimate **state1_est, OrbitStateEstimate **state2_est);

/** @struct GpsRtkData
 *  Serves as a wrapper around a timestamped GPS position in ECEF, velocity in
 *  ECEF, and relative RTK position in ECEF. */
struct GpsRtkData {
  double t;
  lin::Vector3d r_ecef;
  lin::Vector3d v_ecef;
  lin::Vector3d dr_ecef;
};

/** @struct RtkOrbitEstimatorState */
struct RtkOrbitEstimatorState { };

/** @fn estimate_orbit_rtk
 *  @param[in]  state1
 *  @param[in]  state2
 *  @param[in]  gps1
 *  @param[in]  t
 *  @param[out] state1_est
 *  @param[out] state2_est */
void estimate_orbit_rtk(RtkOrbitEstimatorState *state, OrbitState const *state1,
    OrbitState const *state2, GpsRtkData const *gps1, double t,
    OrbitStateEstimate **state1_est, OrbitStateEstimate **state2_est);

}  // namespace gnc

#endif

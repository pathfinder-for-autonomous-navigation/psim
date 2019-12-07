//
// include/gnc_attitude_estimation.hpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#ifndef PAN_PSIM_INCLUDE_GNC_ATTITUDE_ESTIMATION_HPP_
#define PAN_PSIM_INCLUDE_GNC_ATTITUDE_ESTIMATION_HPP_

#include "gnc_containers.hpp"
#include "lin.hpp"

namespace gnc {

/** @struct AttitudeEstimatorState
 *  Contains internal state information needed by the estimate_attitude
 *  function. */
struct AttitudeEstimatorState {
  double t;
  lin::Vector4f q_body_eci;
  CircularBuffer<lin::Vector3f, 5> w_buffer;
  /** Defaults everything's value to NaN. */
  AttitudeEstimatorState();
};

/** @struct AttitudeEstimatorData
 *  Essentially serves as the inputs to the estimate_attitude function. See
 *  member documentation for more details. */
struct AttitudeEstimatorData {
  /** Timestamp for the rest of the data in this struct and the time for which
   *  an attitude estimate will be calculated. This should be in seconds since
   *  the 'PAN epoch'. */
  double t;
  /** Position of the satellite in ECEF (units m). */
  lin::Vector3d r_ecef;
  /** Magnetic field vector in the body frame of the spacecraft (units T). */
  lin::Vector3f b_body;
  /** Sun vector in the body frame of the spacecraft (unit vector). */
  lin::Vector3f s_body;
  /** Defaults everything's value to NaN. */
  AttitudeEstimatorData();
};

/** @struct AttitudeEstimate
 *  Essentially serves as the outputs to the estimate_attitude function. See
 *  member documentation for more details. */
struct AttitudeEstimate {
  /** Quaternion transforming from the body frame to ECI. */
  lin::Vector4f q_body_eci;
  /** Angular rate of the spacecraft in the body frame. */
  lin::Vector3f w_body;
  /** Defaults everything's value to NaN. */
  AttitudeEstimate();
};

/** @fn estimate_attitude
 *  @brief Calculates an attitude quaternion and spacecraft angular rate.
 *  @param[in]  state    Estimate function internal state information.
 *  @param[in]  data     Estimate function inputs.
 *  @param[out] estimate Estimate output. */
void estimate_attitude(AttitudeEstimatorState &state,
    AttitudeEstimatorData const &data, AttitudeEstimate &estimate);

}  // namespace gnc

#endif

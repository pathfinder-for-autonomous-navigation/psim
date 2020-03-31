/** @file gnc/attitude_estimator.hpp
 *  @author Kyle Krol
 *  Defines the interface for the attitude estimators. */

#ifndef GNC_ATTITUDE_ESTIMATOR_HPP_
#define GNC_ATTITUDE_ESTIMATOR_HPP_

#include "config.hpp"
#include "containers.hpp"

#include <lin/core.hpp>

namespace gnc {

/** @struct AttitudeEstimatorState
 *  Contains internal state information needed by the estimate_attitude
 *  function. */
struct AttitudeEstimatorState {
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
  /** Measured magnetic field in the body frame of the satellite (units T). */
  lin::Vector3f b_body;
  /** Measured sun vector in the body frame of the satellite (unit vector). */
  lin::Vector3f s_body;
  /** Measured angular rate of the satellite (units radians/s). */
  lin::Vector3f w_body;
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

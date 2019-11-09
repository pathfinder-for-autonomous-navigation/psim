//
// src/gnc_attitude_estimationr.cpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#include <gnc_attitude_estimation.hpp>
#include <gnc_environment.hpp>
#include <gnc_utilities.hpp>

#include <cmath>
#include <limits>

static_assert(std::numeric_limits<double>::has_quiet_NaN,
    "GNC code requires quiet NaN's to be available.");
static_assert(std::numeric_limits<float>::has_quiet_NaN,
    "GNC code requires quiet NaN's to be available.");

namespace gnc {

AttitudeEstimatorState::AttitudeEstimatorState() {
  this->t = std::numeric_limits<double>::quiet_NaN();
  this->q_body_eci = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN()
  };
}

AttitudeEstimatorData::AttitudeEstimatorData() {
  this->t = std::numeric_limits<double>::quiet_NaN();
  this->r_ecef = {
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()
  };
  this->b_body = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN()
  };
  this->s_body = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN()
  };
}

AttitudeEstimate::AttitudeEstimate() {
  this->q_body_eci = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN()
  };
  this->w_body = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN()
  };
}

void estimate_attitude(AttitudeEstimatorState &state,
    AttitudeEstimatorData const &data, AttitudeEstimate &estimate) {

  // Default everything to NaN and ensure the magnetic field vector, sun vector,
  // and ECEF position were given
  estimate = AttitudeEstimate();
  if (std::isnan(data.b_body(0)) || std::isnan(data.s_body(0)) || std::isnan(data.r_ecef(0)))
    return;

  // Clear the state if it's outdated
  if (data.t - state.t > 1.0) state = AttitudeEstimatorState();

  // Query the magnetic field and sun vector environmental models
  lin::Vector3f b_eci, s_eci, r_ecef = {
    static_cast<float>(data.r_ecef(0)),
    static_cast<float>(data.r_ecef(1)),
    static_cast<float>(data.r_ecef(2))
  };
  env::sun_vector(data.t, s_eci);
  env::magnetic_field(data.t, r_ecef, b_eci);

  // Perform triad and check result
  lin::Vector4f q_body_eci;
  int code = utl::triad(s_eci, b_eci, data.s_body, data.b_body, q_body_eci);
  if (code) return;
  
  // Update the estimate and perform the time derivative
  estimate.q_body_eci = q_body_eci;

  // Record the determined state for the next iteration
  state = AttitudeEstimatorState();
  state.t = data.t;
  state.q_body_eci = q_body_eci;

  // TODO : Add estimation for angular rate

}
}  // namespace gnc

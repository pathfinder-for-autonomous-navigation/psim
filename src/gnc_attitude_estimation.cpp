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
#include <gnc_constants.hpp>
#include <gnc_environment.hpp>
#include <gnc_utilities.hpp>

#include <cmath>

static_assert(std::numeric_limits<double>::has_quiet_NaN,
    "GNC code requires quiet NaN's to be available.");
static_assert(std::numeric_limits<float>::has_quiet_NaN,
    "GNC code requires quiet NaN's to be available.");

namespace gnc {

AttitudeEstimatorState::AttitudeEstimatorState() {
  this->t = std::numeric_limits<double>::quiet_NaN();
  this->q_body_eci = lin::nans<decltype(this->q_body_eci)>();
  this->w_buffer = CircularBuffer<lin::Vector3f, 5>();
}

AttitudeEstimatorData::AttitudeEstimatorData() {
  this->t = std::numeric_limits<double>::quiet_NaN();
  this->r_ecef = lin::nans<decltype(this->r_ecef)>();
  this->b_body = lin::nans<decltype(this->b_body)>();
  this->s_body = lin::nans<decltype(this->s_body)>();
}

AttitudeEstimate::AttitudeEstimate() {
  this->q_body_eci = lin::nans<decltype(this->q_body_eci)>();
  this->w_body = lin::nans<decltype(this->w_body)>();
}

/** @fn estimate_q_body_eci
 *  @returns Zero of success.
 *  Attempts to determine attitude given time, position, a sun vector, and
 *  magnetic field vector. The result, if succesful, is written into the
 *  quaternion q_body_eci.
 *  Prior to running triad, we ensure the following conditions are met:
 *   - Magnitude of the expected magnetic field measurement is above the noise
 *     floor of the magnetometer.
 *   - Magnitude of the measured magnetic field is above the noise floor of the
 *     magnetometer. */
static int estimate_q_body_eci(double t, lin::Vector3d const &r_ecef,
    lin::Vector3f const &s_body, lin::Vector3f const &b_body, lin::Vector4f &q_body_eci) {

  // Setup the rest of the triad inputs (including expected sun and magnetic
  // field vectors in ECI)
  lin::Vector3f s_eci, b_eci, b_u_body, r_ecef_f({
    static_cast<float>(r_ecef(0)),
    static_cast<float>(r_ecef(1)),
    static_cast<float>(r_ecef(2))
  });
  env::sun_vector(t, s_eci);
  env::magnetic_field(t, r_ecef_f, b_eci);

  // Verify magnetic field vectors have are large enough
  constexpr static float threshold = constant::b_noise_floor_f * constant::b_noise_floor_f;
  if (lin::fro(b_eci) <= threshold || lin::fro(b_body) <= threshold)
    return 1;

  // Convert to unit vectors
  b_eci = b_eci / lin::norm(b_eci);
  b_u_body = b_body / lin::norm(b_body);

  // Perform triad and return the result code (result is written to q_body_eci)
  return utl::triad(s_eci, b_eci, s_body, b_u_body, q_body_eci);

}

/** @fn estimate_w_body
 *  Approximates the angular rate of a body given a quaternion q_m measured at a
 *  time t and a quaternion q_n measured at a time t + dt. */
static void estimate_w_body(double dt, lin::Vector4f const &q_m,
    lin::Vector4f const &q_n, lin::Vector3f &w) {

  // Find the 'difference' between the two quaternions
  lin::Vector4f q_n_conj, dq;
  utl::quat_conj(q_n, q_n_conj);
  utl::quat_cross_mult(q_m, q_n_conj, dq);

  // Approximate w_body
  w = ( dq(3) < 0.0f ? -1.0f : 1.0f ) * lin::ref<3, 1>((2.0f / static_cast<float>(dt)) * dq, 0, 0);

}

void estimate_attitude(AttitudeEstimatorState &state,
    AttitudeEstimatorData const &data, AttitudeEstimate &estimate) {

  // Clear the state if it's outdated (more than ~5 control cycles old)
  if (data.t - state.t > 0.5) state = AttitudeEstimatorState();

  // Default everything to NaN and ensure the magnetic field vector, sun vector,
  // ECEF position, and time were given
  estimate = AttitudeEstimate();
  if (std::isnan(data.b_body(0)) || std::isnan(data.s_body(0)) || std::isnan(data.r_ecef(0)) ||
      std::isnan(data.t))
    return;

  // Estimate the attitude quaternion
  lin::Vector4f q_body_eci;
  int code = estimate_q_body_eci(data.t, data.r_ecef, data.s_body, data.b_body, q_body_eci);
  if (code) return;  // Atitude determination failed
  
  // Update the estimate
  estimate.q_body_eci = q_body_eci;

  // If we don't have an old estimate just write the current one into state and
  // wait till the next call to estimate w_body
  if (std::isnan(state.t) || std::isnan(state.q_body_eci(0))) {
    state.t = data.t;
    state.q_body_eci = q_body_eci;
    state.w_buffer.clear();
    return;
  }

  // Estimate the current w_body and update state accordingly
  lin::Vector3f w_body;
  estimate_w_body(data.t - state.t, state.q_body_eci, q_body_eci, w_body);
  state.w_buffer.push(w_body);

  // Moving average of w_body calculations
  w_body = lin::zeros<decltype(w_body)>();
  for (unsigned int i = 0; i < state.w_buffer.size(); i++)
    w_body = w_body + state.w_buffer[i];
  w_body = w_body / static_cast<float>(state.w_buffer.size());

  // Update the estimate
  state.t = data.t;
  state.q_body_eci = q_body_eci;
  estimate.w_body = w_body;

}
}  // namespace gnc

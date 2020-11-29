//
// MIT License
//
// Copyright (c) 2020 Pathfinder for Autonomous Navigation (PAN)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

/** @file psim/fc/attitude_estimator.cpp
 *  @author Kyle Krol
 */

#include <psim/fc/attitude_estimator.hpp>

#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>
#include <lin/references.hpp>

namespace psim {

void AttitudeEstimator::_set_attitude_outputs() {
  fc_satellite_attitude_is_valid.get() = _attitude_estimate.is_valid;
  fc_satellite_attitude_q_body_eci.get() = _attitude_estimate.q_body_eci;
  fc_satellite_attitude_w_bias.get() = _attitude_estimate.gyro_bias;
}

void AttitudeEstimator::add_fields(State &state) {
  this->Super::add_fields(state);

  // This ensures upon simulation construction the state fields hold proper
  // values.
  _set_attitude_outputs();
}

void AttitudeEstimator::step() {
  this->Super::step();

  auto const &t = truth_t_s->get();
  auto const &r = truth_satellite_orbit_r_ecef->get();
  auto const &b = sensors_satellite_magnetometer_b->get();
  auto const &s = sensors_satellite_sun_sensors_s->get();
  auto const &w = sensors_satellite_gyroscope_w->get();

  // If the current estimate is already valid, update it.
  if (_attitude_state.is_valid) {
    _attitude_data = gnc::AttitudeEstimatorData();
    _attitude_data.t = t;
    _attitude_data.r_ecef = r;
    _attitude_data.b_body = b;
    _attitude_data.s_body = s;
    _attitude_data.w_body = w;

    _attitude_estimate = gnc::AttitudeEstimate();
    gnc::attitude_estimator_update(
        _attitude_state, _attitude_data, _attitude_estimate);
  }
  // Attempt to reset the current estimate if it isn't valid.
  else {
    if (lin::all(lin::isfinite(t)) && lin::all(lin::isfinite(r)) &&
        lin::all(lin::isfinite(b)) && lin::all(lin::isfinite(s)))
      gnc::attitude_estimator_reset(_attitude_state, t, r, b, s);
  }

  _set_attitude_outputs();
}

Vector4 AttitudeEstimator::fc_satellite_attitude_q_body_eci_error() const {
  auto const &truth_q_eci_body = truth_satellite_attitude_q_eci_body->get();
  auto const &q_body_eci = Super::fc_satellite_attitude_q_body_eci.get();

  Vector4 q_error;
  gnc::utl::quat_cross_mult(q_body_eci, truth_q_eci_body, q_error);
  return q_error;
}

Real AttitudeEstimator::fc_satellite_attitude_q_body_eci_error_degrees() const {
  auto const &q_error = Super::fc_satellite_attitude_q_body_eci_error.get();

  return 2.0 * lin::asin(lin::norm(lin::ref<3, 1>(q_error, 0, 0)));
}

Vector3 AttitudeEstimator::fc_satellite_attitude_p_body_eci_error() const {
  auto const &q_error = Super::fc_satellite_attitude_q_body_eci_error.get();

  constexpr static Real a = 1.0;
  constexpr static Real f = 2.0 * (a + 1.0);

  Vector3 p_error;
  gnc::utl::quat_to_qrp(q_error, a, f, p_error);
  return p_error;
}

Vector3 AttitudeEstimator::fc_satellite_attitude_p_body_eci_sigma() const {
  auto const &P = _attitude_state.P;

  return {lin::sqrt(P(0, 0)), lin::sqrt(P(1, 1)), lin::sqrt(P(2, 2))};
}

Vector3 AttitudeEstimator::fc_satellite_attitude_w_bias_error() const {
  auto const &truth_bias = sensors_satellite_gyroscope_w_bias->get();
  auto const &bias = fc_satellite_attitude_w_bias.get();

  return truth_bias - bias;
}

Vector3 AttitudeEstimator::fc_satellite_attitude_w_bias_sigma() const {
  auto const &P = _attitude_state.P;

  return {lin::sqrt(P(3, 3)), lin::sqrt(P(4, 4)), lin::sqrt(P(5, 5))};
}
} // namespace psim

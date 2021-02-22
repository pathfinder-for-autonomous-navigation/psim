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

/** @file psim/fc/relative_orbit_estimator.cpp
 *  @author Kyle Krol
 */

#include <psim/fc/relative_orbit_estimator.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>
#include <lin/references.hpp>

namespace psim {

void RelativeOrbitEstimator::_set_relative_orbit_outputs() {
  fc_satellite_relative_orbit_is_valid.get() = estimate.valid();
  fc_satellite_relative_orbit_dr.get() = estimate.dr_ecef();
  fc_satellite_relative_orbit_r_hill.get() = estimate.r_hill();
  fc_satellite_relative_orbit_r_hill_sigma.get() =
      lin::ref<Vector3>(lin::diag(estimate.S()), 0, 0);
  fc_satellite_relative_orbit_dv.get() = estimate.dv_ecef();
  fc_satellite_relative_orbit_v_hill.get() = estimate.v_hill();
  fc_satellite_relative_orbit_v_hill_sigma.get() =
      lin::ref<Vector3>(lin::diag(estimate.S()), 3, 0);
}

void RelativeOrbitEstimator::add_fields(State &state) {
  this->Super::add_fields(state);

  // This ensures upon simulation construction the state fields hold proper
  // values.
  _set_relative_orbit_outputs();
}

void RelativeOrbitEstimator::step() {
  this->Super::step();

  // Process noise
  static constexpr auto sqrtQ =
      lin::diag(Vector<6>({1.0e-8, 1.0e-8, 1.0e-8, 1.0e-4, 1.0e-4, 1.0e-2}))
          .eval();

  // Sensor noise
  static constexpr auto sqrtR =
      lin::diag(lin::consts<Vector<3>>(1.0e-1)).eval();

  auto const &dt = truth_dt_ns->get();
  auto const &w_earth = truth_earth_w->get();
  auto const &r_ecef = truth_satellite_orbit_r_ecef->get();
  auto const &v_ecef = truth_satellite_orbit_v_ecef->get();
  auto const &cdgps_dr = sensors_satellite_cdgps_dr->get();

  // Handle when the estimate is already valid
  if (estimate.valid()) {
    // Predict and update
    if (lin::all(lin::isfinite(cdgps_dr))) {
      estimate.update(dt, w_earth, r_ecef, v_ecef, -cdgps_dr, sqrtQ, sqrtR);
    }
    // No measurement so we just predict
    else {
      estimate.update(dt, w_earth, r_ecef, v_ecef, sqrtQ);
    }
  }
  // Attempt to initialize the estimate
  else {
    // Initialize the velocity with a finite difference
    if (lin::all(lin::isfinite(previous_dr))) {
      Vector3 cdgps_dv = 1.0e9 * (cdgps_dr - previous_dr) / Real(dt);

      Matrix<6, 6> S;
      lin::ref<Matrix<3, 3>>(S, 0, 0) = sqrtR;
      lin::ref<Matrix<3, 3>>(S, 3, 3) = lin::sqrt(2.0e9 / Real(dt)) * sqrtR;

      estimate = gnc::RelativeOrbitEstimate(
          w_earth, r_ecef, v_ecef, -cdgps_dr, -cdgps_dv, S);

      previous_dr = lin::nans<Vector3>();
    }
    // Cache a position reading
    else {
      previous_dr = cdgps_dr;
    }
  }

  _set_relative_orbit_outputs();
}

Vector3 RelativeOrbitEstimator::fc_satellite_relative_orbit_dr_error() const {
  auto const &fc_dr = fc_satellite_relative_orbit_dr.get();
  auto const &truth_r_ecef = truth_satellite_orbit_r_ecef->get();
  auto const &truth_other_r_ecef = truth_other_orbit_r_ecef->get();

  return fc_dr - (truth_other_r_ecef - truth_r_ecef);
}

Vector3
RelativeOrbitEstimator::fc_satellite_relative_orbit_r_hill_error() const {
  auto const &fc_r_hill = fc_satellite_relative_orbit_r_hill.get();
  auto const &truth_r_hill = truth_satellite_hill_dr->get();

  return fc_r_hill - truth_r_hill;
}

Vector3 RelativeOrbitEstimator::fc_satellite_relative_orbit_dv_error() const {
  auto const &fc_dv = fc_satellite_relative_orbit_dv.get();
  auto const &truth_v_ecef = truth_satellite_orbit_v_ecef->get();
  auto const &truth_other_v_ecef = truth_other_orbit_v_ecef->get();

  return fc_dv - (truth_other_v_ecef - truth_v_ecef);
}

Vector3
RelativeOrbitEstimator::fc_satellite_relative_orbit_v_hill_error() const {
  auto const &fc_v_hill = fc_satellite_relative_orbit_v_hill.get();
  auto const &truth_v_hill = truth_satellite_hill_dv->get();

  return fc_v_hill - truth_v_hill;
}
} // namespace psim

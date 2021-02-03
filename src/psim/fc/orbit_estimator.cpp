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

/** @file psim/fc/orbit_estimator.cpp
 *  @author Kyle Krol
 */

#include <psim/fc/orbit_estimator.hpp>

#include <gnc/environment.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>

namespace psim {

void OrbOrbitEstimator::_set_orbit_outputs() {
  fc_satellite_orbit_is_valid.get() = estimate.valid();
  fc_satellite_orbit_r.get() = estimate.recef();
  fc_satellite_orbit_v.get() = estimate.vecef();
}

void OrbOrbitEstimator::add_fields(State &state) {
  this->Super::add_fields(state);

  // This ensures upon simulation construction the state fields hold proper
  // values.
  _set_orbit_outputs();
}

void OrbOrbitEstimator::step() {
  this->Super::step();

  constexpr auto sqrtQ = lin::diag(lin::consts<Vector<6>>(0.1)).eval();
  constexpr auto sqrtR = lin::diag(lin::consts<Vector<6>>(5.0)).eval();

  auto const &t = truth_t_s->get();
  auto const &dt = truth_dt_ns->get();
  auto const &r = sensors_satellite_gps_r->get();
  auto const &v = sensors_satellite_gps_v->get();

  Vector3 w;
  gnc::env::earth_angular_rate(t, w);

  if (estimate.valid()) {
    double _;
    if (lin::all(lin::isfinite(r)) && lin::all(lin::isfinite(v)))
      estimate.shortupdate(dt, w, r, v, sqrtQ, sqrtR, _);
    else
      estimate.shortupdate(dt, w, sqrtQ, _);
  }
  else {
    if (lin::all(lin::isfinite(r)) && lin::all(lin::isfinite(v)))
      estimate = orb::OrbitEstimate(orb::MINGPSTIME_NS, r, v, sqrtR);
  }

  _set_orbit_outputs();
}

Vector3 OrbOrbitEstimator::fc_satellite_orbit_r_error() const {
  auto const &r = Super::fc_satellite_orbit_r.get();
  auto const &truth_r = truth_satellite_orbit_r_ecef->get();

  return r - truth_r;
}

Vector3 OrbOrbitEstimator::fc_satellite_orbit_r_sigma() const {
  return lin::ref<Vector3>(lin::diag(estimate.S()), 0, 0);
}

Vector3 OrbOrbitEstimator::fc_satellite_orbit_v_error() const {
  auto const &v = Super::fc_satellite_orbit_v.get();
  auto const &truth_v = truth_satellite_orbit_v_ecef->get();

  return v - truth_v;
}

Vector3 OrbOrbitEstimator::fc_satellite_orbit_v_sigma() const {
  return lin::ref<Vector3>(lin::diag(estimate.S()), 3, 0);
}
} // namespace psim

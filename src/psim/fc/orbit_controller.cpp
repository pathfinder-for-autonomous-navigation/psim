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

/** @file psim/fc/orbit_controller.cpp
 *  @author Govind Chari
 */

#include <psim/fc/orbit_controller.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>

namespace psim {

void OrbitController::add_fields(State &state) {
  this->Super::add_fields(state);
  fc_satellite_cumulative_dv.get() = 0;
}

void OrbitController::step() {
  this->Super::step();

  auto const &t = truth_t_s->get();
  auto const &t_ns = truth_t_ns->get();

  auto &J_ecef = truth_satellite_orbit_J_ecef->get();
  auto const &m = truth_satellite_m.get();
  auto &cumulative_dv = fc_satellite_cumulative_dv.get();
  auto const &relative_orbit_is_valid = fc_satellite_relative_orbit_is_valid->get();
  auto const &r_ecef = fc_satellite_orbit_r->get();
  auto const &v_ecef = fc_satellite_orbit_v->get();
  auto const &other_r_ecef = truth_other_orbit_r_ecef->get();
  auto const &other_v_ecef = truth_other_orbit_v_ecef->get();
  auto const &fire_time = fc_satellite_fire_time.get();

  Vector3 dr_ecef;
  Vector3 dv_ecef;

  if (relative_orbit_is_valid) {
    dr_ecef = fc_satellite_relative_orbit_dr->get();
    dv_ecef = fc_satellite_relative_orbit_dv->get();
  } else {
    dr_ecef = (other_r_ecef - r_ecef);
    dv_ecef = (other_v_ecef - v_ecef);
  }

  if (t_ns > last_firing + (fire_time * 1e9)) {
    last_firing = t_ns;
    gnc::OrbitControllerData data;
    data.t = t;
    data.r_ecef = r_ecef;
    data.v_ecef = v_ecef;
    data.dr_ecef = dr_ecef;
    data.dv_ecef = dv_ecef;

    gnc::OrbitActuation actuation;
    gnc::control_orbit(_orbit_controller, data, actuation);

    if (lin::all(lin::isfinite(actuation.J_ecef))) {
      J_ecef = actuation.J_ecef;
      cumulative_dv = cumulative_dv + lin::norm(J_ecef) / m;
    } else {
      J_ecef = lin::zeros<Vector3>();
    }
  }
}
} // namespace psim

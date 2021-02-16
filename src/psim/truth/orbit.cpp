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

/** @file psim/truth/orbit.cpp
 *  @author Kyle Krol
 */

#include <psim/truth/orbit.hpp>

#include <gnc/config.hpp>
#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/math.hpp>
#include <lin/references.hpp>

#include <psim/truth/orbit_utilities.hpp>

namespace psim {

OrbitEcef::OrbitEcef(RandomsGenerator &randoms, Configuration const &config,
    std::string const &satellite)
  : Super(randoms, config, satellite, "ecef") {}

void OrbitEcef::step() {
  this->Super::step();

  struct IntegratorData {
    Real const &A;
    Real const &m;
    Vector3 const &earth_w;
    Vector3 const &earth_w_dot;
  };

  auto const &dt = truth_dt_s->get();
  auto const &earth_w = truth_earth_w->get();
  auto const &earth_w_dot = truth_earth_w_dot->get();
  auto const &A = truth_satellite_A.get();
  auto const &m = truth_satellite_m.get();

  auto &r = truth_satellite_orbit_r.get();
  auto &v = truth_satellite_orbit_v.get();
  auto &J = truth_satellite_orbit_J_frame.get();

  // Thruster firings are modelled here as instantaneous impulses. This removes
  // thruster dependance from the state dot function in the integrator.
  v = v + J / m;
  J = lin::zeros<Vector3>();

  // Prepare integrator inputs.
  Vector<6> x = {r(0), r(1), r(2), v(0), v(1), v(2)};
  IntegratorData data = {A, m, earth_w, earth_w_dot};

  // Simulate dynamics.
  x = ode(Real(0.0), dt, x, &data,
      [](Real t, Vector<6> const &x, void *ptr) -> Vector<6> {
        auto const *data = static_cast<IntegratorData *>(ptr);

        // References to our position and velocity in ECEF.
        auto const r = lin::ref<Vector3>(x, 0, 0);
        auto const v = lin::ref<Vector3>(x, 3, 0);

        // Interpolate Earth's angular rate.
        Vector3 w = data->earth_w + t * data->earth_w_dot;
        auto const &w_dot = data->earth_w_dot;

        // Calculate gravitational acceleration.
        Vector3 g;
        orbit::gravity(r.eval(), g);

        // Calculate drag force
        Vector3 F_drag;
        orbit::drag(r.eval(), v.eval(), data->A, F_drag);

        // Calculate total acceleration
        //
        // Reference(s):
        //  - https://en.wikipedia.org/wiki/Rotating_reference_frame
        Vector3 a = g + F_drag / data->m - Real(2.0) * lin::cross(w, v) -
                    lin::cross(w, lin::cross(w, r)) - lin::cross(w_dot, r);

        return {v(0), v(1), v(2), a(0), a(1), a(2)};
      });

  // Write back to our state fields
  r = lin::ref<Vector3>(x, 0, 0);
  v = lin::ref<Vector3>(x, 3, 0);
}

Real OrbitEcef::truth_satellite_orbit_T() const {
  auto const &earth_w = truth_earth_w->get();
  auto const &r = truth_satellite_orbit_r.get();
  auto const &v = truth_satellite_orbit_v.get();
  auto const &m = truth_satellite_m.get();

  return 0.5 * m * lin::fro(v + lin::cross(earth_w, r));
}

Real OrbitEcef::truth_satellite_orbit_U() const {
  auto const &r = truth_satellite_orbit_r.get();
  auto const &m = truth_satellite_m.get();

  Real U;
  Vector3 _;
  orbit::gravity(r, _, U);

  return m * U;
}

Real OrbitEcef::truth_satellite_orbit_E() const {
  auto const &T = this->Super::truth_satellite_orbit_T.get();
  auto const &U = this->Super::truth_satellite_orbit_U.get();

  return T - U;
}
} // namespace psim

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

#include <gnc/environment.hpp>
#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/generators/constants.hpp>
#include <lin/math.hpp>
#include <lin/references.hpp>

namespace psim {

OrbitGncEci::OrbitGncEci(Configuration const &config,
    std::string const &satellite)
  : Super(config, satellite, "eci") { }

void OrbitGncEci::step() {
  this->Super::step();

  // References to the current time and timestep
  auto const &dt = truth_dt_s->get();
  auto const &t = truth_t_s->get();

  // References to position and velocity
  auto &r = truth_satellite_orbit_r.get();
  auto &v = truth_satellite_orbit_v.get();

  // Treat our thruster firings as purely impulses
  v = v + truth_satellite_orbit_J_frame.get() / truth_satellite_m.get();
  truth_satellite_orbit_J_frame.get() = lin::zeros<Vector3>();

  // Simulate our dynamics
  auto const xf = ode(t, dt, {r(0), r(1), r(2), v(0), v(1), v(2)}, nullptr,
      [](Real t, lin::Vector<Real, 6> const &x, void *) -> lin::Vector<Real, 6> {
    // References to our position and velocity in ECI
    auto const r = lin::ref<3, 1>(x, 0, 0);
    auto const v = lin::ref<3, 1>(x, 3, 0);

    // Calculate the Earth's current attitude
    Vector4 q;
    gnc::env::earth_attitude(t, q);  // q = q_ecef_eci

    // Determine our gravitation acceleration in ECI
    Vector3 g;
    {
      Vector3 r_ecef;
      gnc::utl::rotate_frame(q, r.eval(), r_ecef);

      Real _;
      gnc::env::gravity(r_ecef, g, _);  // g = g_ecef
      gnc::utl::quat_conj(q);           // q = q_eci_ecef
      gnc::utl::rotate_frame(q, g);     // g = g_eci
    }

    return {v(0), v(1), v(2), g(0), g(1), g(2)};
  });

  // Write back to our state fields
  r = lin::ref<3, 1>(xf, 0, 0);
  v = lin::ref<3, 1>(xf, 3, 0);
}
}  // namespace psim

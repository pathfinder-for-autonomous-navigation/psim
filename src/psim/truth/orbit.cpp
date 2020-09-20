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

#include <gnc/constants.hpp>

#include <lin/core.hpp>
#include <lin/generators/constants.hpp>
#include <lin/math.hpp>
#include <lin/references.hpp>

namespace psim {

OrbitKeplarianEci::OrbitKeplarianEci(Configuration const &config,
    std::string const &prefix, std::string const &satellite)
  : Super(config, prefix, satellite, "eci") { }

void OrbitKeplarianEci::step() {
  this->Super::step();

  // References to the current time and timestep
  auto const &dt = prefix_dt_s->get();
  auto const &t = prefix_t_s->get();

  // References to position and velocity
  auto &r = prefix_satellite_orbit_r.get();
  auto &v = prefix_satellite_orbit_v.get();

  // Apply and impulse if provided
  v = v + prefix_satellite_orbit_J_frame.get() / prefix_satellite_m.get();
  prefix_satellite_orbit_J_frame.get() = lin::zeros<Vector3>();

  // Simulate our dynamics
  auto const xf = ode4(t, dt, {r(0), r(1), r(2), v(0), v(1), v(2)}, nullptr,
      [](Real t, lin::Vector<Real, 6> const &x, void *) -> lin::Vector<Real, 6> {
    // References to our position and velocity
    auto const r = lin::ref<3, 1>(x, 0, 0);
    auto const v = lin::ref<3, 1>(x, 3, 0);

    // Acceleration calculation
    auto const fro_r = lin::fro(r);
    auto const mag_r = lin::sqrt(fro_r);
    auto const a = ((gnc::constant::mu_earth / (fro_r * mag_r)) * -r).eval();

    return {v(0), v(1), v(2), a(0), a(1), a(2)};
  });

  // Write back to our state fields
  r = lin::ref<3, 1>(xf, 0, 0);
  v = lin::ref<3, 1>(xf, 3, 0);
}
}  // namespace psim

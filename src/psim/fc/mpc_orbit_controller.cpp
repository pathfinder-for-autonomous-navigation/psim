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

/** @file psim/fc/mpc_orbit_controller.hpp
 *  @author Kyle Krol
 */

#include <psim/fc/mpc_orbit_controller.hpp>

#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>

namespace psim {

void MpcOrbitController::add_fields(State &state) {
  this->Super::add_fields(state);

  fc_satellite_orbit_J_hill.get() = lin::zeros<Vector3>();
}

void MpcOrbitController::step() {
  auto const &q_ecef_eci = truth_earth_q_ecef_eci->get();
  auto const &q_eci_hill = truth_satellite_hill_q_hill_eci->get();

  auto &J_hill = fc_satellite_orbit_J_hill.get();
  auto &J_ecef = truth_satellite_orbit_J_ecef->get();

  gnc::utl::rotate_frame(q_eci_hill, J_hill);
  gnc::utl::rotate_frame(q_ecef_eci, J_hill, J_ecef);

  J_hill = lin::zeros<Vector3>();
}
} // namespace psim

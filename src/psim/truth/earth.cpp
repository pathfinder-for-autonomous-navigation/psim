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

/** @file psim/truth/earth.cpp
 *  @author Kyle Krol
 */

#include <psim/truth/earth.hpp>

#include <gnc/environment.hpp>
#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>

namespace psim {

Vector4 EarthGnc::truth_earth_q_ecef_eci() const {
  auto const &t = truth_t_s->get();

  Vector4 q_ecef_eci;
  gnc::env::earth_attitude(t, q_ecef_eci);
  return q_ecef_eci;
}

Vector4 EarthGnc::truth_earth_q_eci_ecef() const {
  auto const &q_ecef_eci = this->Super::truth_earth_q_ecef_eci.get();

  Vector4 q_eci_ecef;
  gnc::utl::quat_conj(q_ecef_eci, q_eci_ecef);
  return q_eci_ecef;
}

Vector3 EarthGnc::truth_earth_w() const {
  auto const &t = truth_t_s->get();

  Vector3 w_earth;
  gnc::env::earth_angular_rate(t, w_earth);
  return w_earth;
}

Vector3 EarthGnc::truth_earth_w_dot() const {
  return lin::zeros<Vector3>();
}
}  // namespace psim

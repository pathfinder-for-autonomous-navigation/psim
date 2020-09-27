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

/** @file psim/truth/environment.cpp
 *  @author Kyle Krol
 */

#include <psim/truth/environment.hpp>

#include <gnc/environment.hpp>

namespace psim {

EnvironmentGnc::EnvironmentGnc(Configuration const &config,
    std::string const &prefix, std::string const &satellite)
  : Super(config, prefix, satellite, "ecef") { }


Vector3 EnvironmentGnc::prefix_satellite_environment_b() const {
  auto const &r_ecef = prefix_satellite_orbit_r_frame->get();
  auto const &t = prefix_t_s->get();

  Vector3 b_ecef;
  gnc::env::magnetic_field(t, r_ecef, b_ecef);
  return b_ecef;
}

Vector3 EnvironmentGnc::prefix_satellite_environment_s() const {
  auto const &t = prefix_t_s->get();

  Vector3 s_eci;
  gnc::env::sun_vector(t, s_eci);
  return s_eci;
}
}  // namespace psim

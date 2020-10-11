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

/** @file psim/simulation/dual_orbit.cpp
 *  @author Kyle Krol
 */

#include <psim/simulations/dual_orbit.hpp>

#include <psim/truth/earth.hpp>
#include <psim/truth/environment.hpp>
#include <psim/truth/orbit.hpp>
#include <psim/truth/time.hpp>
#include <psim/truth/transform_position.hpp>
#include <psim/truth/transform_velocity.hpp>

#include <string>

namespace psim {

DualOrbitGnc::DualOrbitGnc(Configuration const &config) {
  auto const add_satellite = [this, &config](std::string const &satellite) -> void {
    // Orbital dynamics
    this->add<OrbitGncEci>(config, "truth", satellite);
    this->add<TransformPositionEci>(config, "truth", "truth." + satellite + ".orbit.r");
    this->add<TransformVelocityEci>(config, "truth", satellite, "truth." + satellite + ".orbit.v");
    // Environmental models
    this->add<EnvironmentGnc>(config, "truth", satellite);
    this->add<TransformPositionEcef>(config, "truth", "truth." + satellite + ".environment.b");
    this->add<TransformPositionEci>(config, "truth", "truth." + satellite + ".environment.s");
  };
  // Time and Earth ephemeris
  add<Time>(config, "truth");
  add<EarthGnc>(config, "truth");
  // Leader and follower satellites
  add_satellite("leader");
  add_satellite("follower");
}
}  // namespace psim

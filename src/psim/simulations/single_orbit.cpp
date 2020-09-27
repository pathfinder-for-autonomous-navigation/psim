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

/** @file psim/simulation/single_orbit.cpp
 *  @author Kyle Krol
 */

#include <psim/simulations/single_orbit.hpp>

#include <psim/truth/earth.hpp>
#include <psim/truth/environment.hpp>
#include <psim/truth/orbit.hpp>
#include <psim/truth/time.hpp>
#include <psim/truth/transform_position.hpp>
#include <psim/truth/transform_velocity.hpp>

namespace psim {

SingleOrbitGnc::SingleOrbitGnc(Configuration const &config) {
  // Time and Earth ephemeris
  add<Time>(config, "truth");
  add<EarthGnc>(config, "truth");
  // Orbital dynamics and lazy transformations
  add<OrbitGncEci>(config, "truth", "leader");
  add<TransformPositionEci>(config, "truth", "truth.leader.orbit.r");
  add<TransformVelocityEci>(config, "truth", "leader", "truth.leader.orbit.v");
  // Environmental variables with body frame independent lazy transformations
  add<EnvironmentGnc>(config, "truth", "leader");
}
}  // namespace psim

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

#include <psim/truth/satellite_orbit.hpp>

#include <psim/truth/environment.hpp>
#include <psim/truth/orbit.hpp>
#include <psim/truth/transform_position.hpp>
#include <psim/truth/transform_velocity.hpp>

namespace psim {

SatelliteOrbitGnc::SatelliteOrbitGnc(Configuration const &config,
    std::string const &prefix, std::string const &satellite) {
  // Orbital dynamics
  add<OrbitGncEci>(config, prefix, satellite);
  add<TransformPositionEci>(config, prefix, prefix + "." + satellite + ".orbit.r");
  add<TransformVelocityEci>(config, prefix, satellite, prefix + "." + satellite + ".orbit.v");
  // Environmental models
  add<EnvironmentGnc>(config, prefix, satellite);
  add<TransformPositionEcef>(config, prefix, prefix + "." + satellite + ".environment.b");
  add<TransformPositionEci>(config, prefix, prefix + "." + satellite + ".environment.s");
}
}  // namespace psim

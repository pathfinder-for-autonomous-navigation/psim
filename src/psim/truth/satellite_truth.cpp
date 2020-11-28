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

/** @file psim/truth/satellite_truth.cpp
 *  @author Kyle Krol
 */

#include <psim/truth/satellite_truth.hpp>

#include <psim/truth/attitude_orbit.hpp>
#include <psim/truth/environment.hpp>
#include <psim/truth/orbit.hpp>
#include <psim/truth/transform_direction.hpp>
#include <psim/truth/transform_position.hpp>
#include <psim/truth/transform_velocity.hpp>
#include <psim/utilities/norm_vector3.hpp>
#include <psim/utilities/norm_vector4.hpp>

namespace psim {

SatelliteTruthGnc::SatelliteTruthGnc(RandomsGenerator &randoms,
    Configuration const &config, std::string const &satellite)
  : ModelList(randoms) {
  // Orbital dynamics
  add<AttitudeOrbitNoFuelEciGnc>(randoms, config, satellite);
  add<TransformPositionEci>(randoms, config, "truth." + satellite + ".orbit.r");
  add<TransformVelocityEci>(randoms, config, satellite, "truth." + satellite + ".orbit.v");
  // Extra telemetry
  add<NormVector3>(randoms, config, "truth." + satellite + ".attitude.L");
  add<NormVector4>(randoms, config, "truth." + satellite + ".attitude.q.body_eci");
  // Environmental models
  add<EnvironmentGnc>(randoms, config, satellite);
  add<TransformDirectionEcef>(randoms, config, satellite, "truth." + satellite + ".environment.b");
  add<TransformDirectionEci>(randoms, config, satellite, "truth." + satellite + ".environment.s");
}

SatelliteTruthNoAttitudeGnc::SatelliteTruthNoAttitudeGnc(
    RandomsGenerator &randoms,  Configuration const &config,
    std::string const &satellite)
  : ModelList(randoms) {
  // Orbital dynamics
  add<OrbitGncEci>(randoms, config, satellite);
  add<TransformPositionEci>(randoms, config, "truth." + satellite + ".orbit.r");
  add<TransformVelocityEci>(randoms, config, satellite, "truth." + satellite + ".orbit.v");
  // Environmental models
  add<EnvironmentGnc>(randoms, config, satellite);
  add<TransformPositionEcef>(randoms, config, "truth." + satellite + ".environment.b");
  add<TransformPositionEci>(randoms, config, "truth." + satellite + ".environment.s");
}
}  // namespace psim

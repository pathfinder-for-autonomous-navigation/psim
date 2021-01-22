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

/** @file psim/simulations/single_attitude_orbit.cpp
 *  @author Kyle Krol
 */

#include <psim/simulations/single_attitude_orbit.hpp>

#include <psim/fc/satellite_fc.hpp>
#include <psim/sensors/satellite_sensors.hpp>
#include <psim/truth/earth.hpp>
#include <psim/truth/satellite_truth.hpp>
#include <psim/truth/time.hpp>

namespace psim {

SingleAttitudeOrbitGnc::SingleAttitudeOrbitGnc(
    RandomsGenerator &randoms, Configuration const &config)
  : ModelList(randoms) {
  // Truth model
  add<Time>(randoms, config);
  add<EarthGnc>(randoms, config);
  add<SatelliteTruthGnc>(randoms, config, "leader");
  // Sensors model
  add<SatelliteSensors>(randoms, config, "leader");
}

SingleAttitudeOrbitGncFc::SingleAttitudeOrbitGncFc(
    RandomsGenerator &randoms, Configuration const &config)
  : ModelList(randoms) {
    // Simulation
    add<SingleAttitudeOrbitGnc>(randoms, config);
    // Flight computer model
    add<SatelliteFc>(randoms, config, "leader");
  }
} // namespace psim

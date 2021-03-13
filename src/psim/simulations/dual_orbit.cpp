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

/** @file psim/simulations/dual_orbit.cpp
 *  @author Kyle Krol
 */

#include <psim/simulations/dual_orbit.hpp>

#include <psim/sensors/cdgps_no_attitude.hpp>
#include <psim/sensors/satellite_sensors.hpp>
#include <psim/truth/earth.hpp>
#include <psim/truth/hill_frame.hpp>
#include <psim/truth/satellite_truth.hpp>
#include <psim/truth/time.hpp>
#include <psim/utilities/norm_vector3.hpp>

namespace psim {

DualOrbitGnc::DualOrbitGnc(
    RandomsGenerator &randoms, Configuration const &config)
  : ModelList(randoms) {
  // Truth model
  add<Time>(randoms, config);
  add<EarthGnc>(randoms, config);
  add<SatelliteTruthNoAttitudeGnc>(randoms, config, "leader");
  add<SatelliteTruthNoAttitudeGnc>(randoms, config, "follower");
  add<HillFrameEci>(randoms, config, "leader", "follower");
  add<HillFrameEci>(randoms, config, "follower", "leader");
  add<NormVector3>(randoms, config, "truth.leader.hill.dr");
  add<NormVector3>(randoms, config, "truth.leader.hill.dv");
  // Sensors model
  add<SatelliteSensorsNoAttitude>(randoms, config, "leader");
  add<SatelliteSensorsNoAttitude>(randoms, config, "follower");
  add<CdgpsNoAttitude>(randoms, config, "leader", "follower");
  add<CdgpsNoAttitude>(randoms, config, "follower", "leader");
}
} // namespace psim

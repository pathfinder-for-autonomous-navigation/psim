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

/** @file psim/simulation/orbit_mpc_rendezvous.cpp
 *  @author Kyle Krol
 */

#include <psim/simulations/orbit_mpc_rendezvous.hpp>

#include <psim/fc/mpc_orbit_controller.hpp>
#include <psim/fc/orbit_estimator.hpp>
#include <psim/fc/relative_orbit_estimator.hpp>
#include <psim/simulations/dual_orbit.hpp>

namespace psim {

OrbitMpcRendezvous::OrbitMpcRendezvous(
    RandomsGenerator &randoms, Configuration const &config)
  : ModelList(randoms) {
  add<MpcOrbitController>(randoms, config, "follower");
  add<DualOrbitGnc>(randoms, config);
  add<OrbOrbitEstimator>(randoms, config, "follower");
  add<RelativeOrbitEstimator>(randoms, config, "follower", "leader");
}
} // namespace psim

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

/** @file psim/sensors/cdgps_no_attitude.cpp
 *  @author Kyle Krol
 */

#include <psim/sensors/cdgps_no_attitude.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>

namespace psim {

Integer CdgpsNoAttitude::sensors_satellite_cdgps_active() const {
  auto const &model_range = sensors_satellite_cdgps_model_range.get();

  /* If model range if requested. Otherwise, the sensor will be active all the
   * time.
   */
  if (!model_range) {
    return true;
  }

  auto const &range = sensors_satellite_cdgps_range.get();
  auto const &truth_r_ecef = truth_satellite_orbit_r_ecef->get();
  auto const &truth_other_r_ecef = truth_other_orbit_r_ecef->get();

  /* The sensor will be active if within range for this timestep. We're using
   * lin::fro instaed of lin::norm here to avoid a square root.
   */
  return lin::fro(truth_other_r_ecef - truth_r_ecef) < (range * range);
}

Vector3 CdgpsNoAttitude::sensors_satellite_cdgps_dr() const {
  auto const &truth_r_ecef = truth_satellite_orbit_r_ecef->get();
  auto const &truth_other_r_ecef = truth_other_orbit_r_ecef->get();
  auto const &error = Super::sensors_satellite_cdgps_dr_error.get();

  /** If the sensor isn't active, the NaNs from the error field will make this
   *  calculation results in NaNs as well as desired.
   */
  return (truth_other_r_ecef - truth_r_ecef) + error;
}

Vector3 CdgpsNoAttitude::sensors_satellite_cdgps_dr_error() const {
  auto const &active = Super::sensors_satellite_cdgps_active.get();

  /* Only calculate an output if the sensor is active.
   */
  if (active) {
    auto const &sigma = sensors_satellite_cdgps_dr_sigma.get();

    return lin::multiply(sigma, lin::gaussians<Vector3>(_randoms));
  }

  return lin::nans<Vector3>();
}
}  // namespace psim

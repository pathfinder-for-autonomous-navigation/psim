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

/** @file psim/sensors/sun_sensors.cpp
 *  @author Kyle Krol
 */

#include <psim/sensors/sun_sensors.hpp>

#include <gnc/constants.hpp>
#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/math.hpp>

namespace psim {

Vector3 SunSensors::sensors_satellite_sun_sensors_s() const {
  /* Don't generate a sun vector measurement if in eclipse and the model is
   * enabled.
   *
   * We consider ourselves to be in eclipse if the dot product of our position
   * vector and the sun vector, both in ECI, is negative.
   */
  auto const &model_eclipse = sensors_satellite_sun_sensors_model_eclipse.get();

  if (model_eclipse) {
    auto const &truth_r_eci = truth_satellite_orbit_r_eci->get();
    auto const &truth_s_eci = truth_satellite_environment_s_eci->get();

    if (lin::dot(truth_s_eci, truth_r_eci) < 0.0) {
      return lin::nans<Vector3>();
    }
  }

  /* Currently, we're using noise representation in spherical coordinates.
   * This is alright for now but really we should update this model to simulate
   * each photodiode and the sun vector calculation performed on the ADCS
   * computer.
   *
   * TODO: Update the sun sensor model to include individual diodes.
   */
  auto const &truth_s_body = truth_satellite_environment_s_body->get();
  auto const &sigma = sensors_satellite_sun_sensors_s_sigma.get();

  /* 1. Generate a quaternion transform from the x axis to the true sun vector
   *    in the body frame.
   */
  Vector4 q;
  gnc::utl::vec_rot_to_quat(truth_s_body, Vector3({1.0, 0.0, 0.0}), q);

  /* 2. Generate sensors noise values in spherical coordinates centered about
   *    the x axis.
   */
  auto const phi = sigma(0) * _randoms.gaussian();
  auto const theta = gnc::constant::pi / 2.0 + sigma(1) * _randoms.gaussian();

  /* 3. Reconstruct the measured sun vector relative to the x axis (which is the
   *    true sun vector given step 1).
   */
  Vector3 s;
  s(0) = lin::sin(theta) * lin::cos(phi);
  s(1) = lin::sin(theta) * lin::sin(phi);
  s(2) = lin::cos(theta);

  /* 4. Rotate the measured sun vector back into the body frame.
   */
  gnc::utl::rotate_frame(q, s);
  return s;
}

Vector3 SunSensors::sensors_satellite_sun_sensors_s_error() const {
  auto const &truth_s = truth_satellite_environment_s_body->get();
  auto const &s = Super::sensors_satellite_sun_sensors_s.get();

  return s - truth_s;
}
} // namespace psim

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

/** @file psim/sensors/magnetometer.cpp
 *  @author Kyle Krol
 */

#include <psim/sensors/magnetometer.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>

namespace psim {

Boolean Magnetometer::sensors_satellite_magnetometer_valid() const {
  auto const &disabled = sensors_satellite_magnetometer_disabled.get();

  return !disabled;
}

Vector3 Magnetometer::sensors_satellite_magnetometer_b() const {
  auto const &error = Super::sensors_satellite_magnetometer_b_error.get();
  auto const &truth_b = truth_satellite_environment_b_body->get();

  return truth_b + error;
}

Vector3 Magnetometer::sensors_satellite_magnetometer_b_error() const {
  auto const &valid = Super::sensors_satellite_magnetometer_valid.get();
  auto const &sigma = sensors_satellite_magnetometer_b_sigma.get();

  if (valid)
    return lin::multiply(sigma, lin::gaussians<Vector3>(_randoms));
  else
    return lin::nans<Vector3>();
}
}  // namespace psim

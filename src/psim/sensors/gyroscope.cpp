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

/** @file psim/sensors/gyroscope.cpp
 *  @author Kyle Krol
 */

#include <psim/sensors/gyroscope.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>

namespace psim {

void Gyroscope::step() {
  this->Super::step();

  auto const &bias_sigma = sensors_satellite_gyroscope_w_bias_sigma.get();
  auto const &dt = truth_dt_s->get();

  auto &bias = sensors_satellite_gyroscope_w_bias.get();

  bias =
      bias + dt * lin::multiply(bias_sigma, lin::gaussians<Vector3>(_randoms));
}

Boolean Gyroscope::sensors_satellite_gyroscope_valid() const {
  auto const &disabled = sensors_satellite_gyroscope_disabled.get();

  return !disabled;
}

Vector3 Gyroscope::sensors_satellite_gyroscope_w() const {
  auto const &error = Super::sensors_satellite_gyroscope_w_error.get();
  auto const &truth_w = truth_satellite_attitude_w->get();

  return truth_w + error;
}

Vector3 Gyroscope::sensors_satellite_gyroscope_w_error() const {
  auto const &valid = Super::sensors_satellite_gyroscope_valid.get();
  auto const &bias = sensors_satellite_gyroscope_w_bias.get();
  auto const &sigma = sensors_satellite_gyroscope_w_sigma.get();

  if (valid)
    return bias + lin::multiply(sigma, lin::gaussians<Vector3>(_randoms));
  else
    return lin::nans<Vector3>();
}
} // namespace psim

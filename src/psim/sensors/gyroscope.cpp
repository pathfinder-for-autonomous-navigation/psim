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

namespace psim {

void Gyroscope::step() {
  this->Super::step();

  // TODO : Implement gyroscope bias drift
}

Vector3 Gyroscope::sensors_satellite_gyroscope_w() const {
  auto const &truth_w = truth_satellite_attitude_w->get();
  auto const &bias = sensors_satellite_gyroscope_bias.get();

  // TODO : Implement a white noise model
  return truth_w + bias;
}

Vector3 Gyroscope::sensors_satellite_gyroscope_w_error() const {
  auto const &truth_w = truth_satellite_attitude_w->get();
  auto const &sensors_w = this->Super::sensors_satellite_gyroscope_w.get();

  return sensors_w - truth_w;
}
}  // namespace psim

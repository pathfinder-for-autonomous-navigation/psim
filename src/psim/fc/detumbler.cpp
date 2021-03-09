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

/** @file psim/fc/detumbler.cpp
 *  @author Kyle Krol
 */

#include <psim/fc/detumbler.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>

namespace psim {

void Detumbler::step() {
  this->Super::step();

  auto const &b_body = sensors_satellite_magnetometer_b->get();
  auto &m_body = truth_satellite_magnetorquers_m->get();

  gnc::DetumbleControllerData data;
  data.b_body = b_body;

  gnc::DetumbleActuation actutation;
  gnc::control_detumble(_detumbler, data, actutation);

  if (lin::all(lin::isfinite(actutation.mtr_body_cmd))) {
    m_body = lin::cast<Real>(actutation.mtr_body_cmd);
  } else {
    m_body = lin::zeros<Vector3>();
  }
}
} // namespace psim

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
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

/** @file gnc_attitude_estimator.cpp
 *  @author Kyle Krol */

#include <gnc/attitude_estimator.hpp>
#include <gnc/config.hpp>
#include <gnc/constants.hpp>

#include <lin/core.hpp>
#include <lin/generators/constants.hpp>

namespace gnc {

AttitudeEstimatorState::AttitudeEstimatorState()
: q_body_eci(lin::nans<lin::Vector4f>()),
  gyro_bias(lin::nans<lin::Vector3f>()),
  P(lin::nans<lin::Matrixf<6, 6>>()),
  t(constant::nan),
  is_valid(false) { }

AttitudeEstimatorData::AttitudeEstimatorData()
: r_ecef(lin::nans<lin::Vector3d>()),
  b_body(lin::nans<lin::Vector3f>()),
  s_body(lin::nans<lin::Vector3f>()),
  w_body(lin::nans<lin::Vector3f>()),
  t(constant::nan) { }

AttitudeEstimate::AttitudeEstimate()
: q_body_eci(lin::nans<lin::Vector4f>()),
  gyro_bias(lin::nans<lin::Vector3f>()),
  P(lin::nans<lin::Matrixf<6, 6>>()),
  is_valid(false) { }

void attitude_estimator_reset(AttitudeEstimatorState &state,
    double t, lin::Vector3d const &r_ecef, lin::Vector3f const &b_body,
    lin::Vector3f const &s_body) {
  GNC_ASSERT_NORMALIZED(s_body);

  // TODO : Implement this
  state = AttitudeEstimatorState();
}

void attitude_estimator_reset(AttitudeEstimatorState &state,
    double t, lin::Vector4f const &q_eci_body) {
  GNC_ASSERT_NORMALIZED(q_eci_body);

  // TODO : Implement this
  state = AttitudeEstimatorState();
}

void attitude_estimator_update(AttitudeEstimatorState &state,
    AttitudeEstimatorData const &data, AttitudeEstimate &estimate) {
  // TODO : Implement this
  state = AttitudeEstimatorState();
  estimate = AttitudeEstimate();
}
}  // namespace gnc

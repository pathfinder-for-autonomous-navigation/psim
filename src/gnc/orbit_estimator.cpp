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

/** @file gnc/orbit_estimator.hpp
 *  @author Kyle Krol
 */

#include <gnc/config.hpp>
#include <gnc/constants.hpp>
#include <gnc/orbit_estimator.hpp>

#include <lin/core.hpp>
#include <lin/factorizations.hpp>
#include <lin/generators.hpp>
#include <lin/references.hpp>
#include <lin/substitutions.hpp>

namespace gnc {

void OrbitEstimator::_check_validity() {
  Orbit::_check_validity();
}

void OrbitEstimator::_reset(double dt, lin::Vectord<3> const &w,
    lin::Vectord<3> const &z0, lin::Vectord<3> const &z1,
    lin::Vectord<3> const &r) {
  // Initialize state covariance
  _P = lin::zeros<lin::Matrixd<6, 6>>();
  for (lin::size_t i = 0; i < r.size(); i++)
    _P(i, i) = r(i) * r(i) * r(i) * r(i) / dt;

  // Initialize state
  lin::ref<3, 1>(_x, 0, 0) = z1;
  lin::ref<3, 1>(_x, 3, 0) = (z1 - z0) / dt;

  _check_validity();
}

void OrbitEstimator::_update(double dt, lin::Vectord<3> const &w,
    lin::Vectord<3> const &z, lin::Vectord<3> const &q,
    lin::Vectord<3> const &r) {
  // Assemble noise matrices
  lin::Matrixd<6, 6> Q = lin::zeros<lin::Matrixd<6, 6>>();
  for (lin::size_t i = 0; i < q.size(); i++)
    Q(i, i) = q(i) * q(i);
  lin::Matrixd<3, 3> R = lin::zeros<lin::Matrixd<3, 3>>();
  for (lin::size_t i = 0; i < r.size(); i++)
    R(i, i) = r(i) * r(i);

  // Predict state covariance
  lin::Matrixd<6, 6> F;
  _jac(gnc::constant::mu_earth, w, _x, F);
  F = lin::identity<lin::Matrixd<6, 6>>() + dt * F;
  _P = F * (_P * lin::transpose(F)).eval() + Q;

  // Predict state
  Orbit::_update(dt);

  // Kalman gain
  lin::Matrixd<6, 3> K;
  {
    // TODO
  }

  // Update state covariance
  // TODO

  // Update state
  // TODO
}

void OrbitEstimator::reset(
    int64_t dt_ns, lin::Vectord<3> const &dr_ecef, lin::Vectord<3> const &r) {
  _reset(static_cast<double>(dt_ns)*1.0e-9, dr_ecef, r);
  _check_validity();
}

void OrbitEstimator::update(int64_t dt_ns, lin::Vectord<3> const &w_ecef,
    lin::Vectord<3> r_ecef, lin::Vectord<3> const &q,
    lin::Vectord<3> const &r) {
  if (!is_valid())
    return;

  _update(static_cast<double>(dt_ns)*1.0e-9, w_ecef, r_ecef, q, r);
  _check_validity();
}
} // namespace gnc

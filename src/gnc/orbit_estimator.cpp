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

  // TODO : Implement checks on the state uncertainty
}

void OrbitEstimator::_dv(Vector<3> const &dv, Vector<3> const &ds) {
  Orbit::_dv(dv);

  // Apply additive process noise to the velocity states
  auto S = lin::ref<Matrix<3, 3>>(_S, 3, 3);
  S = S + lin::diag(ds);
}

void OrbitEstimator::_reset(Vector<3> const &w, Vector<3> const &r,
    Vector<3> const &v, Vector<6> const &s) {
  Orbit::_reset(w, r, v);

  // Reset the state uncertainty
  _S = lin::diag(s);
}

void OrbitEstimator::_update(
    Real dt, Vector<3> const &w, Vector<6> const &sqrt_q) {
  // State uncertainty prediction step
  {
    Matrix<6, 6> F;
    _jac(mu, w, _x, F);
    F = lin::identity<Matrix<6, 6>>() + dt * F;

    /* This leverages the square root formulation of the EKF covariance
     * prediction step:
     *
     *   qr([ S_k|k transpose(F_k) ]) = _ S_k+1|k
     *     ([       sqrt(Q_k)      ])
     */
    Matrix<12, 6> A, _;
    lin::ref<Matrix<6, 6>>(A, 0, 0) = _S * lin::transpose(F);
    lin::ref<Matrix<6, 6>>(A, 6, 0) = lin::diag(sqrt_q);
    lin::qr(A, _, _S);
  }

  // State prediction step
  Orbit::_update(dt, w);
}

void OrbitEstimator::_update(Real dt, Vector<3> const &w, Vector<3> const &r,
    Vector<3> const &v, Vector<6> const &sqrt_q, Vector<6> const &sqrt_r) {
  // Prediction step
  _update(dt, w, sqrt_q);

  // Update step
  {
    /* This again leverages the square root formulation of the EKF. The update
     * step is given by:
     * 
     *   qr([      sqrt(R)             0    ]) = _ [ transpose(C)      D     ]
     *     ([ S_k+1|k transpose(H)  S_k+1|k ])     [      0        S_k+1|k+1 ]
     */
    Matrix<12, 12> A, B, _;
    lin::ref<Matrix<6, 6>>(A, 0, 0) = lin::diag(sqrt_r);
    lin::ref<Matrix<6, 6>>(A, 0, 6) = lin::zeros<Matrix<6, 6>>();
    lin::ref<Matrix<6, 6>>(A, 6, 0) = _S;  // H = I
    lin::ref<Matrix<6, 6>>(A, 6, 6) = _S;
    lin::qr(A, _, B);

    Matrix<6, 6> C, D;
    C = lin::transpose(lin::ref<Matrix<6, 6>>(B, 0, 0));
    D = lin::ref<Matrix<6, 6>>(B, 0, 6);

    /* Kalman gain calculation from the square root formulation:
     *
     *   K_k+1 = D inverse(C)
     */
    Matrix<6, 6> K;
    {
      Matrix<6, 6> Q, R;
      lin::qr(lin::transpose(C).eval(), Q, R);
      lin::backward_sub(R, K, (lin::transpose(Q) * lin::transpose(D)).eval());

      K = lin::transpose(K).eval();
    }

    // Measurement
    Vector<6> z;
    lin::ref<Vector<3>>(z, 0, 0) = r;
    lin::ref<Vector<3>>(z, 3, 0) = v;

    _x = _x + K * (z - _x).eval();
    _S = lin::ref<Matrix<6, 6>>(B, 6, 6);
  }
}

OrbitEstimator::OrbitEstimator(Vector<3> const &w_ecef, Vector<3> const &r_ecef,
      Vector<3> const &v_ecef, Vector<6> const &s) {
  _reset(w_ecef, r_ecef, v_ecef, s);
  _check_validity();
}

OrbitEstimator::Vector<3> OrbitEstimator::r_ecef_sigma() const {
  if (!is_valid())
    return lin::nans<Vector<3>>();

  return lin::ref<Vector<3>>(lin::diag(_S), 0, 0);
}

OrbitEstimator::Vector<3> OrbitEstimator::v_ecef_sigma() const {
  if (!is_valid())
    return lin::nans<Vector<3>>();

  return lin::ref<Vector<3>>(lin::diag(_S), 3, 0);
}

void OrbitEstimator::dv(Vector<3> const &dv_ecef, Vector<3> const &ds) {
  if (!is_valid())
    return;

  _dv(dv_ecef, ds);
  _check_validity();
}

void OrbitEstimator::reset(Vector<3> const &w_ecef, Vector<3> const &r_ecef,
    Vector<3> const &v_ecef, Vector<6> const &s) {
  _reset(w_ecef, r_ecef, v_ecef, s);
  _check_validity();
}

void OrbitEstimator::update(
    Time dt_ns, Vector<3> const &w_ecef, Vector<6> const &sqrt_q) {
  if (!is_valid())
    return;

  _update(_time(dt_ns), w_ecef, sqrt_q);
  _check_validity();
}

void OrbitEstimator::update(Time dt_ns, Vector<3> const &w_ecef,
    Vector<3> const &r_ecef, Vector<3> const &v_ecef, Vector<6> const &sqrt_q,
    Vector<6> const &sqrt_r) {
  if (!is_valid())
    return;

  _update(_time(dt_ns), w_ecef, r_ecef, v_ecef, sqrt_q, sqrt_r);
  _check_validity();
}
} // namespace gnc

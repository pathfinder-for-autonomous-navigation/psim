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

/** @file gnc_relative_orbit_estimator.cpp
 *  @author Kyle Krol
 */

#include <gnc/config.hpp>
#include <gnc/relative_orbit_estimate.hpp>
#include <gnc/utilities.hpp>

#include <lin/core.hpp>
#include <lin/factorizations.hpp>
#include <lin/generators.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>
#include <lin/references.hpp>
#include <lin/substitutions.hpp>

#include <tuple>

namespace gnc {

RelativeOrbitEstimate::Matrix<6, 6>
RelativeOrbitEstimate::_state_transition_matrix(Time dt_ns, Real n) {
  static constexpr Real zero  = 0.0;
  static constexpr Real one   = 1.0;
  static constexpr Real two   = 2.0;
  static constexpr Real three = 3.0;
  static constexpr Real four  = 4.0;
  static constexpr Real six   = 6.0;

  /* State transition matrix using the Clohessy Wiltshire equations:
   *
   *   F = [ Phi_rr(dt), Phi_rv(dt) ]
   *       [ Phi_vr(dt), Phi_vv(dt) ].
   *
   * Reference(s):
   *  - https://en.wikipedia.org/wiki/Clohessy%E2%80%93Wiltshire_equations
   */
  Real const dt  = static_cast<Real>(dt_ns) * Real(1.0e-9);
  Real const nt  = n * dt;
  Real const snt = lin::sin(nt);
  Real const cnt = lin::cos(nt);

  return {
    four - three * cnt,    zero, zero,     snt / n,               two * (one - cnt) / n,         zero,
    six * (snt - nt),      one,  zero,     two * (cnt - one) / n, (four * snt - three * nt) / n, zero,
    zero,                  zero, cnt,      zero,                  zero,                          snt / n,
    three * n * snt,       zero, zero,     cnt,                   two * snt,                     zero,
    six * n * (cnt - one), zero, zero,     -two * snt,            four * cnt - three,            zero,
    zero,                  zero, -n * snt, zero,                  zero,                          cnt
  };
}

void RelativeOrbitEstimate::_inputs(Vector<3> const &w_earth_ecef,
    Vector<3> const &r_ecef, Vector<3> const &v_ecef) {
  // This satellites position and velocity in ecef.
  _r_ecef = r_ecef;
  _v_ecef = v_ecef;

  // This satellites position and velocity in ecef0.
  _r_ecef0 = r_ecef;
  _v_ecef0 = v_ecef + lin::cross(w_earth_ecef, r_ecef);

  // Angular rate of this satellite's hill frame.
  _w_hill_ecef0 = lin::cross(_r_ecef0, _v_ecef0) / lin::fro(_r_ecef0);

  // Transformation from ecef0 to this satellite's hill frame.
  utl::dcm(_Q_hill_ecef0, _r_ecef0, _v_ecef0);

  // Angular rate of the Earth
  _w_earth_ecef = w_earth_ecef;
}

void RelativeOrbitEstimate::_predict(
    Time dt_ns, Real n, Matrix<6, 6> const &sqrtQ) {
  Matrix<6, 6> const F = _state_transition_matrix(dt_ns, n);

  // State prediction step.
  _x = (F * _x).eval();

  /* Covariance prediction step.
   * 
   * This leverages the square root formulation of the EKF covariance
   * prediction step:
   *
   *   qr([ S_k|k transpose(F_k) ]) = _ S_k+1|k
   *     ([       sqrt(Q_k)      ])
   */
  Matrix<12, 6> A, _;
  lin::ref<Matrix<6, 6>>(A, 0, 0) = _sqrtP * lin::transpose(F);
  lin::ref<Matrix<6, 6>>(A, 6, 0) = sqrtQ;
  lin::qr(A, _, _sqrtP);
}

void RelativeOrbitEstimate::_update(
    Vector<3> const &dr_hill, Matrix<3, 3> const &sqrtR) {
  /* This again leverages the square root formulation of the EKF. The
   * update step is given by:
   *
   *   qr([      sqrt(R)             0    ]) = _ [ transpose(C)      D     ]
   *     ([ S_k+1|k transpose(H)  S_k+1|k ])     [      0        S_k+1|k+1 ]
   *                                   qr(B) = _ A.
   *
   * Note, that in this case here the measurement matrix has the following form:
   *
   *   H = [ I 0 ]
   *
   * which implies the multiplication involving it's transpose can be done
   * symbolically:
   *
   *   S H' = [ S11 S12 ] [ I ] = [ S11 ]
   *          [ S21 S22 ] [ 0 ]   [ S21 ].
   */
  Matrix<9, 9> A;
  {
    Matrix<9, 9> B, _;
    lin::ref<Matrix<3, 3>>(B, 0, 0) = sqrtR;
    lin::ref<Matrix<3, 6>>(B, 0, 3) = lin::zeros<Matrix<3, 6>>();
    lin::ref<Matrix<6, 3>>(B, 3, 0) = lin::ref<Matrix<6, 3>>(_sqrtP, 0, 0);
    lin::ref<Matrix<6, 6>>(B, 3, 3) = _sqrtP;
    lin::qr(B, _, A);
  }

  /* Kalman gain calculation from the square root formulation:
   *
   *                          K = transpose(D) inverse(C)
   *                        K C = transpose(D)
   *   transpose(C) tranpose(K) = D.
   *
   * where transpose(C) is already upper triangular.
   */
  Matrix<6, 3> K;
  {
    Matrix<3, 6> L;
    lin::backward_sub(lin::ref<Matrix<3, 3>>(A, 0, 0), L, lin::ref<Matrix<3, 6>>(A, 0, 3));
    K = lin::transpose(L);
  }

  // Apply the Kalman gain
  _x = _x + K * (dr_hill - lin::ref<Vector<3>>(_x, 0, 0)).eval();
  _sqrtP = lin::ref<Matrix<6, 6>>(A, 3, 3);
}

void RelativeOrbitEstimate::_outputs() {
  auto const r_hill = lin::ref<Vector<3>>(_x, 0, 0);
  auto const v_hill = lin::ref<Vector<3>>(_x, 3, 0);

  /* Invert the equations presented in the constructor to go from r_hill, v_hill
   * to dr_ecef, dv_ecef.
   */
  _dr_ecef = lin::transpose(_Q_hill_ecef0) * r_hill;
  _dv_ecef = lin::transpose(_Q_hill_ecef0) * v_hill - lin::cross(_w_earth_ecef - _w_hill_ecef0, _dr_ecef);
}

void RelativeOrbitEstimate::_check_validity() {
  if (lin::any(!lin::isfinite(_x)) || lin::any(!lin::isfinite(_dr_ecef)) ||
      lin::any(!lin::isfinite(_dv_ecef)) || lin::any(!lin::isfinite(_sqrtP))) {
    _valid = false;
    _x = lin::nans<Vector<6>>();
    _dr_ecef = lin::nans<Vector<3>>();
    _dv_ecef = lin::nans<Vector<3>>();
    _sqrtP = lin::nans<Matrix<6, 6>>();
  }
  _valid = true;
}

RelativeOrbitEstimate::RelativeOrbitEstimate(Vector<3> const &w_earth_ecef,
    Vector<3> const &r_ecef, Vector<3> const &v_ecef, Vector<3> const &dr_ecef,
    Vector<3> const &dv_ecef, Matrix<6, 6> const &S) {
  _inputs(w_earth_ecef, r_ecef, v_ecef);

  auto r_hill = lin::ref<Vector<3>>(_x, 0, 0);
  auto v_hill = lin::ref<Vector<3>>(_x, 3, 0);
  /* r_hill = Q_hill_ecef * dr_ecef
   *
   * v_hill = Q_hill_ecef * (dv_ecef0 - w_hill_ecef0 x dr_ecef0)
   *        = Q_hill_ecef * (dv_ecef + w_earth_ecef x dr_ecef - w_hill_ecef0 x dr_ecef0)
   *        = Q_hill_ecef * (dv_ecef + (w_earth_ecef - w_hill_ecef) x dr_ecef)
   */
  r_hill = _Q_hill_ecef0 * (dr_ecef);
  v_hill = _Q_hill_ecef0 * (dv_ecef + lin::cross(_w_earth_ecef - _w_hill_ecef0, dr_ecef));
  _sqrtP = S;

  _outputs();
  _check_validity();
}

void RelativeOrbitEstimate::update(Time dt_ns, Vector<3> const &w_earth_ecef,
    Vector<3> const &r_ecef, Vector<3> const &v_ecef,
    Matrix<6, 6> const &sqrtQ) {
  _inputs(w_earth_ecef, r_ecef, v_ecef);
  _predict(dt_ns, lin::norm(_w_hill_ecef0), sqrtQ);
  _outputs();
  _check_validity();
}

void RelativeOrbitEstimate::update(Time dt_ns, Vector<3> const &w_earth_ecef,
    Vector<3> const &r_ecef, Vector<3> const &v_ecef, Vector<3> const &dr_ecef,
    Matrix<6, 6> const &sqrtQ, Matrix<3, 3> const &sqrtR) {
  _inputs(w_earth_ecef, r_ecef, v_ecef);
  _predict(dt_ns, lin::norm(_w_hill_ecef0), sqrtQ);
  _update(_Q_hill_ecef0 * dr_ecef, sqrtR);
  _outputs();
  _check_validity();
}
} // namespace gnc

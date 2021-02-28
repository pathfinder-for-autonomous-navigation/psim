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

/** @file gnc/attitude_estimate.hpp
 *  @author Kyle Krol
 */

#include <gnc/attitude_estimate.hpp>

#include <gnc/config.hpp>
#include <gnc/utilities.hpp>

#include <lin/factorizations.hpp>
#include <lin/math.hpp>
#include <lin/queries.hpp>
#include <lin/references.hpp>
#include <lin/substitutions.hpp>

namespace gnc {

AttitudeEstimate::Matrix<6, 6> AttitudeEstimate::Q(
    Time dt_ns, Real sig_u, Real sig_v) {
  auto const dt = lin::cast<Real>(dt_ns) * Real(1.0e-9);
  auto const var_u = sig_u * sig_u;
  auto const var_v = sig_v * sig_v;

  Matrix<6, 6> Q = lin::zeros<Matrix<6, 6>>();
  lin::diag(lin::ref<Matrix<3, 3>>(Q, 0, 0)) =
      lin::consts<Vector<3>>(dt * (var_v - var_u * dt * dt / Real(6.0)));
  lin::diag(lin::ref<Matrix<3, 3>>(Q, 3, 3)) =
      lin::consts<Vector<3>>(dt * (var_u));
}

void AttitudeEstimate::update(
    Time dt_ns, Vector<3> const &w_body, Matrix<6, 6> const &Q) {
  static constexpr Real half = 0.5;
  static constexpr Real zero = 0.0;
  static constexpr Real N = 6.0;

  /* Unscented Kalman filter tuning parameter adjusting the distance of the
   * sigma points from the mean.
   */
  GNC_TRACKED_CONSTANT(static constexpr Real, UKF_LAMBDA, 1.0);

  /* Reassemble our estimator state using the GRP error distribution
   * representation stacked on top of the gyroscope bias.
   */
  auto x = lin::col(X_, 0);
  x = {zero, zero, zero, w_bias_body_(0), w_bias_body_(1), w_bias_body_(2)};

  /* Generate the rest of the sigma points using the cholesky factorization of
   * (P + Q/2).
   *
   * This is an improvement of pure additive process noise where the other (Q/2)
   * is incorporated later on in the prediction step.
   */
  {
    Matrix<6, 6> L = P_ + half * Q;
    lin::chol(L);

    L = lin::sqrt(N + UKF_LAMBDA) * L;
    for (lin::size_t i = 0; i < L.cols(); i++) {
      lin::col(X_, i + 1) = x + lin::col(L, i);
      lin::col(X_, i + 7) = x - lin::col(L, i);
    }
  }

  /* Propagate sigma points forward in time.
   *
   * The error distribution is then reconstructed about the center sigma point.
   */
  {
    static auto const propegate = [](Real dt, Vector<3> const &w, Vector<4> const &q) {
      GNC_ASSERT_NORMALIZED(q);

      Vector<4> dq;
      Vector<4> const qw = {half * w(0), half * w(1), half * w(2), zero};
      gnc::utl::quat_cross_mult(qw, q, dq);

      return (q + dt * dq).eval();
    };

    Real const dt = static_cast<Real>(dt_ns) * 1.0e-9;
  }
}
} // namespace gnc

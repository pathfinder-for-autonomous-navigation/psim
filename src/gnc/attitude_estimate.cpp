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

AttitudeEstimate::Matrix<6, 6>
AttitudeEstimate::Q(Time dt_ns, Real sig_u, Real sig_v) {
  auto const dt = lin::cast<Real>(dt_ns) * Real(1.0e-9);
  auto const var_u = sig_u * sig_u;
  auto const var_v = sig_v * sig_v;

  Matrix<6, 6> Q = lin::zeros<Matrix<6, 6>>();
  lin::diag(lin::ref<Matrix<3, 3>>(Q, 0, 0)) =
      lin::consts<Vector<3>>(dt * (var_v - var_u * dt * dt / Real(6.0)));
  lin::diag(lin::ref<Matrix<3, 3>>(Q, 3, 3)) =
      lin::consts<Vector<3>>(dt * (var_u));
}
} // namespace gnc

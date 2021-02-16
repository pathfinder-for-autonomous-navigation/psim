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

#ifndef GNC_ATTITUDE_ESTIMATE_HPP_
#define GNC_ATTITUDE_ESTIMATE_HPP_

#include <gnc/config.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>

#include <cstdint>

namespace gnc {

/** @brief
 */
class AttitudeEstimate {
 public:
  /** @brief Type representing real scalars within the class.
   */
  typedef double Real;

  /** @brief Type representing time in nanoseconds within the class.
   */
  typedef int32_t Time;

  /** @brief Convenience template for defining lin matrix types within the
   *         class.
   */
  template <lin::size_t R, lin::size_t C, lin::size_t MR = R,
      lin::size_t MC = C>
  using Matrix = lin::Matrix<Real, R, C, MR, MC>;

  /** @brief Convenience template for defining lin row vector types within the
   *         class.
   */
  template <lin::size_t N, lin::size_t MN = N>
  using RowVector = lin::RowVector<Real, N, MN>;

  /** @brief Convenience template for defining lin vector types within the
   *         class.
   */
  template <lin::size_t N, lin::size_t MN = N>
  using Vector = lin::Vector<Real, N, MN>;

  /** @brief
   *
   *  @param dt_ns
   *  @param sig_u
   *  @param sig_v
   *
   *  @return
   */
  static Matrix<6, 6> Q(Time dt_ns, Real sig_u, Real sig_v);

 private:
  /** @internal
   *
   *  @brief
   */
  bool _valid = false;

  /** @internal
   *
   *  @brief
   */
  Vector<4> _q_body_eci = lin::nans<Vector<4>>();

  /** @internal
   *
   *  @brief
   */
  Vector<3> _w_body = lin::nans<Vector<3>>();

  /** @internal
   *
   *  @brief
   */
  Vector<3> _w_bias_body = lin::nans<Vector<3>>();

  /** @internal
   *
   *  @brief
   */
  Matrix<6, 6> _cov = lin::nans<Matrix<6, 6>>();

  /** @internal
   *
   *  @brief
   */
  Vector<3> _b_eci, _s_eci;
  Vector<4> _q_sun_body;

 public:
  AttitudeEstimate() = default;
  AttitudeEstimate(AttitudeEstimate const &) = default;
  AttitudeEstimate(AttitudeEstimate &&) = default;
  AttitudeEstimate &operator=(AttitudeEstimate const &) = default;
  AttitudeEstimate &operator=(AttitudeEstimate &&) = default;

  /** @brief
   *
   *  @param t_ns
   *  @param r_ecef
   *  @param b_body
   *  @param s_body
   *  @param P
   */
  AttitudeEstimate(Time t_ns, Vector<3> const &r_ecef, Vector<3> const &b_body,
      Vector<3> const &s_body, Matrix<6, 6> const &P);

  /** @return
   */
  inline bool valid() const {
    return _valid;
  }

  /** @return
   */
  inline Vector<4> q_body_eci() const {
    return _q_body_eci;
  }

  /** @return
   */
  inline Vector<3> w_body() const {
    return _w_body;
  }

  /** @return
   */
  inline Vector<3> w_bias_body() const {
    return _w_bias_body;
  }

  /** @return
   */
  inline Matrix<6, 6> P() const {
    return _cov;
  }

  /** @brief
   *
   *  @parma dt
   *  @param w_body
   */
  void update(Time dt, Vector<3> const &w_body);

  /** @brief
   *
   *  @param dt_ns
   *  @param w_body
   *  @param Q
   *  @param t_ns
   *  @param r_ecef
   *  @param b_body
   *  @param R
   */
  void update(Time dt_ns, Vector<3> const &w_body, Matrix<6, 6> const &Q,
      Time t, Vector<3> const &r_ecef, Vector<3> const &b_body,
      Matrix<3, 3> const &R);

  /** @brief
   *
   *  @param dt_ns
   *  @param w_body
   *  @param Q
   *  @param t_ns
   *  @param r_ecef
   *  @param b_body
   *  @param R
   */
  void update(Time dt_ns, Vector<3> const &w_body, Matrix<6, 6> const &Q,
      Time t_ns, Vector<3> const &r_ecef, Vector<3> const &b_body,
      Vector<3> const &s_body, Matrix<5, 5> const &R);
};
};
} // namespace gnc

#endif

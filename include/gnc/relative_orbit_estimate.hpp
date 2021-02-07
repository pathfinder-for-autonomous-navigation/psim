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

/** @file gnc/relative_orbit_estimate.hpp
 *  @author Kyle Krol
 */

#ifndef GNC_RELATIVE_ORBIT_ESTIMATE_HPP_
#define GNC_RELATIVE_ORBIT_ESTIMATE_HPP_

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/references.hpp>

#include <cstdint>

namespace gnc {

/** @brief
 */
class RelativeOrbitEstimate {
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
  template <lin::size_t R, lin::size_t C, lin::size_t MR = R, lin::size_t MC = C>
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

 private:
  /** @internal
   *
   *  @brief True is the relative orbit estimate is valid and false otherwise.
   */
  bool _valid = false;

  /** @internal
   *
   *  @brief State space representation of the relative orbit estimate.
   */
  Vector<6> _x = lin::nans<Vector<6>>();

  /** @internal
   *
   *  @brief Cholesky factorization of the estimate's covariance.
   */
  Matrix<6, 6> _sqrtP = lin::nans<Matrix<6, 6>>();

  /** @internal
   *
   *  @brief Relative position estimate in ECEF.
   */
  Vector<3> _dr_ecef = lin::nans<Vector<3>>();

  /** @internal
   *
   *  @brief Relative velocity estimate in ECEF.
   */
  Vector<3> _dv_ecef = lin::nans<Vector<3>>();

  /** @internal
   *
   *  @brief Temporary variabels used during update steps.
   */
  Vector<3> _r_ecef, _v_ecef, _r_ecef0, _v_ecef0, _w_hill_ecef0, _w_earth_ecef;

  /** @internal
   *
   *  @brief Another temporary variables used during update steps.
   */
  Matrix<3, 3> _Q_hill_ecef0;

  /** @internal
   *
   *  @brief Calculates the state transition matrix from the Clohessy Wiltshire
   *         equations.
   *
   *  @param dt_ns Timestep (ns).
   *  @param n     Mean motion for this spacecraft (rad/s).
   *
   *  @return State transition matrix.
   */
  static Matrix<6, 6> _state_transition_matrix(Time dt_ns, Real n);

  /** @internal
   *
   *  @brief Populates the temporary variables used during update steps.
   *
   *  @param w_earth_ecef Earth's angular rate in ECEF (rad/s).
   *  @param r_ecef       Position of this spacecraft in ECEF (m).
   *  @param v_ecef       Velocity of this spacecraft in ECEF (m).
   */
  void _inputs(Vector<3> const &w_earth_ecef, Vector<3> const &r_ecef,
      Vector<3> const &v_ecef);

  /** @internal
   *
   *  @brief Square root Kalman filter prediction step.
   *
   *  @param dt_ns Timestep (ns)
   *  @param n     Mean motion for this spacecraft (rad/s).
   *  @param sqrtQ Cholesky factorization of the process noise.
   *
   *  Steps the member variables `_x` and `_sqrtP` forward in time.
   */
  void _predict(Time dt_ns, Real n, Matrix<6, 6> const &sqrtQ);

  /** @internal
   *
   *  @brief Square root Kalman filter update step.
   *
   *  @param dr_hill Relative position measurement in the HILL frame (m).
   *  @param sqrtR   Cholesky factorization of the sensor noise.
   */
  void _update(Vector<3> const &dr_hill, Matrix<3, 3> const &sqrtR);

  /** @internal
   *
   *  @brief Sets filter outputs.
   *
   *  The temporary variables, `_x`, and `_sqrtP` must all be set.
   */
  void _outputs();

  /** @internal
   *
   *  @brief Tests whether the current estimate is valid.
   *
   *  If it's not valid, all outputs will be set to `nan` and the `_valid` flag
   *  to false.
   */
  void _check_validity();

 public:
  RelativeOrbitEstimate() = default;
  RelativeOrbitEstimate(RelativeOrbitEstimate const &) = default;
  RelativeOrbitEstimate(RelativeOrbitEstimate &&) = default;
  RelativeOrbitEstimate &operator=(RelativeOrbitEstimate const &) = default;
  RelativeOrbitEstimate &operator=(RelativeOrbitEstimate &&) = default;

  /** @brief
   *
   *  @param w_earth_ecef
   *  @param r_ecef
   *  @param v_ecef
   *  @param dr_ecef
   *  @param dv_ecef
   *  @param S
   */
  RelativeOrbitEstimate(Vector<3> const &w_earth_ecef, Vector<3> const &r_ecef,
      Vector<3> const &v_ecef, Vector<3> const &dr_ecef,
      Vector<3> const &dv_ecef, Matrix<6, 6> const &S);

  /** @return True if the estimate is valid and false otherwise.
   */
  inline bool valid() const {
    return _valid;
  }

  /** @return Relative position estimate of the other spacecraft in ECEF (m).
   *
   *  Will be set to NaNs if the estimate isn't valid.
   */
  inline Vector<3> dr_ecef() const {
    return _dr_ecef;
  }

  /** @return Position estimate of the other spacecraft in the HILL frame (m).
   *
   *  Will be set to NaNs if the estimate isn't valid.
   */
  inline Vector<3> r_hill() const {
    return lin::ref<Vector<3>>(_x, 0, 0);
  }

  /** @return Relative velocity estimate of the other spacecraft in ECEF (m/s).
   *
   *  Will be set to NaNs if the estimate isn't valid.
   */
  inline Vector<3> dv_ecef() const {
    return _dv_ecef;
  }

  /** @return Velocity estimate of the other spacecraft in the HILL frame (m/s).
   *
   *  Will be set to NaNs if the estimate isn't valid.
   */
  inline Vector<3> v_hill() const {
    return lin::ref<Vector<3>>(_x, 3, 0);
  }

  /** @return Cholesky factorization of the state covariance.
   *
   *  This matrix will be upper triangular and will be set to NaNs if the
   *  estimate isn't valid.
   */
  inline Matrix<6, 6> S() const {
    return _sqrtP;
  }

  /** @brief
   *
   *  @param dt_ns
   *  @param w_earth_ecef
   *  @param r_ecef
   *  @param v_ecef
   *  @param sqrtQ
   */
  void update(Time dt_ns, Vector<3> const &w_earth_ecef,
      Vector<3> const &r_ecef, Vector<3> const &v_ecef,
      Matrix<6, 6> const &sqrtQ);

  /** @brief
   *
   *  @param dt_ns
   *  @param w_earth_ecef
   *  @param r_ecef
   *  @param v_ecef
   *  @param dr_ecef
   *  @param sqrtQ
   *  @param sqrtR
   */
  void update(Time dt_ns, Vector<3> const &w_earth_ecef,
      Vector<3> const &r_ecef, Vector<3> const &v_ecef,
      Vector<3> const &dr_ecef, Matrix<6, 6> const &sqrtQ,
      Matrix<3, 3> const &sqrtR);
};
} // namespace gnc

#endif

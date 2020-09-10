/** @file gnc/utilities.hpp
 *  @author Kyle Krol */

#ifndef GNC_UTILITIES_HPP_
#define GNC_UTILITIES_HPP_

#include "config.hpp"
#include "constants.hpp"

#include <lin/core.hpp>
#include <lin/generators/constants.hpp>
#include <lin/references.hpp>

#ifdef abs
#undef abs
#endif
#include <cmath>

namespace gnc {
namespace utl {

/** @fn quat_conj
 *  @param[in]  q   Input quaternion.
 *  @param[out] res Output, conjugated quaternion.
 *  Calculates the conjugate of a quaternion. There is no explicit handling of
 *  NaNs built into this function; however, a finite input will always yield a
 *  finite result. */
template <typename T>
constexpr void quat_conj(lin::Vector<T, 4> const &q, lin::Vector<T, 4> &res);

/** @fn quat_conj
 *  @param[inout] q
 *  Calculates the conjugate of a quaternion in place. There is no explicit
 *  handling of NaNs built into this function; however, a finite input will
 *  always yield a finite result. */
template <typename T>
constexpr void quat_conj(lin::Vector<T, 4> &q);

/** @fn quat_cross_mult
 *  @param[in]  q1
 *  @param[in]  q2
 *  @param[out] res
 *  Cross multiplies two quaternions. This function essentially performs
 *  res = q1 x q2. There is no explicit handling of NaNs built into this
 *  function; however, a finite input will always yield a finite result. */
template <typename T>
constexpr void quat_cross_mult(lin::Vector<T, 4> const &q1,
        lin::Vector<T, 4> const &q2, lin::Vector<T, 4> &res);

/** @fn quat_to_grp
 *  @param[in]  q Input quaternion.
 *  @param[in]  a 
 *  @param[in]  f
 *  @param[out] p Generalized Rodrigues parameter representation.
 *  Converts a quaternion attitude representation to generalized Rodrigues
 *  parameters. There is no explicit handling of NaNs built into this function;
 *  however, a finite input will always yield a finite result.
 *  Source: "Unscented Filtering for Spacecraft Attitude Estimation"
 *          by Markley and Crasidis */
template <typename T>
constexpr void quat_to_qrp(lin::Vector<T, 4> const &q, T a, T f, lin::Vector<T, 3> &p);

/** @fn grp_to_quat
 *  @param[in]  p Input Rodrigues parameters.
 *  @param[in]  a
 *  @param[in]  f
 *  @param[out] q Quaternion representation. */
template <typename T>
constexpr void grp_to_quat(lin::Vector<T, 3> const &p, T a, T f, lin::Vector<T, 4> &q);

/** @fn rotate_frame
 *  @param[in]  q   Quaternion specifying the frame rotation.
 *  @param[in]  v   Vector to be transformed.
 *  @param[out] res Result of the transformation.
 *  Rotates a vector as specified by a quaternion. There is no explicit handling
 *  of NaNs built into this function; however, a finite input will always yield
 *  a finite result. */
template <typename T>
constexpr void rotate_frame(lin::Vector<T, 4> const &q,
    lin::Vector<T, 3> const &v, lin::Vector<T, 3> &res);

/** @fn rotate_frame
 *  @param[in]    q Quaternion specifying the frame rotation.
 *  @param[inout] v Vector rotated in place. 
 *  Rotates a vector as specified by a quaternion in place. There is no explicit
 *  handling of NaNs built into this function; however, a finite input will
 *  always yield a finite result. */
template <typename T>
constexpr void rotate_frame(lin::Vector<T, 4> const &q, lin::Vector<T, 3> &v);

/** @fn dcm
 *  @param[out] DCM Direction cosine matrix.oth x and y must not be the zero vector.
 *  @param[in]  x
 *  @param[in]  y
 *  Generates a DCM from two reference vector in a particular frame. The DCM
 *  will convert from the frame x and y are specified in to the new frame.
 *  The x input vector is normalized to give the first basis vector of the new
 *  frame. The y vector's projection along x is removed resulting in the second
 *  basis vector for the new frame. The third is generated assuming a right hand
 *  coordinate system.
 *  Both x and y must not be the zero vector.
 *  If the vector x and y are very near to being parallel/anti-parallel (
 *  abs(dot(x_hat, y_hat)) > 0.999), DCM is set to NaNs. */
template <typename T>
constexpr void dcm(lin::Matrix<T, 3, 3> &DCM, lin::Vector<T, 3> const &x,
    lin::Vector<T, 3> const &y);

/** @fn dcm_to_quat
 *  @param[in]  DCM Direction cosine matrix.
 *  @param[out] q   Output quaternion.
 *  Determines the quaternion corresponding to the provided direction cosine
 *  matrix. The input direction cosine matrix needs to be orthonormal. There is
 *  no explicit handling of NaNs built into this function; however, a finite
 *  input will always yield a finite result. */ 
template <typename T>
constexpr void dcm_to_quat(lin::Matrix<T, 3, 3> const &DCM,
    lin::Vector<T, 4> &q);

/** @fn triad
 *  @param[in]  R1 First reference vector in a known frame.
 *  @param[in]  R2 Second reference vector in a known frame.
 *  @param[in]  r1 First reference vector in an unknown frame.
 *  @param[in]  r2 Second reference vector in an unknown frame.
 *  @param[out] q  Quaternion from known to unknown frame.
 *  Determines the rotation from a known frame to an unknown frame in the form
 *  of a quaternion.
 *  The vectors R1 & r1 and R2 & r2 represent two vector in two different
 *  frames. As an example, R1 & r1 could be an expected and measured sun vector
 *  while R2 & r2 are an expected and measured magnetic field vector.
 *  The triad algorithm first determines a rotation to move R1 onto r1 and then,
 *  with the last degree of freedom, gets R2 as close as possible to r2 when
 *  operated on with the same rotation that places R1 perfectly on r1.
 *  If R1 & R2 or r1 &r2 are too close to parallel/antiparallel (currently, the
 *  threshold is within a degree of one another) the quaternion q is set to
 *  NaNs. Otherwise, a finite input will always yield a finite result.
 *  REQUIRES: R1, R2, r1, and r2 to be unit vectors. */
template <typename T>
constexpr void triad(lin::Vector<T, 3> const &R1, lin::Vector<T, 3> const &R2,
    lin::Vector<T, 3> const &r1, lin::Vector<T, 3> const &r2,
    lin::Vector<T, 4> &q);

/** @fn vec_rot_to_quat
 *  @param[in]  u Unit vector being rotated to.
 *  @param[in]  v Unit vector being rotated from.
 *  @param[out] q Quaternion storing the calculated rotation.
 *  Calculates the quaternion which would rotate vector v onto vector u in the
 *  smallest possible rotation. If the vectors are antiparallel, the quaternion
 *  defaults to a rotation about the z-axis. There is no explicit handling of
 *  NaNs built into this function; however, a finite input will always yield a
 *  finite result.
 *  REQUIRES: Vectors u and v must both be unit vectors. */
template <typename T>
constexpr void vec_rot_to_quat(lin::Vector<T, 3> const &u,
    lin::Vector<T, 3> const &v, lin::Vector<T, 4> &q);

}  // namespace utl
}  // namespace gnc

#include "inl/utilities.inl"

#endif

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

/** @fn dcm_to_quat
 *  @param[in]  M Direction cosine matrix.
 *  @param[out] q Output quaternion.
 *  Determines the quaternion corresponding to the provided direction cosine
 *  matrix. The input direction cosine matrix needs to be orthonormal. There is
 *  no explicit handling of NaNs built into this function; however, a finite
 *  input will always yield a finite result. */ 
template <typename T>
inline void dcm_to_quat(lin::Matrix<T, 3, 3> const &M, lin::Vector<T, 4> &q);

/** @fn triad
 *  @param[in]  R1 First known vector in a known frame.
 *  @param[in]  R2 Second known vector in a known frame.
 *  @param[in]  r1 First measurement vector in an unknown frame.
 *  @param[in]  r2 Second measurement vector in an unknown frame.
 *  @param[out] q  Quaternion transforming from the unknown to known frame.
 *  Given a set of unit vector from a known frame and then unknown frame, this
 *  function calculates the rotation required to transform from the latter to
 *  the former. The first vectors, R1 and r1, will be matched exactly by the
 *  rotation while R2 and r2 will be matched as best as possible given the first
 *  constraint. If R1 and R2, or r1 and r2 are within a degree of one another,
 *  the function will return a quaternion of NaNs. Otherwise, a finite input
 *  will always yield a finite result.
 *  REQUIRES: R1, R2, r1, and r2 to be unit vectors. */
template <typename T>
inline void triad(lin::Vector<T, 3> const &R1, lin::Vector<T, 3> const &R2,
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
inline void vec_rot_to_quat(lin::Vector<T, 3> const &u,
    lin::Vector<T, 3> const &v, lin::Vector<T, 4> &q);

}  // namespace utl
}  // namespace gnc

#include "inl/utilities.inl"

#endif

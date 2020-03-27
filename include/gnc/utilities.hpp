/** @file gnc/utilities.hpp
 *  @author Kyle Krol */

#ifndef GNC_UTILITIES_HPP_
#define GNC_UTILITIES_HPP_

#include "config.hpp"

#include <lin/core.hpp>

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
 *  Given a set of vectors in a knows frame (N_*), and a matching set of
 *  measured vectors in an unknown frame (B_*), this function will determine the
 *  quaternion required to rotate the unknown frame to the known frame. This
 *  result is stored in q.
 *  @returns Zero if the function was succesful and false otherwise.
 * 
 *  REQUIRES: N_* and B_* vectors must be unit vectors. */
template <typename T>
inline int triad(lin::Vector<T, 3> const &N_sun, lin::Vector<T, 3> const &N_mag,
    lin::Vector<T, 3> const &B_sun, lin::Vector<T, 3> const &B_mag, lin::Vector<T, 4> &q);

/** @fn vec_rot_to_quat
 *  Calculates the quaternion which would rotate vector v onto vector u in the
 *  smallest possible rotation. If the vector are near antiparallel, the
 *  rotations is defaulted to about the z-axis.
 * 
 *  REQUIRES: Vectors u and v must both be unit vectors. */
template <typename T>
inline void vec_rot_to_quat(lin::Vector<T, 3> const &u, lin::Vector<T, 3> const &v,
    lin::Vector<T, 4> &q);

}  // namespace utl
}  // namespace gnc

#include "inl/utilities.inl"

#endif

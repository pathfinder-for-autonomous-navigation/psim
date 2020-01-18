//
// include/gnc_utilities.hpp
// PSim
//
// Contributors:
//   Kyle Krol  kpk63@cornell.edu
//
// Pathfinder for Autonomous Navigation
// Space Systems Design Studio
// Cornell Univeristy
//

#ifndef PAN_PSIM_INCLUDE_GNC_UTILITIES_HPP_
#define PAN_PSIM_INCLUDE_GNC_UTILITIES_HPP_

#include "lin.hpp"

#ifdef abs
#undef abs
#endif
#include <cmath>

namespace gnc {
namespace utl {

/** @fn quat_conj
 *  Stores the conjugate of the quaternion q in res. */
template <typename T>
constexpr void quat_conj(lin::Vector<T, 4> const &q, lin::Vector<T, 4> &res);

/** @fn quat_conj
 *  Takes the conjugate of the quaternion q in place. */
template <typename T>
constexpr void quat_conj(lin::Vector<T, 4> &q);

/** @fn quat_cross_mult
 *  Cross multiply quaternions q1 and q2 then store the result in res. */
template <typename T>
constexpr void quat_cross_mult(lin::Vector<T, 4> const &q1,
    lin::Vector<T, 4> const &q2, lin::Vector<T, 4> &res);

/** @fn rotate_frame
 *  Rotates the vector v according to the quaternion q. The result is stored in
 *  res. */
template <typename T>
constexpr void rotate_frame(lin::Vector<T, 4> const &q, lin::Vector<T, 3> const &v,
    lin::Vector<T, 3> &res);

/** @fn rotate_frame
 *  Rotates the vector v in-place according to the quaternion q. */
template <typename T>
constexpr void rotate_frame(lin::Vector<T, 4> const &q, lin::Vector<T, 3> &v);

/** @fn dcm_to_quat
 *  Calculates the quaternion corresponding to the DCM m and stores the result
 *  in q. */ 
template <typename T>
inline void dcm_to_quat(lin::Matrix<T, 3, 3> const &m, lin::Vector<T, 4> &q);

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

#include "inl/gnc_utilities.inl"

#endif

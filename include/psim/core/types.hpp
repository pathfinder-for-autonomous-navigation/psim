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

/** @file psim/core/types.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_TYPES_HPP_
#define PSIM_CORE_TYPES_HPP_

#include <lin/core.hpp>
#include <lin/generators/randoms.hpp>

namespace psim {

/** @brief Boolean parameter and field type used throughout psim.
 *
 *  This type is transactable to and from Python.
 */
using Boolean = bool;

/** @brief Signed integer parameter and field type used throughout psim.
 *
 *  This type is transactable to and from Python.
 */
using Integer = long;

/** @brief Floating point parameter and field type used throughout psim.
 *
 *  This type is transactable to and from Python.
 */
using Real = double;

/** @brief Standardized vector type used throughout psim.
 */
template <lin::size_t N, lin::size_t MN = N>
using Vector = lin::Vector<Real, N, MN>;

/** @brief Two dimensional floating point vector parameter and field type used
 *         throughout psim.
 *
 *  This type is transactable to and from Python.
 */
using Vector2 = Vector<2>;

/** @brief Three dimensional floating point vector parameter and field type used
 *         throughout psim.
 *
 *  This type is transactable to and from Python.
 */
using Vector3 = Vector<3>;

/** @brief Four dimensional floating point vector parameter and field type used
 *         throughout psim.
 *
 *  This type is transactable to and from Python.
 */
using Vector4 = Vector<4>;

/** @brief Standardized row vector type used throughout psim.
 */
template <lin::size_t N, lin::size_t MN = N>
using RowVector = lin::RowVector<Real, N, MN>;

/** @brief Standardized matrix type used throughout psim.
 */
template <lin::size_t R, lin::size_t C, lin::size_t MR = R, lin::size_t MC = C>
using Matrix = lin::Matrix<Real, R, C, MR, MC>;

/** @brief Randoms number generator used throughout psim.
 */
using RandomsGenerator = lin::internal::RandomsGenerator;

} // namespace psim

#endif

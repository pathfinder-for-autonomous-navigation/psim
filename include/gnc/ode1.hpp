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

/** @file gnc/ode1.hpp
 *  @author Kyle Krol
 */

#ifndef GNC_ODE1_HPP_
#define GNC_ODE1_HPP_

#include <lin/core.hpp>

namespace gnc {

/** @brief First order fixed step size integrator.
 *
 *  @tparam T Fundamental data type.
 *  @tparam N Number of state parameters.
 */
template <typename T, lin::size_t N>
class Ode1 {
 private:
  lin::Vector<T, N> _k[1];

 public:
  /** @brief Step a differential equation forward in time by a single timestep.
   *
   *  @param[in] ti  Initial time.
   *  @param[in] dt  Integrator timestep.
   *  @param[in] xi  Initial state.
   *  @param[in] ptr Pointer to arbitrary data accesible in the update function.
   *  @param[in] dx  Differential update function.
   *
   *  Used for fixed timestep numerical integration. Implements the first order
   *  integration scheme known as Euler's method.
   *
   *  Reference(s):
   *   - https://en.wikipedia.org/wiki/Euler_method
   */
  lin::Vector<T, N> operator()(
      T ti, T dt,
      lin::Vector<T, N> const &xi, void *ptr,
      lin::Vector<T, N> (*dx)(T t, lin::Vector<T, N> const &x, void *ptr)) {
    // Scratch buffers
    auto &k1 = _k[0];

    k1 = dx(ti, xi, ptr);

    // Step forwad in time
    return (xi + dt * k1).eval();
  }
};
}  // namespace gnc

#endif

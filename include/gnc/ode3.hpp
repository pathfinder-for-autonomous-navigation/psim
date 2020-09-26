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

/** @file gnc/ode3.hpp
 *  @author Kyle Krol
 */

#ifndef GNC_ODE3_HPP_
#define GNC_ODE3_HPP_

#include <lin/core.hpp>

namespace gnc {

/** @brief Third order fixed step size integrator.
 *
 *  @tparam T Fundamental data type.
 *  @tparam N Number of state parameters.
 */
template <typename T, lin::size_t N>
class Ode3 {
 private:
  lin::Vector<T, N> _k[5];

 public:
  /** @brief Step a differential equation forward in time by a single timestep.
   *
   *  @param[in] ti  Initial time.
   *  @param[in] dt  Integrator timestep.
   *  @param[in] xi  Initial state.
   *  @param[in] ptr Pointer to arbitrary data accesible in the update function.
   *  @param[in] dx  Differential update function.
   *
   *  Used for fixed timestep numerical integration. Implements the third order
   *  integration scheme known as Ralston's method.
   *
   *  Reference(s):
   *   - https://en.wikipedia.org/wiki/List_of_Runge–Kutta_methods#Ralston's_method
   */
  lin::Vector<T, N> operator()(
      T ti, T dt,
      lin::Vector<T, N> const &xi, void *ptr,
      lin::Vector<T, N> (*dx)(T t, lin::Vector<T, N> const &x, void *ptr)) {
    // Table of values
    constexpr static T
        c2 = 1.0 / 2.0, a21 = 1.0 / 2.0,
        c3 = 3.0 / 4.0,                  a22 = 3.0 / 4.0,
                        b1  = 2.0 / 9.0, b2  = 1.0 / 3.0, b3 = 4.0 / 9.0;

    // Scratch buffers
    auto &ks = _k[0];
    auto &k1 = _k[1];
    auto &k2 = _k[2];
    auto &k3 = _k[3];

    k1 = dx(ti, xi, ptr);
    ks = xi + a21 * dt * k1;
    k2 = dx(ti + c2 * dt, ks, ptr);
    ks = xi + a22 * dt * k2;
    k3 = dx(ti + c3 * dt, ks, ptr);

    // Step forward in time
    return (xi + dt * (b1 * k1 + b2 * k2 + b3 * k3)).eval();
  }
};
}  // namespace gnc

#endif

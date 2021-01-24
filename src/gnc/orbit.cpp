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

/** @file include/orbit.cpp
 *  @author Kyle Krol
 */

#include <gnc/config.hpp>
#include <gnc/constants.hpp>
#include <gnc/orbit.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>
#include <lin/references.hpp>

namespace gnc {

void Orbit::_dot(Real mu, Vector<3> const &w, Vector<6> const &x, Vector<6> &dx) {
  auto const r = lin::ref<Vector<3>>(x, 0, 0);
  auto const v = lin::ref<Vector<3>>(x, 3, 0);

  auto const nr1 = lin::norm(r);
  auto const nr3 = nr1 * nr1 * nr1;

  Vector<3> const F = -mu * r / nr3 - 2.0 * lin::cross(w, v) - lin::cross(w, lin::cross(w, r));
  dx = {v(0), v(1), v(2), F(0), F(1), F(2)};
}

void Orbit::_jac(Real mu, Vector<3> const &w, Vector<6> const &x, Matrix<6, 6> &J) {
  // Create references to vector elements
  auto const &w_x = w(0);
  auto const &w_y = w(1);
  auto const &w_z = w(2);
  auto const &r_x = x(0);
  auto const &r_y = x(1);
  auto const &r_z = x(2);

  // Useful substitution variables
  auto const nr = lin::norm(lin::ref<Vector<3>>(x, 0, 0));
  auto const nr3 = nr * nr * nr;
  auto const nr5 = nr3 * nr * nr;

  // Default all elements to zero
  J = lin::zeros<Matrix<6, 6>>();

  /*  To run the code generation yourself, make sure you have sympy and jupyer
   *  installed in your virtual environment with:
   *
   *    pip install jupyter sympy
   *
   *  Then, start a jupyer notebook with the following command from the root of
   *  the psim repository:
   *
   *    jupyer notebook src/gnc/orbit_estimator_jacobian.ipynb
   *
   *  The last step will spit out the code seen below.
   */
  // BEGIN autocoded portion
  J(0, 3) = 1;
  J(1, 4) = 1;
  J(2, 5) = 1;
  J(3, 0) = -mu*1.0/nr3 + 3*mu*1.0/nr5*(r_x*r_x) + w_y*w_y + w_z*w_z;
  J(3, 1) = 3*mu*r_x*r_y*1.0/nr5 - w_x*w_y;
  J(3, 2) = 3*mu*r_x*r_z*1.0/nr5 - w_x*w_z;
  J(3, 4) = 2*w_z;
  J(3, 5) = -2*w_y;
  J(4, 0) = 3*mu*r_x*r_y*1.0/nr5 - w_x*w_y;
  J(4, 1) = -mu*1.0/nr3 + 3*mu*1.0/nr5*(r_y*r_y) + w_x*w_x + w_z*w_z;
  J(4, 2) = 3*mu*r_y*r_z*1.0/nr5 - w_y*w_z;
  J(4, 3) = -2*w_z;
  J(4, 5) = 2*w_x;
  J(5, 0) = 3*mu*r_x*r_z*1.0/nr5 - w_x*w_z;
  J(5, 1) = 3*mu*r_y*r_z*1.0/nr5 - w_y*w_z;
  J(5, 2) = -mu*1.0/nr3 + 3*mu*1.0/nr5*(r_z*r_z) + w_x*w_x + w_y*w_y;
  J(5, 3) = 2*w_y;
  J(5, 4) = -2*w_x;
  // END autocoded portion
}

void Orbit::_check_validity() {
  // TODO : Carry over validity checks from Nathan's implementation

  _is_valid = true;
}

void Orbit::_dv(Vector<3> const &dv) {
  auto v = lin::ref<Vector<3>>(_x, 3, 0);
  v = v + dv;
}

void Orbit::_reset(Vector<3> const &w, Vector<3> const &r, Vector<3> const &v) {
  _w = w;
  _x = {r(0), r(1), r(2), v(0), v(1), v(2)};
}

void Orbit::_update(Real dt, Vector<3> const &w) {
  _w = w;
  _x = _ode(0.0, dt, _x, &_w, [](Real t, Vector<6> const &x, void *ptr) {
    Vector<6> dx;
    _dot(mu, *((Vector<3> const *)ptr), x, dx);
    return dx;
  });
}

Orbit::Orbit(Vector<3> const &w_ecef, Vector<3> const &r_ecef, Vector<3> const &v_ecef) {
  _reset(w_ecef, r_ecef, v_ecef);
  _check_validity();
}

Orbit::Vector<3> Orbit::r_ecef() const {
  if (!is_valid())
    return lin::nans<Vector<3>>();

  return lin::ref<Vector<3>>(_x, 0, 0);
}

Orbit::Vector<3> Orbit::v_ecef() const {
  if (!is_valid())
    return lin::nans<Vector<3>>();

  return lin::ref<Vector<3>>(_x, 3, 0);
}

void Orbit::dv(Vector<3> const &dv_ecef) {
  if (!is_valid())
    return;

  _dv(dv_ecef);
  _check_validity();
}

void Orbit::reset(Vector<3> const &w_ecef, Vector<3> const &r_ecef, Vector<3> const &v_ecef) {
  _reset(w_ecef, r_ecef, v_ecef);
  _check_validity();
}

void Orbit::update(Time dt_ns, lin::Vector3d const &w_ecef) {
  if (!is_valid())
    return;

  _update(_time(dt_ns), w_ecef);
  _check_validity();
}
} // namespace gnc

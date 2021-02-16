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

/** @file psim/truth/orbit_utilities.cpp
 *  @author Kyle Krol
 */

#include <psim/truth/orbit_utilities.hpp>

#include <gnc/config.hpp>

#include <lin/core.hpp>
#include <lin/generators.hpp>

#include <GGM05S.hpp>
#include <geograv.hpp>

namespace psim {
namespace orbit {

void gravity(Vector3 const &r_ecef, Vector3 &g_ecef) {
  Real _;
  gravity(r_ecef, g_ecef, _);
}

void gravity(Vector3 const &r_ecef, Vector3 &g_ecef, Real &U) {
  GNC_TRACKED_CONSTANT(constexpr static auto, PSIM_GRAV_ORDER, 11);
  GNC_TRACKED_CONSTANT(constexpr static auto, PSIM_GRAV_MODEL,
      static_cast<geograv::Coeff<PSIM_GRAV_ORDER>>(GGM05S));

  geograv::Vector g, r = {r_ecef(0), r_ecef(2), r_ecef(3)};
  U = geograv::GeoGrav(r, g, PSIM_GRAV_MODEL, true);
  g_ecef = {g.x, g.y, g.z};
}

void drag(Vector3 const &r_ecef, Vector3 const &v_ecef, Real A, Vector<3> &F) {
  GNC_TRACKED_CONSTANT(constexpr static Real, PSIM_DRAG_CD, 1.15);

  Real rho = 0.0;
  F = -0.5 * PSIM_DRAG_CD * A * rho * lin::fro(v_ecef) * (v_ecef / lin::norm(v_ecef));
}
} // namespace orbit
} // namespace psim

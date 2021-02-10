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

/** @file psim/truth/earth.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_TRUTH_EARTH_HPP_
#define PSIM_TRUTH_EARTH_HPP_

#include <psim/truth/earth.yml.hpp>

namespace psim {

/** @brief Ephemeris model implemented using the flight software GNC library.
 *
 *  An updated model should be implemented as well that uses a more accurate
 *  model as our truth model.
 */
class EarthGnc : public Earth<EarthGnc> {
 private:
  typedef Earth<EarthGnc> Super;

 public:
  using Super::Earth;

  EarthGnc() = delete;
  virtual ~EarthGnc() = default;

  Vector4 truth_earth_q_ecef_eci() const;
  Vector4 truth_earth_q_eci_ecef() const;
  Vector3 truth_earth_w() const;
  Vector3 truth_earth_w_dot() const;
};
}  // namespace psim

#endif

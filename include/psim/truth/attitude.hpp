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

/** @file psim/truth/attitude.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_TRUTH_ATTITUDE_HPP_
#define PSIM_TRUTH_ATTITUDE_HPP_

#include <psim/truth/attitude.yml.hpp>

namespace psim {

/** @brief Attitude dynamics without fuel slosh simulation.
 */
class AttitudeNoFuel : public Attitude<AttitudeNoFuel> {
 private:
  typedef Attitude<AttitudeNoFuel> Super;

 public:
  using Super::Attitude;

  AttitudeNoFuel() = delete;
  virtual ~AttitudeNoFuel() = default;

  virtual void step() override;
};
}  // namespace psim

#endif

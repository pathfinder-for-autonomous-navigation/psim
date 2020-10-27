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

/** @file psim/truth/hill_frame.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_TRUTH_HILL_FRAME_HPP_
#define PSIM_TRUTH_HILL_FRAME_HPP_

#include <psim/truth/hill_frame.yml.hpp>

namespace psim {

class HillFrameEci : public HillFrame<HillFrameEci> {
 private:
  typedef HillFrame<HillFrameEci> Super;

 public:
  HillFrameEci() = delete;
  virtual ~HillFrameEci() = default;

  /** @brief Set the frame argument to ECI.
   */
  HillFrameEci(Configuration const &config, std::string const &prefix,
      std::string const &satellite);

  Vector4 prefix_leader_q_hill_frame() const;
  Vector3 prefix_leader_w_hill() const;
  Vector3 prefix_follower_orbit_r_hill() const;
  Vector3 prefix_follower_orbit_v_hill() const;
};
}  // namespace psim

#endif

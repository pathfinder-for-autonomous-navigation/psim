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

/** @file psim/truth/environment.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_TRUTH_ENVIRONMENT_HPP_
#define PSIM_TRUTH_ENVIRONMENT_HPP_

#include <psim/truth/environment.yml.hpp>

namespace psim {

class EnvironmentGnc : public Environment<EnvironmentGnc> {
 private:
  typedef Environment<EnvironmentGnc> Super;

 public:
  EnvironmentGnc() = delete;
  virtual ~EnvironmentGnc() = default;

  /** @brief Set the frame argument to ECEF.
   */
  EnvironmentGnc(RandomsGenerator &randoms, Configuration const &config,
      std::string const &satellite);

  Vector3 truth_satellite_environment_b() const; // Reported in ECEF
  Vector3 truth_satellite_environment_s() const; // Reported in ECI
};
} // namespace psim

#endif

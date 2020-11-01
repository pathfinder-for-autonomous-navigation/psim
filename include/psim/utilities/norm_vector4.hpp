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

/** @file psim/utilities/norm_vector4.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_UTILITIES_NORM_VECTOR4_HPP_
#define PSIM_UTILITIES_NORM_VECTOR4_HPP_

#include <psim/utilities/norm_vector4.yml.hpp>

namespace psim {

class NormVector4 : public NormVector4Interface<NormVector4> {
 private:
  typedef NormVector4Interface<NormVector4> Super;

 public:
  using Super::NormVector4Interface;

  NormVector4() = delete;
  virtual ~NormVector4() = default;

  Real vector_norm() const;
};
}  // namespace psim

#endif

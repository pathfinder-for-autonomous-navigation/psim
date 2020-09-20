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

/** @file psim/core/parameter_base.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_PARAMETER_BASE_HPP_
#define PSIM_CORE_PARAMETER_BASE_HPP_

#include <stdexcept>

namespace psim {

template <typename T>
class Parameter;

/** @brief Virtual base class for all parameters.
 *
 *  The main purpose of this class is to allow dynamic casting to be used to check
 *  parameter types at runtime and provide helpful casting functions.
 */
class ParameterBase {
 protected:
  ParameterBase() = default;

 public:
  virtual ~ParameterBase() = default;

  /** @brief Attempt to get the parameter's underlying value.
   *
   *  @tparam Expected underlying type.
   *
   *  @return Reference to the underlying value.
   *
   *  If the underlying type doesn't match the expected underlying type, a runtime
   *  error will be thrown.
   *
   *  @{
   */
  template <typename T>
  T const &get() const {
    auto const *param_ptr = dynamic_cast<Parameter<T> const *>(this);
    if (!param_ptr)
      throw std::runtime_error("Invalid parameter type in call to 'get'.");
    
    return (T const &) *param_ptr;
  }

  template <typename T>
  T &get() {
    auto *param_ptr = dynamic_cast<Parameter<T> *>(this);
    if (!param_ptr)
      throw std::runtime_error("Invalid parameter type in call to 'get'.");

    return (T &) *param_ptr;
  }
  /** @}
   */
};
}  // namespace psim

#endif

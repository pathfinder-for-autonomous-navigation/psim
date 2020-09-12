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

/** @file psim/core/parameters.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_PARAMETER_HPP_
#define PSIM_CORE_PARAMETER_HPP_

#include "parameter_base.hpp"

#include <type_traits>
#include <utility>

namespace psim {

/** @brief Simulation parameter holding a single value of the underlying type.
 *
 *  @tparam T Underlying type.
 */
template <typename T>
class Parameter : public ParameterBase {
 private:
  /** @brief Underlying value.
   */
  T _value;

 public:
  virtual ~Parameter() = default;

  /** @brief Default constructs the parameter's initial value.
   *
   *  https://stackoverflow.com/questions/2417065/does-the-default-constructor-initialize-built-in-types
   */
  Parameter()
      : _value() { }

  /** @param[in] value Parameter's initial value.
   *
   *  @{
   */
  Parameter(T const &value)
      : _value(value) { }

  Parameter(T &&value)
      : _value(std::move(value)) { }
  /** @}
   */

  /** @param[in] value Parameter's initial value.
   *
   *  If the passed parameter's underlying type does not match, a runtime error
   *  will be thrown.
   *
   *  @{
   */
  Parameter(ParameterBase const &param)
      : _value(param.get<T>()) { }

  Parameter(ParameterBase &&param)
      : _value(std::move(param.get<T>())) { }
  /** @}
   */

  /** @return Reference to the underlying type.
   *
   *  @{
   */
  operator T const & () const {
    return _value;
  }

  operator T & () {
    return _value;
  }
  /** @}
   */

  /** @param[in] value New value held by the parameter.
   *
   *  @return Reference to this parameter.
   *
   *  @{
   */
  Parameter<T> &operator=(T const &value) {
    _value = value;
  }

  Parameter<T> &operator=(T &&value) {
    _value = std::move(value);
  }
  /** @}
   */

  /** @param[in] param New value held by the parameter.
   *
   *  @return Reference to this parameter.
   *
   *  If the passed parameter's underlying type does not match a runtime error
   *  will be thrown.
   *
   *  @{
   */
  Parameter<T> &operator=(ParameterBase const &param) {
    _value = param.get<T>();
  }

  Parameter<T> &operator=(ParameterBase &&param) {
    _value = std::move(param.get<T>());
  }
  /** @}
   */

  /** @returns Reference to the underlying value.
   *
   *  This function was re-implemented to avoid potential type checking overhead
   *  and keep a consistant interface.
   *
   *  @{
   */
  template <typename U = T>
  U const &get() const {
    static_assert(std::is_same<U, T>::value, "Invalid parameter type in call to 'get'.");
    return _value;
  }

  template <typename U = T>
  U &get() {
    static_assert(std::is_same<U, T>::value, "Invalid parameter type in call to 'get'.");
    return _value;
  }
  /** @}
   */
};
}  // namespace psim

#endif

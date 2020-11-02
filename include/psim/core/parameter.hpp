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

/** @file psim/core/parameter.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_PARAMETER_HPP_
#define PSIM_CORE_PARAMETER_HPP_

#include <psim/core/castable_base.hpp>
#include <psim/core/nameable.hpp>

#include <utility>

namespace psim {

template <typename T>
class Parameter;

/** @brief Virtual base class for all parameters.
 *
 *  The main purpose of this class is to allow dynamic casting to be used to
 *  check parameter types at runtime and provide helpful casting functions.
 */
class ParameterBase : public virtual Nameable, public CastableBase<Parameter> {
 protected:
  ParameterBase() = default;

 public:
  using CastableBase<Parameter>::cast;

  virtual ~ParameterBase() = default;

  /** @brief Attempt to get the parameter's underlying value.
   *
   *  @tparam Expected underlying type.
   *
   *  @return Reference to the underlying value.
   *
   *  If the underlying type doesn't match the expected underlying type, a
   *  runtime error will be thrown.
   *
   *  @{
   */
  template <typename T>
  T const &get() const {
    return cast<T>().get();
  }

  template <typename T>
  T &get() {
    return cast<T>().get();
  }
  /** @}
   */
};

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

  /** @brief Default constructs the parameter's initial value with no name.
   *
   *  https://stackoverflow.com/questions/2417065/does-the-default-constructor-initialize-built-in-types
   */
  Parameter() : Nameable("", "parameter"), _value() {}

  /** @brief Default constructs the parameter's value.
   *
   *  @param[in] name Parameter's name.
   *
   *  @{
   */
  Parameter(std::string const &name) : Nameable(name, "parameter"), _value() {}

  Parameter(std::string &&name)
    : Nameable(std::move(name), "parameter"), _value() {}
  /** @}
   */

  /** @param[in] value Parameter's initial value.
   *
   *  @{
   */
  Parameter(T const &value) : Nameable("", "parameter"), _value(value) {}

  Parameter(T &&value) : Nameable("", "parameter"), _value(std::move(value)) {}
  /** @}
   */

  /** @param[in] name Parameter's name.
   *  @param[in] value Parameter's initial value.
   *
   *  @{
   */
  Parameter(std::string const &name, T const &value)
    : Nameable(name, "parameter"), _value(value) {}

  Parameter(std::string &&name, T const &value)
    : Nameable(std::move(name), "parameter"), _value(value) {}

  Parameter(std::string const &name, T &&value)
    : Nameable(name, "parameter"), _value(std::move(value)) {}

  Parameter(std::string &&name, T &&value)
    : Nameable(std::move(name), "parameter"), _value(std::move(value)) {}
  /** @}
   */

  /** @return Reference to itself.
   *
   *  This function was implemented here with additional type checking to keep a
   *  consistant interface with the parameter base class.
   *
   *  @{
   */
  template <typename U = T>
  Parameter<U> const &cast() const {
    static_assert(std::is_same<U, T>::value,
        "Invalid type in call to Parameter::cast() const");

    return *this;
  }

  template <typename U = T>
  Parameter<U> &cast() {
    static_assert(
        std::is_same<U, T>::value, "Invalid type in call to Parameter::cast()");

    return *this;
  }
  /** @}
   */

  /** @returns Reference to the underlying value.
   *
   *  This function was implemented here with additional type checking to keep a
   *  consistant interface with the parameter base class.
   *
   *  @{
   */
  template <typename U = T>
  U const &get() const {
    static_assert(std::is_same<U, T>::value,
        "Invalid type in call to Parameter::get() const");
    return _value;
  }

  template <typename U = T>
  U &get() {
    static_assert(
        std::is_same<U, T>::value, "Invalid type in call to Parameter::get()");
    return _value;
  }
  /** @}
   */
};
} // namespace psim

#endif

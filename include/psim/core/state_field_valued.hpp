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

/** @file psim/core/state_field_valued.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_STATE_FIELD_VALUED_HPP_
#define PSIM_CORE_STATE_FIELD_VALUED_HPP_

#include "state_field.hpp"
#include "state_field_writable.hpp"

#include <utility>

namespace psim {

/** @brief Writable state field implemenation directly backed by a value.
 *
 *  @tparam Underlying type.
 */
template <typename T>
class StateFieldValued : public StateFieldWritable<T> {
 private:
  /** @brief Underlying value.
   */
  T mutable _value;

 public:
  using StateFieldWritable<T>::operator=;

  virtual ~StateFieldValued() = default;

  /** @brief Default constructs the parameter's initial value.
   *
   *  https://stackoverflow.com/questions/2417065/does-the-default-constructor-initialize-built-in-types
   */
  StateFieldValued()
      : _value() { }

  /** @param[in] value Initial value of the state field.
   *
   *  @{
   */
  StateFieldValued(T const &value)
      : _value(value) { }

  StateFieldValued(T &&value)
      : _value(std::move(value)) { }
  /** @}
   */

  /** @param[in] param Initial value of the state field.
   *
   *  If the parameter's underlying type doesn't match the field's underlying
   *  type, a runtime error will be thrown.
   *
   *  @{
   */
  StateFieldValued(ParameterBase const &param)
      : _value(param.get<T>()) { }

  StateFieldValued(ParameterBase &&param)
      : _value(std::move(param.get<T>())) { }
  /** @}
   */

  /** @return Reference to the underlying value.
   *
   *  @{
   */
  virtual operator T const & () const override {
    return _value;
  }

  virtual operator T & () override {
    return _value;
  }
  /** @}
   */
};
}  // namespace psim

#endif

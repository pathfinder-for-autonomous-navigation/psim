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

#include <psim/core/nameable.hpp>
#include <psim/core/state_field.hpp>

#include <string>
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
  T _value;

  virtual T const &_get() const override {
    return _value;
  }

  virtual T &_get() override {
    return _value;
  }

 public:
  StateFieldValued() = delete;

  virtual ~StateFieldValued() = default;

  /** @brief Default constructs the parameter's initial value.
   *
   *  @param[in] name State field's name.
   *
   *  https://stackoverflow.com/questions/2417065/does-the-default-constructor-initialize-built-in-types
   *
   *  @{
   */
  StateFieldValued(std::string const &name)
    : Nameable(name, "state_field_valued"), _value() {}

  StateFieldValued(std::string &&name)
    : Nameable(std::move(name), "state_field_valued"), _value() {}
  /** @}
   */

  /** @param[in] name State field's name.
   *  @param[in] value Initial value of the state field.
   *
   *  @{
   */
  StateFieldValued(std::string const &name, T const &value)
    : Nameable(name, "state_field_valued"), _value(value) {}

  StateFieldValued(std::string &&name, T const &value)
    : Nameable(std::move(name), "state_field_valued"), _value(value) {}

  StateFieldValued(std::string const &name, T &&value)
    : Nameable(name, "state_field_valued"), _value(std::move(value)) {}

  StateFieldValued(std::string &&name, T &&value)
    : Nameable(std::move(name), "state_field_valued"),
      _value(std::move(value)) {}
  /** @}
   */

  /** @return Reference to the underlying type.
   *
   *  This function was re-implemented to avoid potential type checking overhead
   *  and keep a consistant interface.
   *
   *  @{
   */
  T const &get() const {
    return _get();
  }

  T &get() {
    return _get();
  }
  /** @}
   */
};
} // namespace psim

#endif

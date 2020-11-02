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

/** @file psim/core/state_field_lazy.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_STATE_FIELD_LAZY_HPP_
#define PSIM_CORE_STATE_FIELD_LAZY_HPP_

#include <psim/core/nameable.hpp>
#include <psim/core/state_field.hpp>

#include <functional>
#include <string>
#include <utility>

namespace psim {

/** @brief State field implemenation whose value is lazily evaluated.
 *
 *  @tparam Underlying type.
 *
 *  This field is particularly useful for fields that may be helpful when
 *  debugging or devloping but don't need to be evaluated all the time. The
 * field take a function that can later be evaluated if the field is read.
 */
template <typename T>
class StateFieldLazy : public StateField<T> {
 private:
  /** @brief Lazy evaluation function.
   */
  std::function<T()> const _function;

  /** @brief Flag specifying if the currently held value is up to date.
   */
  bool mutable _evaluated;

  /** @brief Current value (may or may not be valid).
   */
  T mutable _value;

  virtual T const &_get() const override {
    if (!_evaluated) {
      _value = _function();
      _evaluated = true;
    }
    return _value;
  }

 public:
  StateFieldLazy() = delete;

  virtual ~StateFieldLazy() = default;

  /** @param[in] name State field's name.
   *  @param[in] function Lazy evaluation function.
   *
   *  @{
   */
  StateFieldLazy(std::string const &name, std::function<T()> const &function)
    : Nameable(name, "state_field_lazy"), _function(function),
      _evaluated(false) {}

  StateFieldLazy(std::string &&name, std::function<T()> const &function)
    : Nameable(std::move(name), "state_field_lazy"), _function(function),
      _evaluated(false) {}

  StateFieldLazy(std::string const &name, std::function<T()> &&function)
    : Nameable(name, "state_field_lazy"), _function(std::move(function)),
      _evaluated(false) {}

  StateFieldLazy(std::string &&name, std::function<T()> &&function)
    : Nameable(std::move(name), "state_field_lazy"),
      _function(std::move(function)), _evaluated(false) {}
  /** @}
   */

  /** @brief Clears the current value held by the state field.
   */
  void reset() {
    _evaluated = false;
  }

  /** @return Reference to the underlying type.
   *
   *  This function was re-implemented to avoid potential type checking overhead
   *  and keep a consistant interface.
   */
  T const &get() const {
    return _get();
  }
};
} // namespace psim

#endif

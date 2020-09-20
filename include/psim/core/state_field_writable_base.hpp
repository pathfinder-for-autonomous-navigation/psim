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

/** @file psim/core/state_field_writable_base.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_STATE_FIELD_WRITABLE_BASE_HPP_
#define PSIM_CORE_STATE_FIELD_WRITABLE_BASE_HPP_

#include "state_field_base.hpp"

#include <stdexcept>

namespace psim {

/** @brief 
 */
class StateFieldWritableBase : virtual public StateFieldBase {
 protected:
  StateFieldWritableBase() = default;

 public:
  using StateFieldBase::cast_writable;

  virtual ~StateFieldWritableBase() = default;

  /** @brief Attempt to cast this field to a specific underlying type.
   *
   *  @tparam Expected underlying type.
   *
   *  @return Casted pointer.
   *
   *  This function will either succeed in casting or throw an expection if the
   *  expected underlying type does not match.
   *
   *  This function was re-implemented to avoid potential type checking overhead
   *  and keep a consistant interface.
   *
   *  @{
   */
  template <typename T>
  StateFieldWritable<T> const *cast() const {
    auto const *field_ptr = dynamic_cast<StateFieldWritable<T> const *>(this);
    if (!field_ptr)
      throw std::runtime_error("Invalid field type in call to 'cast'.");

    return field_ptr;
  }

  template <typename T>
  StateFieldWritable<T> *cast() {
    auto *field_ptr = dynamic_cast<StateFieldWritable<T> *>(this);
    if (!field_ptr)
      throw std::runtime_error("Invalid field type in call to 'cast'.");

    return field_ptr;
  }
  /** @}
   */

  /** @brief Attempt to get data from the field.
   *
   *  @tparam Expected underlying type.
   *
   *  @return Reference to the underlying type.
   *
   *  If the underlying type doesn't match the expected underlying type, a runtime
   *  error will be thrown.
   *
   *  @{
   */
  template <typename T>
  T const &get() const {
    return cast<T>()->get();
  }

  template <typename T>
  T &get() {
    return cast<T>()->get();
  }
  /** @}
   */

  /** @brief Attempt to get read write data from the field.
   *
   *  @tparam Expected underlying type.
   *
   *  @return Reference to the underlying value.
   *
   *  If the underlying type doesn't match the expected underlying type or the
   *  field isn't writable, a runtime error will be thrown.
   *
   *  This function was re-implemented to avoid potential type checking overhead
   *  and keep a consistant interface.
   */
  template <typename T>
  T &get_writable() {
    return get<T>();
  }
};
}  // namespace psim

#endif

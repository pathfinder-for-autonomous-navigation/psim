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

/** @file psim/core/state_field.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_STATE_FIELD_HPP_
#define PSIM_CORE_STATE_FIELD_HPP_

#include "state_field_base.hpp"

#include <stdexcept>
#include <type_traits>

namespace psim {

/** @brief Interface to a read only state field.
 *
 *  @tparam T Underlying type.
 *
 *  A generic state field allows reads producing data of the specified underlying
 *  type. Convenience functions to attempt casts to writable state fields are
 *  still supported.
 */
template <typename T>
class StateField : virtual public StateFieldBase {
 protected:
  StateField() = default;

 public:
  virtual ~StateField() = default;

  /** @return Constant reference to the underlying value.
   */
  virtual operator T const & () const = 0;

  /** @returns Pointer to this object.
   *
   *  This function was re-implemented to avoid potential type checking overhead
   *  and keep a consistant interface.
   *
   *  @{
   */
  template <typename U = T>
  StateField<U> const *cast() const {
    static_assert(std::is_same<U, T>::value, "Invalid field type in call to 'cast'.");
    return this;
  }

  template <typename U = T>
  StateField<U> *cast() {
    static_assert(std::is_same<U, T>::value, "Invalid field type in call to 'cast'.");
    return this;
  }
  /** @}
   */

  /** @brief Attempt to cast this field to a writable field.
   *
   *  @return Casted pointer.
   *
   *  This function will either succeed in casting or throw as exception if the
   *  field is not writable.
   *
   *  This function was re-implemented to avoid potential type checking overhead
   *  and keep a consistant interface.
   *
   *  @{
   */
  template <typename U = T>
  StateFieldWritable<U> const *cast_writable() const {
    static_assert(std::is_same<U, T>::value, "Invalid field type in call to 'cast_writable'.");

    auto const *field_ptr = dynamic_cast<StateFieldWritable<U> const *>(this);
    if (!field_ptr)
      throw std::runtime_error("Invalid field type in call to 'cast_writable'.");

    return field_ptr;
  }

  template <typename U = T>
  StateFieldWritable<U> *cast_writable() {
    static_assert(std::is_same<U, T>::value, "Invalid field type in call to 'cast_writable'.");

    auto *field_ptr = dynamic_cast<StateFieldWritable<U> *>(this);
    if (!field_ptr)
      throw std::runtime_error("Invalid field type in call to 'cast_writable'.");

    return field_ptr;
  }
  /** @}
   */

  /** @return Constant reference to the underlying value.
   *
   *  This function was re-implemented to avoid potential type checking overhead
   *  and keep a consistant interface.
   */
  template <typename U = T>
  U const &get() const {
    static_assert(std::is_same<U, T>::value, "Invalid field type in call to 'get'.");
    return (U const &) *this;
  }

  /** @brief Attempt to get read write data from the field.
   *
   *  @tparam Expected underlying type.
   *
   *  @return Reference to the underlying value.
   *
   *  If the underlying type doesn't match the expected underlying type or the
   *  field isn't writable, a runtime error will be thrown.
   */
  template <typename U = T>
  T &get_writable() {
    return cast_writable<T>()->get_writable();
  }
};
}  // namespace psim

#endif

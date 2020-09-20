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

/** @file psim/core/state_field_writable.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_STATE_FIELD_WRITABLE_HPP_
#define PSIM_CORE_STATE_FIELD_WRITABLE_HPP_

#include "parameter.hpp"
#include "state_field.hpp"
#include "state_field_writable_base.hpp"

#include <type_traits>
#include <utility>

namespace psim {

/** @brief Interface to a read write state field
 *
 *  @tparam T Underlying type.
 *
 *  A writable state field allows reads or writes to or from the fields as long as
 *  they are of the underlying data type.
 */
template <typename T>
class StateFieldWritable : public StateFieldWritableBase, public StateField<T> {
 protected:
  StateFieldWritable() = default;

 public:
  virtual ~StateFieldWritable() = default;

  /** @return Reference to the underlying value.
   */
  virtual operator T & () = 0;

  /** @brief Assign a value to this state field.
   *
   *  @param[in] value Value assigned.
   *
   *  @return Reference to this state field.
   *
   *  @{
   */
  StateFieldWritable<T> &operator=(T const &value) {
    (T &) *this = value;
    return *this;
  }

  StateFieldWritable<T> &operator=(T &&value) {
    (T &) *this = std::move(value);
    return *this;
  }
  /** @}
   */

  /** @returns Pointer to this object.
   *
   *  This function was re-implemented to avoid potential type checking overhead
   *  and keep a consistant interface.
   *
   *  @{
   */
  template <typename U = T>
  StateFieldWritable<U> const *cast() const {
    static_assert(std::is_same<U, T>::value, "Invalid field type in call to 'cast'.");
    return this;
  }

  template <typename U = T>
  StateFieldWritable<U> *cast() {
    static_assert(std::is_same<U, T>::value, "Invalid field type in call to 'cast'.");
    return this;
  }

  template <typename U = T>
  StateFieldWritable<U> const *cast_writable() const {
    return cast<T>();
  }

  template <typename U = T>
  StateFieldWritable<U> *cast_writable() {
    return cast<T>();
  }
  /** @}
   */

  /** @return Reference to the underlying type.
   *
   *  This function was re-implemented to avoid potential type checking overhead
   *  and keep a consistant interface.
   *
   *  @{
   */
  template <typename U = T>
  U const &get() const {
    static_assert(std::is_same<U, T>::value, "Invalid field type in call to 'get'.");
    return (U const &) *this;
  }

  template <typename U = T>
  U &get() {
    static_assert(std::is_same<U, T>::value, "Invalid field type in call to 'get'.");
    return (U &) *this;
  }

  template <typename U = T>
  U &get_writable() {
    return get<U>();
  }
  /** @}
   */
};
}  // namespace psim

#endif

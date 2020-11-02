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

#include <psim/core/castable.hpp>
#include <psim/core/castable_base.hpp>
#include <psim/core/nameable.hpp>

#include <stdexcept>
#include <type_traits>

namespace psim {

template <typename T>
class StateField;

class StateFieldWritableBase;

template <typename T>
class StateFieldWritable;

/** @brief Parent class for all state fields.
 *
 *  The main purpose of this class is to allow dynamic casting to be used to
 * check field types at runtime and provided casting functions for convenience.
 */
class StateFieldBase : public virtual Nameable,
                       public CastableBase<StateField>,
                       public CastableBase<StateFieldWritable> {
 protected:
  StateFieldBase() = default;

 public:
  using CastableBase<StateField>::cast;

  template <typename T>
  auto const &cast_writable() const {
    return CastableBase<StateFieldWritable>::template cast<T>();
  }

  template <typename T>
  auto &cast_writable() {
    return CastableBase<StateFieldWritable>::template cast<T>();
  }

  StateFieldBase(StateFieldBase const &) = delete;
  StateFieldBase(StateFieldBase &&) = delete;
  StateFieldBase &operator=(StateFieldBase const &) = delete;
  StateFieldBase &operator=(StateFieldBase &&) = delete;

  virtual ~StateFieldBase() = default;

  /** @brief Attempt to get read only data from the field.
   *
   *  @tparam Expected underlying type.
   *
   *  @return Constant reference to the underlying value.
   */
  template <typename T>
  T const &get() const {
    return cast<T>().get();
  }

  /** @brief Attempt to get read write data from the field.
   *
   *  @tparam Expected underlying type.
   *
   *  @return Reference to the underlying value.
   */
  template <typename T>
  T &get_writable() {
    return cast_writable<T>().get_writable();
  }
};

/** @brief Interface to a read only state field.
 *
 *  @tparam T Underlying type.
 *
 *  A generic state field allows reads producing data of the specified
 * underlying type. Convenience functions to attempt casts to writable state
 * fields are still supported.
 */
template <typename T>
class StateField : virtual public StateFieldBase,
                   public Castable<StateFieldWritable, T> {
 protected:
  StateField() = default;

  /** @return Constant reference to the underlying type.
   */
  virtual T const &_get() const = 0;

 public:
  template <typename U = T>
  auto const &cast_writable() const {
    return Castable<StateFieldWritable, T>::template cast<U>();
  }

  template <typename U = T>
  auto &cast_writable() {
    return Castable<StateFieldWritable, T>::template cast<U>();
  }

  virtual ~StateField() = default;

  /** @returns Pointer to this object.
   *
   *  This function was re-implemented to avoid potential type checking overhead
   *  and keep a consistant interface.
   *
   *  @{
   */
  template <typename U = T>
  StateField<U> const &cast() const {
    static_assert(std::is_same<U, T>::value,
        "Invalid type in call to StateField::cast() const");
    return *this;
  }

  template <typename U = T>
  StateField<U> &cast() {
    static_assert(std::is_same<U, T>::value,
        "Invalid type in call to StateField::cast()");
    return *this;
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
    static_assert(std::is_same<U, T>::value,
        "Invalid type in call to StateField::get() const");
    return this->_get();
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
    return cast_writable<T>().get_writable();
  }
};

/** @brief Parent class for all writable state fields.
 *
 *  The main purpose of this class is to allow dynamic casting to be used to
 *  check field types at runtime and provided casting functions for convenience.
 */
class StateFieldWritableBase : virtual public StateFieldBase {
 protected:
  StateFieldWritableBase() = default;

 public:
  using CastableBase<StateFieldWritable>::cast;
  using StateFieldBase::cast_writable;

  virtual ~StateFieldWritableBase() = default;

  /** @brief Attempt to get data from the field.
   *
   *  @tparam Expected underlying type.
   *
   *  @return Reference to the underlying type.
   *
   *  If the underlying type doesn't match the expected underlying type, a
   * runtime error will be thrown.
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

/** @brief Interface to a read write state field
 *
 *  @tparam T Underlying type.
 *
 *  A writable state field allows reads or writes to or from the fields as long
 * as they are of the underlying data type.
 */
template <typename T>
class StateFieldWritable : public StateFieldWritableBase, public StateField<T> {
 protected:
  StateFieldWritable() = default;

  /** @return Reference to the underlying value.
   */
  virtual T &_get() = 0;

 public:
  virtual ~StateFieldWritable() = default;

  /** @returns Pointer to this object.
   *
   *  This function was re-implemented to avoid potential type checking overhead
   *  and keep a consistant interface.
   *
   *  @{
   */
  template <typename U = T>
  StateFieldWritable<U> const &cast() const {
    static_assert(std::is_same<U, T>::value,
        "Invalid type in call to StateFieldWritable::cast() const");
    return *this;
  }

  template <typename U = T>
  StateFieldWritable<U> &cast() {
    static_assert(std::is_same<U, T>::value,
        "Invalid type in call to StateFieldWritable::cast()");
    return *this;
  }

  template <typename U = T>
  StateFieldWritable<U> const &cast_writable() const {
    return cast<T>();
  }

  template <typename U = T>
  StateFieldWritable<U> &cast_writable() {
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
    static_assert(std::is_same<U, T>::value,
        "Invalid type in call to StateFieldWritable::get() const");
    return this->_get();
  }

  template <typename U = T>
  U &get() {
    static_assert(std::is_same<U, T>::value,
        "Invalid type in call to StateFieldWritable::get()");
    return this->_get();
  }

  template <typename U = T>
  U &get_writable() {
    return get<U>();
  }
  /** @}
   */
};
} // namespace psim

#endif

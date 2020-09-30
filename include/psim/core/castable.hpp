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

/** @file psim/core/castable.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_CASTABLE_HPP_
#define PSIM_CORE_CASTABLE_HPP_

#include <psim/core/nameable.hpp>

#include <stdexcept>
#include <type_traits>

namespace psim {

/** @brief Casting utility for nameable objects with 'underlying' types.
 *
 *  @tparam D<T> Expected derived type.
 *
 *  This is used to help with state field and parameter casting. The difference
 *  between this implementation and castable base is we assume the type her is
 *  fixed.
 */
template <template <typename> class D, typename T>
class Castable : public virtual Nameable {
 protected:
  Castable() = default;

 public:
  virtual ~Castable() = default;

  /** @brief Attempt to cast to an expected derived type.
   *
   *  @tparam U Expected underlying type.
   *
   *  @return Casted reference.
   *
   *  This function will either succeed in casting or throw an exception if the
   *  the derived type isn't valid. The expected and actual underlying types are
   *  ensured to be the same at compile time.
   *
   *  @{
   */
  template <typename U = T>
  D<U> const &cast() const {
    static_assert(std::is_same<U, T>::value, "Invalid type in call to Castable::cast() const");

    auto const *ptr = dynamic_cast<D<U> const *>(this);
    if (!ptr)
      throw std::runtime_error("Invalid call to 'Castable::cast() const' on '" + name() + ":" + type() + "'");

    return *ptr;
  }

  template <typename U = T>
  D<U> &cast() {
    static_assert(std::is_same<U, T>::value, "Invalid type in call to Castable::cast()");

    auto *ptr = dynamic_cast<D<U> *>(this);
    if (!ptr)
      throw std::runtime_error("Invalid call to 'Castable::cast()' on '" + name() + ":" + type() + "'");

    return *ptr;
  }
  /** @}
   */
};
}  // namespace psim

#endif

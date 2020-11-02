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

#ifndef PSIM_CORE_CASTABLE_BASE_HPP_
#define PSIM_CORE_CASTABLE_BASE_HPP_

#include <psim/core/nameable.hpp>

#include <stdexcept>

namespace psim {

/** @brief Casting utility for nameable objects with 'underlying' types.
 *
 *  @tparam D Expected derived type.
 *
 *  This is used to help with state field and parameter casting.
 */
template <template <typename> class D>
class CastableBase : public virtual Nameable {
 protected:
  CastableBase() = default;

 public:
  virtual ~CastableBase() = default;

  /** @brief Attempt to cast to an expected derived type.
   *
   *  @tparam T Expected underlying type.
   *
   *  @return Casted reference.
   *
   *  This function will either succeed in casting or throw an exception if the
   *  the cast fails. A failing cast may be due to an invalid derived type or
   *  invalid derived type.
   *
   *  @{
   */
  template <typename T>
  D<T> const &cast() const {
    auto const *ptr = dynamic_cast<D<T> const *>(this);
    if (!ptr)
      throw std::runtime_error(
          "Invalid call to 'CastableBase::cast() const' on '" + name() + ":" +
          type() + "'");

    return *ptr;
  }

  template <typename T>
  D<T> &cast() {
    auto *ptr = dynamic_cast<D<T> *>(this);
    if (!ptr)
      throw std::runtime_error("Invalid call to 'CastableBase::cast()' on '" +
                               name() + ":" + type() + "'");

    return *ptr;
  }
  /** @}
   */
};
} // namespace psim

#endif

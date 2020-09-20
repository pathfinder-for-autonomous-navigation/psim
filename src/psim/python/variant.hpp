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
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

/** @file psim/python/variant.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_PYTHON_VARIANT_HPP_
#define PSIM_PYTHON_VARIANT_HPP_

#include <pybind11/stl.h>

#include <mapbox/variant.hpp>

namespace pybind11 {
namespace detail {

/* This code allows us to plug the variant type into pybind11. See the following
 * link for more information:
 *  - https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html?highlight=stl
 */

template <typename... Ts>
struct type_caster<mapbox::util::variant<Ts...>> : variant_caster<mapbox::util::variant<Ts...>> {};

template <>
struct visit_helper<mapbox::util::variant> {
  template <typename... Args>
  static auto call(Args &&...args) -> decltype(mapbox::util::apply_visitor(args...)) {
    return mapbox::util::apply_visitor(args...);
  }
};
}  // namespace detail
}  // namespace pybind11

#endif

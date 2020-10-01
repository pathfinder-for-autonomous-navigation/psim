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

/** @file psim/core/nameable.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_NAMEABLE_HPP_
#define PSIM_CORE_NAMEABLE_HPP_

#include <string>
#include <utility>

namespace psim {

/** @brief Ties a string name and string type representation to an object.
 *
 *  This is useful for debugging and making useful error messages.
 */
class Nameable {
 private:
  std::string const _name, _type;

 protected:
  Nameable(std::string const &name, std::string const &type)
      : _name(name), _type(type) { }

  Nameable(std::string &&name, std::string const &type)
      : _name(std::move(name)), _type(type) { }

  Nameable(std::string const &name, std::string &&type)
      : _name(name), _type(std::move(type)) { }

  Nameable(std::string &&name, std::string &&type)
      : _name(std::move(name)), _type(std::move(type)) { }

 public:
  Nameable() = default;
  virtual ~Nameable() = default;

  /** @returns String name tied to this object.
   */
  inline std::string const &name() const {
    return _name;
  }

  /** @returns String representation of this object's type.
   */
  inline std::string const &type() const {
    return _type;
  }
};
}  // namespace psim

#endif

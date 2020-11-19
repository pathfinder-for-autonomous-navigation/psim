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

/** @file psim/core/configuration.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_CORE_CONFIGURATION_HPP_
#define PSIM_CORE_CONFIGURATION_HPP_

#include <psim/core/parameter.hpp>
#include <psim/core/types.hpp>

#include "types.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace psim {

class Configuration {
 private:
  template <typename T>
  void _add(std::string const &name, T &&value, std::string const &file,
      std::size_t l);

 protected:
  std::unordered_map<std::reference_wrapper<std::string const>,
                     std::unique_ptr<ParameterBase const>,
                     std::hash<std::string>, std::equal_to<std::string>>
                     _parameters;

  Configuration() = default;

  void _parse(std::string const &file);

 public:
  Configuration(Configuration const &) = delete;
  Configuration & operator=(Configuration const &) = delete;

  Configuration(Configuration &&) = default;
  Configuration &operator=(Configuration &&) = default;

  virtual ~Configuration() = default;

  /** @brief
   *
   *  @param[in] file
   */
  Configuration(std::string const &file);

  /** @brief
   *
   *  @param[in] files
   */
  Configuration(std::vector<std::string> const &files);

  /** @brief Retrives a parameter by name.
   *
   *  @param[in] name
   *
   *  @return Pointer to the parameter.
   *
   *  If no parameter is found by the specified name, a null pointer is
   * returned.
   */
  ParameterBase const *get(std::string const &name) const;

  /** @brief Retrives a parameter by name.
   *
   *  @param[in] name
   *
   *  @return Pointer to the parameter.
   *
   *  If no parameter is found by the specified name, a runtime error is thrown.
   */
  ParameterBase const &operator[](std::string const &name) const;
};
} // namespace psim

#endif

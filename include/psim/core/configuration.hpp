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

/** @brief Represents a set of parameters used to initialize a simulation.
 *
 *  A configuration is a set of parameters parsed from a configuration file.
 *  These parameters are made accesible to the simulation's models during
 *  initialization to specify initial conditions and configure constants.
 */
class Configuration {
 private:
  template <typename T>
  void _add(std::string const &name, T &&value, std::string const &file,
      std::size_t l);

 protected:
  /** @brief Map containing the parameter pointers.
   *
   *  https://stackoverflow.com/questions/9139748/using-stdreference-wrapper-as-the-key-in-a-stdmap
   */
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

  /** @brief Parses a configuration file into a configuration.
   *
   *  @param[in] file Configuration file.
   *
   *  If the configuration file doesn't exist, a line with invalid formatting is
   *  found, or a particular key is specified twice, a runtime error will be
   *  thrown.
   */
  Configuration(std::string const &file);

  /** @brief Parses a set of configuration files into a configuration.
   *
   *  @param[in] files Set of configuration file names.
   *
   *  If one of the configuration files don't exist, a line with invalid
   *  formatting is found, or a particular key is specified twice, a runtime
   *  error will be thrown.
   */
  Configuration(std::vector<std::string> const &files);

  /** @brief Retrives a parameter by name.
   *
   *  @param[in] name
   *
   *  @return Pointer to the parameter.
   *
   *  If no parameter is found by the specified name, a null pointer is
   *  returned.
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

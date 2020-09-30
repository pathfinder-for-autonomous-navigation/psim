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

/** @file psim/core/configuration.cpp
 *  @author Kyle Krol
 */

#include <psim/core/configuration.hpp>

#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace psim {

template <typename T>
void Configuration::_add(std::string const &name, T &&value,
    std::string const &file, std::size_t l) {
  auto const iter = _parameters.find(name);
  if (iter != _parameters.end())
    throw std::runtime_error(
        "Error on line + " + std::to_string(l) + " while parsing " + file +
        "as a configuration file.\n" +
        "Duplicate parameter name '" + name + "'."
      );

  auto const *param = new Parameter<T>(name, std::forward<T>(value));
  _parameters[param->name()] = param;
}

void Configuration::_parse(std::string const &file) {
  // Attempt to open the configuration file
  std::ifstream ifs(file);
  if (!ifs.is_open())
    throw std::runtime_error(
        "Error while parsing '" + file + "' as a configuration file.\n" +
        "Unable to open file. This is most likely because the file cannot be" +
        "found."
      );

  // Parse line by line
  std::string line;
  std::size_t l = 0;
  while (std::getline(ifs, line)) {
    l++;

    // Lines starting with '#' are considered comments
    if (line[0] == '#')
      continue;

    std::istringstream iss(line);
    std::vector<std::string> const tokens{
      std::istream_iterator<std::string>{iss},
      std::istream_iterator<std::string>{}
    };

    // Ignore lines only containing whitespace
    if (tokens.size() == 0)
      continue;

    // Ensure a valid parameter name
    {
      static std::regex re("[A-Za-z][A-Za-z_0-9\\.]*");

      if (!std::regex_match(tokens[0], re))
        throw std::runtime_error(
            "Error on line " + std::to_string(l) + " while parsing '" + file +
            "' as a configuration file.\n" +
            "Invalid configuration parameter name '" + tokens[0] + "'. " +
            "Parameter names must match the following regular expression: " +
            "'[A-Za-z][A-Za-z_0-9\\\\.]*'."
          );
    }

    try {
      switch (tokens.size()) {
        // Require a parameter value
        case 1:
          throw std::runtime_error(
              "Error on line " + std::to_string(l) + " while parsing '" + file +
              "' as a configuration file.\n" +
              "Only one token detected in the following line: '" + line + "'"
            );

        case 2:
          if (tokens[1].find('.') == std::string::npos)
            _add(tokens[0], std::stol(tokens[1]), file, l);
          else
            _add(tokens[0], std::stod(tokens[1]), file, l);
          break;

        case 3:
          _add(
            tokens[0],
            Vector2({std::stod(tokens[1]), std::stod(tokens[2])}),
            file, l
          );
          break;

        case 4:
          _add(
            tokens[0],
            Vector3({std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3])}),
            file, l
          );
          break;

        case 5:
          _add(
            tokens[0],
            Vector4({std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4])}),
            file, l
          );
          break;

        // Only up to four dimensional vector can be specified
        default:
          throw std::runtime_error(
              "Error on line " + std::to_string(l) + " while parsing '" + file +
              "' as a configuration file.\n" +
              "More than five token detected in the following line: '" + line +
              "'"
            );
      }
    } catch (std::logic_error const &e) {
      // Reinterpret errors potentially thrown by 'std::stod' and 'std::stol'.
      throw std::runtime_error(
          "Error on line " + std::to_string(l) + " while parsing '" + file +
          "' as a configuration file.\n" + e.what()
        );
    }
  }
}

Configuration::Configuration(Configuration &&config)
    : _parameters(std::move(config._parameters)) { }

Configuration &Configuration::operator=(Configuration &&config) {
  _parameters = std::move(config._parameters);
  return *this;
}

Configuration::~Configuration() {
  for (auto &parameter : _parameters)
    delete parameter.second;
}

ParameterBase const &Configuration::operator[](std::string const &name) const {
  auto const *parameter_ptr = this->get(name);
  if (!parameter_ptr)
    throw std::runtime_error("Parameter not found with name: " + name);

  return *parameter_ptr;
}

ParameterBase const *Configuration::get(std::string const &name) const {
  auto const iter = _parameters.find(name);
  return (iter == _parameters.end() ? nullptr : iter->second);
}

Configuration Configuration::make(std::string const &file) {
  Configuration config;
  config._parse(file);
  return config;
}

Configuration Configuration::make(std::vector<std::string> const &files) {
  Configuration config;
  for (auto const &file : files)
    config._parse(file);
  return config;
}
}  // namespace psim

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
#include <vector>

namespace psim {

template <typename T>
void Configuration::add(std::string const &name, T &&value) {
  auto const iter = _parameters.find(name);
  if (iter != _parameters.end())
    throw std::runtime_error("Two or more parameters under the name: " + name);

  _parameters[name] = new Parameter<T>(std::forward<T>(value));
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

  std::ifstream ifs(file);
  if (!ifs.is_open())
    throw std::runtime_error("Unable to open parameter file: " + std::string(file));

  std::string line;
  while (std::getline(ifs, line)) {
    if (line[0] == '#')
      continue;

    std::istringstream iss(line);
    std::vector<std::string> const tokens{
      std::istream_iterator<std::string>{iss},
      std::istream_iterator<std::string>{}
    };

    // Ignore empty lines
    if (tokens.size() == 0)
      continue;

    // Ensure a valid parameter name
    {
      static std::regex re("[A-Za-z][A-Za-z_0-9\\.]*");

      if (!std::regex_match(tokens[0], re))
        throw std::runtime_error("Invalid parameter name: " + tokens[0]);
    }

    switch (tokens.size()) {
      // Require a parameter value
      case 1:
        throw std::runtime_error("More than one token expected per line.");

      case 2:
        if (tokens[1].find('.') == std::string::npos)
          config.add(tokens[0], std::stol(tokens[1]));
        else
          config.add(tokens[0], std::stod(tokens[1]));
        break;

      case 3:
        config.add(
          tokens[0],
          Vector2({std::stod(tokens[1]), std::stod(tokens[2])})
        );
        break;

      case 4:
        config.add(
          tokens[0],
          Vector3({std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3])})
        );
        break;

      case 5:
        config.add(
          tokens[0],
          Vector4({std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]), std::stod(tokens[4])})
        );
        break;

      // Only up to four dimensional vector can be specified
      default:
        throw std::runtime_error("No more than five tokens expected per line");
    }
  }

  return config;
}
}  // namespace psim

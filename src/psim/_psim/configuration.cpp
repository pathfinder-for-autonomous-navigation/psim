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

/** @file python/psim/_psim/configuration.hpp
 *  @author Kyle Krol
 */

#include "pybind11.hpp"

#include <psim/core/configuration.hpp>
#include <psim/core/parameter.hpp>
#include <psim/core/types.hpp>

namespace py = pybind11;

#include <string>

using PyVariant = mapbox::util::variant<psim::Integer, psim::Real, psim::Vector2, psim::Vector3, psim::Vector4>;

static PyVariant py_visit(psim::ParameterBase const &field) {
  // TODO : Update this to use double dispatch or something else that's better
  {
    auto const *ptr = dynamic_cast<psim::Parameter<psim::Integer> const *>(&field);
    if (ptr) return ptr->get();
  }
  {
    auto const *ptr = dynamic_cast<psim::Parameter<psim::Real> const *>(&field);
    if (ptr) return ptr->get();
  }
  {
    auto const *ptr = dynamic_cast<psim::Parameter<psim::Vector2> const *>(&field);
    if (ptr) return ptr->get();
  }
  {
    auto const *ptr = dynamic_cast<psim::Parameter<psim::Vector3> const *>(&field);
    if (ptr) return ptr->get();
  }
  {
    auto const *ptr = dynamic_cast<psim::Parameter<psim::Vector4> const *>(&field);
    if (ptr) return ptr->get();
  }
  throw std::runtime_error("Attempted to retrieve state field of an unsupported type.");
}

void py_configuration(py::module &m) {
  py::class_<psim::Configuration>(m, "Configuration")
    .def(py::init([](std::string const &file) { return psim::Configuration::make(file); }))
    .def(py::init([](std::vector<std::string> const &files) { return psim::Configuration::make(files); }))
    .def("__getitem__", [](psim::Configuration const &self, std::string const &name) { return py_visit(self[name]); });
}

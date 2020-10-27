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

/** @file python/psim/_psim/simulations.hpp
 *  @author Kyle Krol
 */

#include "pybind11.hpp"

#include <psim/core/simulation.hpp>
#include <psim/core/state_field.hpp>
#include <psim/core/types.hpp>
#include <psim/simulations/dual_attitude_orbit.hpp>
#include <psim/simulations/dual_orbit.hpp>
#include <psim/simulations/single_attitude_orbit.hpp>
#include <psim/simulations/single_orbit.hpp>

#include <lin/core.hpp>

namespace py = pybind11;

using PyVariant = mapbox::util::variant<psim::Integer, psim::Real, psim::Vector2, psim::Vector3, psim::Vector4>;

static PyVariant py_visit(psim::StateFieldBase const &field) {
  // TODO : Update this to use double dispatch or something else that's better
  {
    auto const *ptr = dynamic_cast<psim::StateField<psim::Integer> const *>(&field);
    if (ptr) return ptr->get();
  }
  {
    auto const *ptr = dynamic_cast<psim::StateField<psim::Real> const *>(&field);
    if (ptr) return ptr->get();
  }
  {
    auto const *ptr = dynamic_cast<psim::StateField<psim::Vector2> const *>(&field);
    if (ptr) return ptr->get();
  }
  {
    auto const *ptr = dynamic_cast<psim::StateField<psim::Vector3> const *>(&field);
    if (ptr) return ptr->get();
  }
  {
    auto const *ptr = dynamic_cast<psim::StateField<psim::Vector4> const *>(&field);
    if (ptr) return ptr->get();
  }
  throw std::runtime_error("Attempted to retrieve state field of an unsupported type.");
}

template <typename T>
static void py_assign(psim::StateFieldWritableBase &field, T const &value) {
  auto *ptr = dynamic_cast<psim::StateFieldWritable<T> *>(&field);
  if (!ptr)
    std::runtime_error("Attempted to write to state field with incorrect type.");
  ptr->get() = value;
}

template <class C>
class PySimulation : public psim::Simulation {
 public:
  PySimulation() = delete;
  virtual ~PySimulation() = default;

  template <typename... Ts>
  PySimulation(psim::Configuration const &config, Ts &&... ts)
    : Simulation(std::make_unique<C>(config, std::forward<Ts>(ts)...)) { }
};

#define PY_SIMULATION(model) \
    py::class_<PySimulation<psim::model>>(m, #model) \
      .def(py::init([](psim::Configuration const &config) { \
        return new PySimulation<psim::model>(config); \
      })) \
      .def("__getitem__", [](PySimulation<psim::model> const &self, std::string const &name) { \
        auto const *ptr = self.get(name); \
        if (!ptr) \
          throw std::runtime_error("State field '" + name + "' does not exist."); \
        return py_visit(*ptr); \
      }) \
      .def("__setitem__", [](PySimulation<psim::model> &self, std::string const &name, PyVariant const &value) { \
        auto *ptr = self.get_writable(name); \
        if (!ptr) \
          throw std::runtime_error("Writable state field '" + name + "' does not exist."); \
        value.match( \
          [&ptr](psim::Real    const &v) { py_assign(*ptr, v); }, \
          [&ptr](psim::Integer const &v) { py_assign(*ptr, v); }, \
          [&ptr](psim::Vector2 const &v) { py_assign(*ptr, v); }, \
          [&ptr](psim::Vector3 const &v) { py_assign(*ptr, v); }, \
          [&ptr](psim::Vector4 const &v) { py_assign(*ptr, v); } \
        ); \
      }) \
      .def("step", [](PySimulation<psim::model> &self) { \
        self.step(); \
      })

void py_simulation(py::module &m) {
  PY_SIMULATION(SingleAttitudeOrbitGnc);
  PY_SIMULATION(SingleOrbitGnc);
  PY_SIMULATION(DualAttitudeOrbitGnc);
  PY_SIMULATION(DualOrbitGnc);
}

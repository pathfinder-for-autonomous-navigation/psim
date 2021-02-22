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

/** @file python/psim/_psim.cpp
 *  @author Kyle Krol
 */

#include <mapbox/variant.hpp>

#include <psim/core/configuration.hpp>
#include <psim/core/parameter.hpp>
#include <psim/core/simulation.hpp>
#include <psim/core/state_field.hpp>
#include <psim/core/types.hpp>

#include <psim/simulations/attitude_estimator_test.hpp>
#include <psim/simulations/dual_attitude_orbit.hpp>
#include <psim/simulations/dual_orbit.hpp>
#include <psim/simulations/orbit_estimator_test.hpp>
#include <psim/simulations/relative_orbit_estimator_test.hpp>
#include <psim/simulations/single_attitude_orbit.hpp>
#include <psim/simulations/single_orbit.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iostream>

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

using PyVariant = mapbox::util::variant<psim::Integer, psim::Real, psim::Vector2, psim::Vector3, psim::Vector4>;

namespace py = pybind11;

class PyConfiguration : public psim::Configuration {
 private:
  template <typename T>
  void _set(std::string const &name, T const &value) {
    _parameters[name] = std::make_unique<psim::Parameter<T>>(name, value);
  }

 public:
  using psim::Configuration::Configuration;

  virtual ~PyConfiguration() = default;

  PyVariant get(std::string const &name) const {
    auto const *param = this->psim::Configuration::get(name);
    if (!param)
      throw std::runtime_error("Parameter '" + name + "' does not exist.");
    {
      auto const *ptr = dynamic_cast<psim::Parameter<psim::Integer> const *>(param);
      if (ptr) return ptr->get();
    }
    {
      auto const *ptr = dynamic_cast<psim::Parameter<psim::Real> const *>(param);
      if (ptr) return ptr->get();
    }
    {
      auto const *ptr = dynamic_cast<psim::Parameter<psim::Vector2> const *>(param);
      if (ptr) return ptr->get();
    }
    {
      auto const *ptr = dynamic_cast<psim::Parameter<psim::Vector3> const *>(param);
      if (ptr) return ptr->get();
    }
    {
      auto const *ptr = dynamic_cast<psim::Parameter<psim::Vector4> const *>(param);
      if (ptr) return ptr->get();
    }
    throw std::runtime_error("Parameter '" + name + "' holds an unsupported type.");
  }

  void set(std::string const &name, PyVariant const &value) {
    value.match(
      [this, &name](psim::Real    const &v) { _set(name, v); },
      [this, &name](psim::Integer const &v) { _set(name, v); },
      [this, &name](psim::Vector2 const &v) { _set(name, v); },
      [this, &name](psim::Vector3 const &v) { _set(name, v); },
      [this, &name](psim::Vector4 const &v) { _set(name, v); }
    );
  }
};

void py_configuration(py::module &m) {
  py::class_<PyConfiguration>(m, "Configuration")
    .def(py::init([]() { return new PyConfiguration; }))
    .def(py::init([](std::string const &file) { return new PyConfiguration(file); }))
    .def(py::init([](std::vector<std::string> const &files) { return new PyConfiguration(files); }))
    .def("__getitem__", [](PyConfiguration const &self, std::string const &name) { return self.get(name); })
    .def("__setitem__", [](PyConfiguration &self, std::string const &name, PyVariant const &value) { self.set(name, value); });
}

template <typename T>
static void py_assign(psim::StateFieldWritableBase &field, T const &value) {
  auto *ptr = dynamic_cast<psim::StateFieldWritable<T> *>(&field);
  if (!ptr)
    std::runtime_error("Attempted to write to '" + field.name() + "' but the underlying type was incorrect.");
  ptr->get() = value;
}

#define PY_SIMULATION(model) \
    py::class_<psim::Simulation<psim::model>>(m, #model) \
      .def(py::init([](PyConfiguration const &config) { \
        return new psim::Simulation<psim::model>(config); \
      })) \
      .def("__getitem__", [](psim::Simulation<psim::model> const &self, std::string const &name) -> PyVariant { \
        auto const *field = self.get(name); \
        if (!field) \
          throw std::runtime_error("State field '" + name + "' does not exist."); \
        { \
            auto const *ptr = dynamic_cast<psim::StateField<psim::Integer> const *>(field); \
            if (ptr) return ptr->get(); \
        } \
        { \
            auto const *ptr = dynamic_cast<psim::StateField<psim::Real> const *>(field); \
            if (ptr) return ptr->get(); \
        } \
        { \
            auto const *ptr = dynamic_cast<psim::StateField<psim::Vector2> const *>(field); \
            if (ptr) return ptr->get(); \
        } \
        { \
            auto const *ptr = dynamic_cast<psim::StateField<psim::Vector3> const *>(field); \
            if (ptr) return ptr->get(); \
        } \
        { \
            auto const *ptr = dynamic_cast<psim::StateField<psim::Vector4> const *>(field); \
            if (ptr) return ptr->get(); \
        } \
        throw std::runtime_error("State field '" + name + "' holds an unsupported type."); \
      }) \
      .def("__setitem__", [](psim::Simulation<psim::model> &self, std::string const &name, PyVariant const &value) { \
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
      .def("step", [](psim::Simulation<psim::model> &self) { \
        self.step(); \
      })

void py_simulation(py::module &m) {
  PY_SIMULATION(AttitudeEstimatorTestGnc);
  PY_SIMULATION(SingleAttitudeOrbitGnc);
  PY_SIMULATION(SingleOrbitGnc);
  PY_SIMULATION(OrbOrbitEstimatorTest);
  PY_SIMULATION(RelativeOrbitEstimatorTest);
  PY_SIMULATION(DualAttitudeOrbitGnc);
  PY_SIMULATION(DualOrbitGnc);
}

PYBIND11_MODULE(_psim, m) {
  py_configuration(m);
  py_simulation(m);
}

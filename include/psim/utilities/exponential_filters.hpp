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

/** @file psim/utilities/exponential_filters.hpp
 *  @author Kyle Krol
 */

#ifndef PSIM_UTILITIES_EXPONENTIAL_FILTERS_HPP_
#define PSIM_UTILITIES_EXPONENTIAL_FILTERS_HPP_

#include <psim/utilities/exponential_filter_real.yml.hpp>
#include <psim/utilities/exponential_filter_vector2.yml.hpp>
#include <psim/utilities/exponential_filter_vector3.yml.hpp>
#include <psim/utilities/exponential_filter_vector4.yml.hpp>

namespace psim {

class ExponentialFilterReal
  : public ExponentialFilterRealInterface<ExponentialFilterReal> {
 private:
  using Super = ExponentialFilterRealInterface<ExponentialFilterReal>;

 public:
  using Super::ExponentialFilterRealInterface;

  ExponentialFilterReal() = delete;
  virtual ~ExponentialFilterReal() = default;

  virtual void add_fields(State &state) override;
  virtual void step() override;
};

class ExponentialFilterVector2
  : public ExponentialFilterVector2Interface<ExponentialFilterVector2> {
 private:
  using Super = ExponentialFilterVector2Interface<ExponentialFilterVector2>;

 public:
  using Super::ExponentialFilterVector2Interface;

  ExponentialFilterVector2() = delete;
  virtual ~ExponentialFilterVector2() = default;

  virtual void add_fields(State &state) override;
  virtual void step() override;
};

class ExponentialFilterVector3
  : public ExponentialFilterVector3Interface<ExponentialFilterVector3> {
 private:
  using Super = ExponentialFilterVector3Interface<ExponentialFilterVector3>;

 public:
  using Super::ExponentialFilterVector3Interface;

  ExponentialFilterVector3() = delete;
  virtual ~ExponentialFilterVector3() = default;

  virtual void add_fields(State &state) override;
  virtual void step() override;
};

class ExponentialFilterVector4
  : public ExponentialFilterVector4Interface<ExponentialFilterVector4> {
 private:
  using Super = ExponentialFilterVector4Interface<ExponentialFilterVector4>;

 public:
  using Super::ExponentialFilterVector4Interface;

  ExponentialFilterVector4() = delete;
  virtual ~ExponentialFilterVector4() = default;

  virtual void add_fields(State &state) override;
  virtual void step() override;
};
} // namespace psim

#endif

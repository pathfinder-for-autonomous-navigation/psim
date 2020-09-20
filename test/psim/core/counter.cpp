/** @file test/psim/core/counter.cpp
 *  @author Kyle Krol
 */

#include "counter.hpp"

Counter::Counter(psim::Configuration const &config)
    : _dn(config["dn"]), _n(config["n"]) { }

void Counter::add_fields(psim::State &state) {
  this->psim::Model::add_fields(state);

  state.add_writable("dn", &_dn);
  state.add("n", &_n);
}

void Counter::step() {
  this->psim::Model::step();

  _n += (psim::Integer) _dn;
}

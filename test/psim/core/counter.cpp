/** @file test/psim/core/counter.cpp
 *  @author Kyle Krol
 */

#include "counter.hpp"

Counter::Counter(psim::Configuration const &config)
    : _dn("dn", config["dn"].get<psim::Integer>()), _n("n", config["n"].get<psim::Integer>()) { }

void Counter::add_fields(psim::State &state) {
  this->psim::Model::add_fields(state);

  state.add_writable(&_dn);
  state.add(&_n);
}

void Counter::step() {
  this->psim::Model::step();

  _n.get() += _dn.get();
}

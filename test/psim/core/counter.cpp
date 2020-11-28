/** @file test/psim/core/counter.cpp
 *  @author Kyle Krol
 */

#include "counter.hpp"

Counter::Counter(psim::RandomsGenerator &randoms, psim::Configuration const &config)
  : Model(randoms),
    _dn("dn", config["dn"].template get<psim::Integer>()),
    _n("n", config["n"].template get<psim::Integer>()) {}

void Counter::add_fields(psim::State &state) {
  this->psim::Model::add_fields(state);

  state.add_writable(&_dn);
  state.add(&_n);
}

void Counter::step() {
  this->psim::Model::step();

  _n.get() += _dn.get();
}

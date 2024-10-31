// Silisizer, Operator sizer
// Copyright (c) 2024, Silimate Inc.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
#include "Silisizer.h"

#include <iostream>

#include "sta/Network.hh"
#include "sta/PathEnd.hh"
#include "sta/PortDirection.hh"
#include "sta/Sta.hh"

namespace SILISIZER {

int Silisizer::silisize() {
  sta::Network* network = this->network();
  std::cout << "Network: " << network << std::endl;
  sta::Instance* top_inst = network->topInstance();
  std::cout << "topInstance: " << top_inst << std::endl;

  sta::PathEndSeq ends = sta_->findPathEnds(
      /*exception from*/ nullptr, /*exception through*/ nullptr,
      /*exception to*/ nullptr, /*unconstrained*/ false, /*corner*/ nullptr,
      sta::MinMaxAll::all(),
      /*group_count*/ 10000, /*endpoint_count*/ 1, /*unique_pins*/ true,
      /* min_slack */ -1.0e+30, /*max_slack*/ 1.0e+30,
      /*sort_by_slack*/ true,
      /*groups->size() ? groups :*/ nullptr,
      /*setup*/ true, /*hold*/ false,
      /*recovery*/ false, /*removal*/ false,
      /*clk_gating_setup*/ false, /*clk_gating_hold*/ false);

  for (sta::PathEnd* pathend : ends) {
    sta::Path* path = pathend->path();
    sta::Pin* pin = path->pin(this);
    std::cout << "End Violation at: " << network->name(pin) << std::endl;
    sta::PathRef p;
    path->prevPath(this, p);
    while (!p.isNull()) {
      pin = p.pin(this);
      sta::Instance* inst = network->instance(pin);
      std::cout << " From: " << network->name(inst) << " / "
                << network->name(pin) << std::endl;
      p.prevPath(this, p);
    }
  }

  sta::InstancePinIterator* pin_iter = network->pinIterator(top_inst);
  while (pin_iter->hasNext()) {
    sta::Pin* pin = pin_iter->next();
    if (network->direction(pin)->isAnyOutput())
      std::cout << "Output pins: " << network->name(pin) << std::endl;
  }

  return 0;
}

}  // namespace SILISIZER

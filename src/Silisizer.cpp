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
#include <regex>

#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/PathEnd.hh"
#include "sta/PortDirection.hh"
#include "sta/Sta.hh"

namespace SILISIZER {

int Silisizer::silisize() {
  sta::Network* network = this->network();

  bool debug = 0;

  while (1) {
    sta::PathEndSeq ends = sta_->findPathEnds(
        /*exception from*/ nullptr, /*exception through*/ nullptr,
        /*exception to*/ nullptr, /*unconstrained*/ false, /*corner*/ nullptr,
        sta::MinMaxAll::max(),
        /*group_count*/ 10000, /*endpoint_count*/ 1, /*unique_pins*/ true,
        /* min_slack */ -1.0e+30, /*max_slack*/ 1.0e+30,
        /*sort_by_slack*/ true,
        /*groups->size() ? groups :*/ nullptr,
        /*setup*/ true, /*hold*/ false,
        /*recovery*/ false, /*removal*/ false,
        /*clk_gating_setup*/ false, /*clk_gating_hold*/ false);
    bool moreOptNeeded = !ends.empty();

    std::unordered_map<sta::Instance*, int> offendingInstCount;

    for (sta::PathEnd* pathend : ends) {
      sta::Path* path = pathend->path();
      sta::Pin* pin = path->pin(this);
      if (debug)
        std::cout << "End Violation at: " << network->name(pin) << std::endl;
      sta::PathRef p;
      path->prevPath(this, p);
      //float slack = p.slack(this);
      //if (slack >= 0.0)
      //  continue;
      while (!p.isNull()) {
        pin = p.pin(this);
        sta::Instance* inst = network->instance(pin);
        if (offendingInstCount.find(inst) == offendingInstCount.end()) {
          offendingInstCount.emplace(inst, 1);
        } else {
          offendingInstCount.find(inst)->second++;
        }
        if (debug)
          std::cout << " From: " << network->name(inst) << " / "
                    << network->name(pin) << std::endl;
        p.prevPath(this, p);
      }
    }

    sta::Instance* offender = nullptr;
    int highestCount = 0;
    for (auto pair : offendingInstCount) {
      if (pair.second > highestCount) {
        sta::Cell* cell = network->cell(pair.first);
        // If the instance does not have a cell, skip
        if (!cell) {
          continue;
        }
        sta::LibertyCell* libcell = network->libertyCell(cell);
        // If it's not a liberty cell (module), skip
        if (!libcell) continue;
        std::string libcellname = libcell->name();
        // If it's not a speed0 cell, skip
        if (libcellname.find("_sp0_") == std::string::npos) continue;
        highestCount = pair.second;
        offender = pair.first;
      }
    }

    if (offender == nullptr) {
      std::cout << "Silisizer optimization done!" << std::endl;
      break;
    }
    sta::Cell* cell = network->cell(offender);
    sta::LibertyLibrary* library = network->libertyLibrary(offender);
    sta::LibertyCell* libcell = network->libertyCell(cell);
    std::string from_cell_name = libcell->name();
    if (debug)
      std::cout << "Fixing: " << network->name(offender)
                << " of type: " << libcell->name() << std::endl;
    std::string to_cell_name =
        std::regex_replace(from_cell_name, std::regex("_sp0_"), "_sp1_");
    sta::LibertyCell* to_cell = library->findLibertyCell(to_cell_name.c_str());

    if (!to_cell) {
      std::cout << "Silisizer optimization done!" << std::endl;
      break;
    }
    Sta::sta()->replaceCell(offender, to_cell);

    if (!moreOptNeeded) {
      std::cout << "Silisizer optimization done!" << std::endl;
      break;
    }
  }
  return 0;
}

}  // namespace SILISIZER

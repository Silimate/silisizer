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
  int timing_group_count = 10;
  int end_point_count = 10;

  bool debug = 0;

  while (1) {
    sta::PathEndSeq ends = sta_->findPathEnds(
        /*exception from*/ nullptr, /*exception through*/ nullptr,
        /*exception to*/ nullptr, /*unconstrained*/ false, /*corner*/ nullptr,
        sta::MinMaxAll::max(),
        /*group_count*/ timing_group_count, /*endpoint_count*/ end_point_count,
        /*unique_pins*/ true,
        /* min_slack */ -1.0e+30, /*max_slack*/ 1.0e+30,
        /*sort_by_slack*/ true,
        /*groups->size() ? groups :*/ nullptr,
        /*setup*/ true, /*hold*/ false,
        /*recovery*/ false, /*removal*/ false,
        /*clk_gating_setup*/ false, /*clk_gating_hold*/ false);
    bool moreOptNeeded = !ends.empty();
    if (!moreOptNeeded) {
      std::cout << "Silisizer optimization done!" << std::endl;
      break;
    }
    std::unordered_map<sta::Instance*, double> offendingInstCount;

    for (sta::PathEnd* pathend : ends) {
      sta::Path* path = pathend->path();
      sta::Pin* pin = path->pin(this);
      if (debug)
        std::cout << "End Violation at: " << network->name(pin) << std::endl;
      sta::PathRef p;
      path->prevPath(this, p);
      float slack = pathend->slack(this);
      if (slack >= 0.0) continue;
      while (!p.isNull()) {
        pin = p.pin(this);
        sta::PathRef prev_path;
        sta::TimingArc* prev_arc = nullptr;
        p.prevPath(this, prev_path, prev_arc);
        sta::Delay delay = 0.0f;
        if (prev_arc) delay = prev_arc->intrinsicDelay();
        sta::Instance* inst = network->instance(pin);
        if (offendingInstCount.find(inst) == offendingInstCount.end()) {
          offendingInstCount.emplace(inst, delay);
        } else {
          offendingInstCount.find(inst)->second += delay;
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
    std::string to_cell_name =
        std::regex_replace(from_cell_name, std::regex("_sp0_"), "_sp1_");
    // if (debug)
    std::cout << "Resizing instance " << network->name(offender)
              << " of type: " << from_cell_name << " to type: " << to_cell_name << std::endl;
    sta::LibertyCell* to_cell = library->findLibertyCell(to_cell_name.c_str());

    if (!to_cell) {
      std::cout << "WARNING: Missing cell model: " << to_cell_name << std::endl;
      std::cout << "Silisizer optimization done!" << std::endl;
      break;
    }
    Sta::sta()->replaceCell(offender, to_cell);
  }
  return 0;
}

}  // namespace SILISIZER

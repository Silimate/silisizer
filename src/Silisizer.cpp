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

#include <fstream>
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
  uint32_t timing_group_count = 10;
  uint32_t end_point_count = 10;
  uint32_t concurent_replace_count = 3;
  bool debug = 0;
  std::ofstream transforms("preqorsor/data/resized_cells.csv");
  while (1) {
    std::cout << "  Timer is called..." << std::endl;
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
      std::cout << "Timing optimization done!" << std::endl;
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
        sta::Cell* cell = network->cell(inst);
        // If the instance does not have a cell, skip
        if (!cell) {
          p.prevPath(this, p);
          continue;
        }
        sta::LibertyCell* libcell = network->libertyCell(cell);
        // If it's not a liberty cell (module), skip
        if (!libcell) {
          p.prevPath(this, p);
          continue;
        }
        std::string libcellname = libcell->name();
        // If it's not a speed0 cell, skip
        if (libcellname.find("_sp0_") == std::string::npos) {
          p.prevPath(this, p);
          continue;
        }
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
    if (offendingInstCount.empty()) {
      std::cout << "Timing optimization done!" << std::endl;
      break;
    }
    std::vector<std::pair<sta::Instance*, double>> offenders;
    for (auto pair : offendingInstCount) {
      if (offenders.empty()) {
        offenders.push_back(std::pair(pair.first, pair.second));
      } else {
        for (std::vector<std::pair<sta::Instance*, double>>::iterator itr =
                 offenders.begin();
             itr != offenders.end(); itr++) {
          if ((*itr).second < pair.second) {
            offenders.insert(++itr, std::pair(pair.first, pair.second));
            if (offenders.size() > concurent_replace_count) {
              offenders.pop_back();
            }
            break;
          }
        }
      }
    }

    if (offenders.empty()) {
      std::cout << "Timing optimization done!" << std::endl;
      break;
    }

    for (auto offender_pair : offenders) {
      sta::Instance* offender = offender_pair.first;
      sta::Cell* cell = network->cell(offender);
      sta::LibertyLibrary* library = network->libertyLibrary(offender);
      sta::LibertyCell* libcell = network->libertyCell(cell);
      std::string from_cell_name = libcell->name();
      std::string to_cell_name =
          std::regex_replace(from_cell_name, std::regex("_sp0_"), "_sp1_");
      // if (debug)
      std::cout << "  Resizing instance " << network->name(offender)
                << " of type: " << from_cell_name
                << " to type: " << to_cell_name << std::endl;
      sta::LibertyCell* to_cell =
          library->findLibertyCell(to_cell_name.c_str());

      if (!to_cell) {
        std::cout << "WARNING: Missing cell model: " << to_cell_name
                  << std::endl;
        std::cout << "Timing optimization done!" << std::endl;
        transforms.close();
        return 0;
      }
      Sta::sta()->replaceCell(offender, to_cell);
      if (transforms.good())
        transforms << network->name(offender) << "," << from_cell_name << ","
                   << to_cell_name << std::endl;
    }
  }
  transforms.close();
  return 0;
}

}  // namespace SILISIZER

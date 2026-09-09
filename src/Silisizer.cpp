// Silisizer: resize operator-level cells to resolve timing violations
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

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/PathEnd.hh"
#include "sta/PortDirection.hh"
#include "sta/Sta.hh"
#include "sta/TimingRole.hh"

namespace silisizer {

// DEBUG flag
const bool DEBUG = 0;

// Number of consecutive non-improving timing passes allowed by the WNS policy.
const int WNS_STALL_ROUND_LIMIT = 3;

// Replace all occurrences of `from` in `str` with `to`
std::string replaceAll(std::string_view str, std::string_view from,
                       std::string_view to) {
  size_t start_pos = 0;
  std::string result(str);
  while ((start_pos = result.find(from, start_pos)) != std::string::npos) {
    result.replace(start_pos, from.length(), to);
    start_pos += to.length(); // handles case where 'to' is a substr of 'from'
  }
  return result;
}

// Reverse the internal naming convention used by OpenSTA for readback
std::string reverseOpenSTANaming(std::string cellname) {
  cellname = replaceAll(cellname, "\\[", "[");
  cellname = replaceAll(cellname, "\\]", "]");
  cellname = replaceAll(cellname, "\\/", "/");
  cellname = replaceAll(cellname, "\\\\", "\\");
  return cellname;
}

// Silisizer: resize operator-level cells to resolve timing violations
int Silisizer::silisize(const char *workdir,
                        bool upsize_all,
                        bool stop_on_wns_stall) {
  // Initialize network
  sta::Network* network = this->network();

  // Effort variables (multiply swaps per iteration by 2 until complete)
  int swaps_per_iter = 1;

  // Record of (module, cell) pairs that have already been upsized.
  std::set<std::pair<std::string, std::string>> recorded;

  // Populate map of sp0 leaf copies by (module, cell) for fast lookups
  // {
  //   (module1, cell1) -> [leaf instances]
  //   (module1, cell2) -> [leaf instances]
  //   (module2, cell1) -> [leaf instances]
  //   (module2, cell2) -> [leaf instances]
  //   ...
  // }
  std::unordered_map<std::string, std::vector<sta::Instance*>> fold_insts;
  {
    std::unique_ptr<sta::LeafInstanceIterator> leaves(
        network->leafInstanceIterator());
    while (leaves->hasNext()) { // loop over all leaves
      sta::Instance* leaf = leaves->next();
      sta::Instance* parent = network->parent(leaf);
      if (!parent)
        continue;
      sta::LibertyCell* leaf_lib = network->libertyCell(network->cell(leaf));
      if (!leaf_lib ||
          std::string(leaf_lib->name()).find("_sp0_") == std::string::npos)
        continue;
      // Construct key for future lookup
      std::string key = std::string(network->cellName(parent)) + '\t' +
                        reverseOpenSTANaming(network->name(leaf));
      // Add leaf instance to list for the cell
      fold_insts[key].push_back(leaf);
    }
  }

  // Output the header for back-annotation TSV. Preqorsor always reads this
  // file after SPEED==2 STA, so failing to create it must be a hard error.
  std::string workdir_str = workdir;
  std::string data_dir = workdir_str + "/data";
  std::string transforms_path = data_dir + "/resized_cells.tsv";
  std::error_code ec;
  std::filesystem::create_directories(data_dir, ec);
  if (ec) {
    std::cerr << "silisize: cannot create " << data_dir << ": " << ec.message()
              << std::endl;
    return 1;
  }
  std::ofstream transforms(transforms_path);
  if (!transforms.good()) {
    std::cerr << "silisize: cannot open " << transforms_path << " for write"
              << std::endl;
    return 1;
  }
  transforms << "Scope" << "\t" << "Instance" << std::endl;

  // Flush/close the transforms file and surface any write failure (disk full,
  // NFS stale handle, flush error) as a hard error, so Preqorsor never
  // back-annotates a truncated resized_cells.tsv.
  auto close_transforms = [&transforms, &transforms_path]() -> int {
    transforms.close();
    if (transforms.fail()) {
      std::cerr << "silisize: failed writing " << transforms_path << std::endl;
      return 1;
    }
    return 0;
  };

  // Iterate until the maximum number of iterations is reached
  double previous_wns = 1;
  int wns_stall_rounds = 0;
  for (int cur_iter = 0; true; cur_iter++) {
    // Run timer to get violating paths (one per endpoint)
    std::cout << "Running timer..." << std::endl;

    sta::StringSeq group_names;  // empty = report all path groups

    sta::PathEndSeq ends = sta_->findPathEnds(
        /*exception from*/ nullptr, /*exception through*/ nullptr,
        /*exception to*/ nullptr, /*unconstrained*/ false, /*scenes*/ sta_->scenes(),
        /*min_max*/ sta::MinMaxAll::max(),
        /*group_count*/ 10000, /*endpoint_count*/ 1,
        /*unique_pins*/ true,
        /*unique_edges*/ true,
        /*min_slack*/ -1.0e+30, /*max_slack*/ 0.0,
        /*sort_by_slack*/ false,
        /*groups->size() ? groups :*/ group_names,
        /*setup*/ true, /*hold*/ false,
        /*recovery*/ false, /*removal*/ false,
        /*clk_gating_setup*/ false, /*clk_gating_hold*/ false);

    // If no paths are found, we are done
    if (ends.empty()) {
      std::cout << "No paths found..." << std::endl
                << "Final WNS: 0" << std::endl
                << "Timing optimization done!" << std::endl;
      break;
    }

    // DEBUG: Print the number of paths found
    if (DEBUG)
      std::cout << "Violating path count: " << ends.size() << std::endl;

    // Initialize variables
    std::unordered_map<sta::Instance*, double> offending_inst_score;
    double wns = 0.0;

    // For each path with negative slack
    for (sta::PathEnd* pathend : ends) {
      // Get path
      sta::Path* path = pathend->path();

      // DEBUG: Print the endpoint
      if (DEBUG)
        std::cout << "Violation endpoint: " << network->name(path->pin(this))
                  << std::endl;
      
      // Get the path slack
      double slack = pathend->slack(this);
      if (slack >= 0.0) continue;

      // Record the path with the worst negative slack (WNS)
      if (slack < wns) {
        wns = slack;
      }

      // Follow the path backwards to populate offending instance count
      for (sta::Path* p = path; p && !p->isNull(); p = p->prevPath()) {
        // Get the pin
        sta::Pin *pin = p->pin(this);
        // Get previous arc
        sta::TimingArc* prev_arc = p->prevArc(this);
        // Past a transparent latch the path is in a different launch cycle
        if (prev_arc && prev_arc->role()->isLatchDtoQ()) break;
        // Get the arc delay
        sta::Delay delay = 0.0f;
        if (prev_arc) delay = prev_arc->intrinsicDelay();
        // Get the instance and cell
        sta::Instance* inst = network->instance(pin);
        sta::Cell* cell = network->cell(inst);
        // If the instance does not have a cell, skip
        if (!cell) continue;
        // If cell is not a Liberty cell, skip
        sta::LibertyCell* libcell = network->libertyCell(cell);
        if (!libcell) continue;
        // If cell is not speed 0, skip
        std::string libcellname = libcell->name();
        if (libcellname.find("_sp0_") == std::string::npos) {
          if (DEBUG) std::cout << "Speed 1 cell: " << libcellname << std::endl;
          continue;
        }
        // Map instances found in all paths, record cumulative arc delay
        // contribution for each instance accross all paths
        double delta_score = std::min((double) delay, -slack);
        if (offending_inst_score.find(inst) == offending_inst_score.end()) {
          offending_inst_score.emplace(inst, delta_score);
        } else {
          offending_inst_score.find(inst)->second += delta_score;
        }
      }
    }

    // Set previous WNS to current if not initialized (-1)
    if (previous_wns > 0) previous_wns = wns;

    // DEBUG: Print the number of offending instances
    if (DEBUG)
      std::cout << "offending_inst_score: " << offending_inst_score.size()
                << std::endl;

    // Check if there is nothing left to do
    if (offending_inst_score.empty()) {
      // If there are no fixable cells at all and the WNS is zero, we are done
      if (wns == 0.0f) {
        std::cout << "No fixable cells and WNS is 0!" << std::endl
                  << "Final WNS: 0" << std::endl
                  << "Timing optimization done!" << std::endl;
      }
      // If there are no fixable cells at all and the WNS is non-zero, then we
      // have done all we can, but we are still failing timing
      else {
        std::cout << "No fixable cells and WNS is non-zero!" << std::endl
                  << "Final WNS: " << -(wns * 1e12) << std::endl
                  << "Timing optimization partially done!" << std::endl;
      }
      break;
    }

    // The WNS policy intentionally stops even while other violating paths may
    // still benefit from resizing.
    if (stop_on_wns_stall && cur_iter > 0) {
      double delta_wns = wns - previous_wns;
      if (delta_wns <= 0.0)
        wns_stall_rounds++;
      else
        wns_stall_rounds = 0;

      if (wns_stall_rounds >= WNS_STALL_ROUND_LIMIT) {
        std::cout << "WNS did not improve for " << wns_stall_rounds
                  << " consecutive rounds." << std::endl
                  << "Final WNS: " << -(wns * 1e12) << std::endl
                  << "WNS optimization done!" << std::endl;
        break;
      }
    }

    // Sort the offender list and, unless requested otherwise, limit it to the
    // adaptive number of swaps for this iteration.
    std::list<std::pair<sta::Instance*, double>> offenders;
    for (const auto& pair : offending_inst_score)
      offenders.push_back(pair);
    offenders.sort([](const std::pair<sta::Instance*, double>& a,
                      const std::pair<sta::Instance*, double>& b) {
      return a.second > b.second;
    });
    if (!upsize_all)
      offenders.resize(std::min(swaps_per_iter, (int) offenders.size()));

    // DEBUG: Print the number of offenders
    if (DEBUG) std::cout << "offenders: " << offenders.size() << std::endl;

    // If no offending cells, we are done
    if (offenders.empty()) {
      std::cout << "No offenders found..." << std::endl
                << "Final WNS: 0" << std::endl
                << "Timing optimization done!" << std::endl;
      break;
    }

    // For each offending cell, resize to speed 1
    for (auto offender_pair : offenders) {
      // Get the instance, cell, library, and Liberty cell
      sta::Instance* offender = offender_pair.first;
      sta::Cell* cell = network->cell(offender);
      sta::LibertyLibrary* library = network->libertyLibrary(offender);
      sta::LibertyCell* libcell = network->libertyCell(cell);
      // Get speed 0 and 1 cell names
      std::string sp0_name = libcell->name();
      std::string sp1_name = replaceAll(sp0_name, "_sp0_", "_sp1_");
      // Get hierarchical parent module name
      std::string fullname;
      sta::Instance* parent = network->parent(offender);
      std::string parentcellname = network->cellName(parent);
      for (; parent; parent = network->parent(parent)) {
        std::string parentName = network->name(parent);
        if (!parentName.empty()) fullname += parentName + ".";
      }
      std::string cellname = reverseOpenSTANaming(network->name(offender));

      // Insert key into recorded set
      std::pair<std::string, std::string> key{parentcellname, cellname};
      if (recorded.find(key) != recorded.end())
        continue; // skip if already recorded, as we swap all instances already
      // A sibling in this batch may already have closed the fold to sp1.
      if (sp0_name.find("_sp0_") == std::string::npos)
        continue;
      recorded.insert(key);

      // Log resizing operation
      std::cout << "Resizing instance " << fullname + cellname
                << " of type " << sp0_name
                << " to type " << sp1_name << std::endl;

      // Find the corresponding speed 1 Liberty cell
      sta::LibertyCell* to_cell = library->findLibertyCell(sp1_name.c_str());
      if (!to_cell) {
        // Should never happen since we create Liberty cells for both speeds
        std::cout << "WARNING: Missing cell model " << sp1_name << std::endl
                  << "This should never happen!" << std::endl
                  << "Final WNS: " << -(wns * 1e12) << std::endl
                  << "Timing optimization partially done!" << std::endl;
        return close_transforms();
      }

      // Swap every folded copy of this (module, cell) to the speed 1 cell
      auto fold_it = fold_insts.find(parentcellname + '\t' + cellname);
      if (fold_it != fold_insts.end()) {
        for (sta::Instance* leaf : fold_it->second) {
          sta::LibertyCell* leaf_lib = network->libertyCell(network->cell(leaf));
          if (!leaf_lib ||
              std::string(leaf_lib->name()).find("_sp0_") == std::string::npos)
            continue;
          Sta::sta()->replaceCell(leaf, to_cell);
        }
      }

      // Record the transformation for back-annotation in the folded model
      // (unique module name/cell name)
      transforms << reverseOpenSTANaming(parentcellname) << "\t" << cellname
                 << std::endl;
    }

    // Get delta WNS and delta WNS fraction
    double delta_wns = wns - previous_wns;
    double delta_wns_frac = - delta_wns / previous_wns;

    // Print the abs delta WNS in between loops
    if (cur_iter > 0) {
      std::cout << "Delta WNS: " << delta_wns * 1e12 << std::endl;
      std::cout << "Delta WNS frac: " << delta_wns_frac << std::endl;
    }

    // Set effort based on delta WNS when adaptive batching is enabled.
    if (!upsize_all && delta_wns_frac < 0.1 && swaps_per_iter < 1048576)
      swaps_per_iter *= 2;

    // Print the current iteration and WNS
    std::cout << "Iter " << cur_iter + 1 << std::endl;
    std::cout << "Current WNS: " << -(wns * 1e12) << std::endl;

    // DEBUG: Print the current effort and corresponding variables
    if (DEBUG) {
      std::cout << "******************************" << std::endl;
      std::cout << "Current iter: " << cur_iter << std::endl;
      std::cout << "------------------------------" << std::endl;
      std::cout << "Previous WNS: " << -(previous_wns * 1e12) << std::endl;
      std::cout << "Current WNS: " << -(wns * 1e12) << std::endl;
      std::cout << "Delta WNS: " << delta_wns * 1e12 << std::endl;
      std::cout << "Delta WNS frac: " << delta_wns_frac << std::endl;
      std::cout << "------------------------------" << std::endl;
      std::cout << "Swaps per iter: " << swaps_per_iter << std::endl;
      std::cout << "******************************" << std::endl;
    }

    // Store previous WNS for delta calculation
    previous_wns = wns;
  }
  
  // Clean up
  return close_transforms();
}

// Remove escape characters from JSON output
static std::string jsonName(std::string_view s) {
  std::string out;
  out.reserve(s.length());
  for (char c : s) if (c != '\\') out.push_back(c);
  return out;
}

// JSON dumper for clock gating related instances
void dumpIcgJson(const char *path) {
  sta::Sta *sta = sta::Sta::sta();
  sta::Network *network = sta->network();

  std::ofstream f(path);
  if (!f.good()) {
    std::cerr << "dump_icg_json: cannot open " << path << " for write" << std::endl;
    return;
  }

  // Dump gated registers.
  f << "{\n  \"gated_flops\": [";
  bool first = true;
  for (const sta::Instance *reg : sta->clockGatedRegisters()) {
    if (!first) f << ",";
    f << "\n    \"" << jsonName(network->pathName(reg)) << "\"";
    first = false;
  }
  f << (first ? "" : "\n  ") << "],\n  \"icgs\": {";

  // Dump clock-gating instances mapped to their liberty cell names.
  first = true;
  std::unique_ptr<sta::LeafInstanceIterator> it(network->leafInstanceIterator());
  while (it->hasNext()) {
    sta::Instance *inst = it->next();
    sta::Cell *cell = network->cell(inst);
    if (!cell) continue;
    sta::LibertyCell *lc = network->libertyCell(cell);
    if (!lc || !lc->isClockGate()) continue;
    if (!first) f << ",";
    f << "\n    \"" << jsonName(network->pathName(inst))
      << "\": \"" << jsonName(lc->name()) << "\"";
    first = false;
  }
  f << (first ? "" : "\n  ") << "}\n}\n";
}

}  // namespace silisizer

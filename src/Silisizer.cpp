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

#include <cmath>
#include <fstream>
#include <iostream>
#include <list>

#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/PathEnd.hh"
#include "sta/PortDirection.hh"
#include "sta/Sta.hh"

namespace SILISIZER {

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
                        int max_iter,
                        int min_paths_per_group,
                        int max_paths_per_group,
                        int min_swaps_per_iter,
                        int max_swaps_per_iter,
                        double delay_weight_exp,
                        double slack_weight_exp) {
  // DEBUG
  const bool DEBUG = 0;
  
  // Initialize network
  sta::Network* network = this->network();

  // Effort variables (PI control)
  int paths_per_group = min_paths_per_group;
  int swaps_per_iter = min_swaps_per_iter;
  double effort = 0; // next_effort = effort + P * err + I * cum_err
  double target_wns_frac_cum_err = 0; // next_cum_err = cum_err + err 
  const double P = 1; // proportional gain (P multiplier)
  const double I = 1 / max_iter; // integral gain (I multiplier)
  const double TARGET_FINISH_ITER = max_iter * 0.5; // iter to try to finish by

  // Output the header for back-annotation CSV
  std::string workdir_str = workdir;
  std::ofstream transforms(workdir_str + "/data/resized_cells.csv");
  if (transforms.good()) {
    transforms << "Scope" << "," << "Instance" << "," << "From cell" << ","
               << "To cell" << std::endl;
  }

  // Iterate until the maximum number of iterations is reached
  double previous_wns = 1;
  for (int cur_iter = 0; cur_iter < max_iter; cur_iter++) {
    // Run timer to get violating paths (one per endpoint)
    std::cout << "Running timer..." << std::endl;
    sta::PathEndSeq ends = sta_->findPathEnds(
        /*exception from*/ nullptr, /*exception through*/ nullptr,
        /*exception to*/ nullptr, /*unconstrained*/ false, /*corner*/ nullptr,
        /*min_max*/ sta::MinMaxAll::all(),
        /*group_count*/ paths_per_group, /*endpoint_count*/ 1,
        /*unique_pins*/ true,
        /*min_slack*/ -1.0e+30, /*max_slack*/ 0.0,
        /*sort_by_slack*/ false,
        /*groups->size() ? groups :*/ nullptr,
        /*setup*/ true, /*hold*/ false,
        /*recovery*/ false, /*removal*/ false,
        /*clk_gating_setup*/ false, /*clk_gating_hold*/ false);

    // If no paths are found, we are done
    if (ends.empty()) {
      std::cout << "Final WNS: 0" << std::endl;
      std::cout << "Timing optimization done!" << std::endl;
      break;
    }

    // DEBUG: Print the number of paths found
    if (DEBUG)
      std::cout << "Violating path count: " << ends.size() << std::endl;

    // Initialize variables
    std::unordered_map<sta::Instance*, double> offending_inst_score;
    double wns = 0.0f;

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
      for (sta::PathRef p(path); !p.isNull(); p.prevPath(this, p)) {
        // Get the pin
        sta::Pin *pin = p.pin(this);
        // Get previous path and arc
        sta::PathRef prev_path;
        sta::TimingArc* prev_arc = nullptr;
        p.prevPath(this, prev_path, prev_arc);
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
        double delta_score = pow(delay, delay_weight_exp);
        delta_score *= pow(fabs(slack), slack_weight_exp);
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
        std::cout << "Final WNS: 0" << std::endl;
        std::cout << "Timing optimization done!" << std::endl;
      }
      // If there are no fixable cells at all and the WNS is non-zero, then we
      // have done all we can, but we are still failing timing
      else {
        std::cout << "Final WNS: " << -(wns * 1e12) << std::endl;
        std::cout << "Timing optimization partially done!" << std::endl;
      }
      break;
    }

    // Sort top offender list and allow max list size of swaps_per_iter
    std::list<std::pair<sta::Instance*, double>> offenders;
    for (const auto& pair : offending_inst_score)
      offenders.push_back(pair);
    offenders.sort([](const std::pair<sta::Instance*, double>& a,
                      const std::pair<sta::Instance*, double>& b) {
      return a.second > b.second;
    });
    offenders.resize(std::min(swaps_per_iter, (int) offenders.size()));

    // DEBUG: Print the number of offenders
    if (DEBUG) std::cout << "offenders: " << offenders.size() << std::endl;

    // If no offending cells, we are done
    if (offenders.empty()) {
      std::cout << "Final WNS: 0" << std::endl
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
      std::cout << "Resizing instance " << fullname + cellname
                << " of type " << sp0_name
                << " to type " << sp1_name << std::endl;
      // Find the corresponding speed 1 Liberty cell
      sta::LibertyCell* to_cell = library->findLibertyCell(sp1_name.c_str());
      if (!to_cell) {
        // Should never happen since we create Liberty cells for both speeds
        std::cout << "WARNING: Missing cell model " << sp1_name << std::endl;
        std::cout << "This should never happen!" << std::endl;
        std::cout << "Final WNS: " << -(wns * 1e12) << std::endl;
        std::cout << "Timing optimization partially done!" << std::endl;
        transforms.close();
        return 0;
      }
      // Swap the cell with speed 1 cell
      Sta::sta()->replaceCell(offender, to_cell);
      // Record the transformation for back-annotation in the folded model
      // (unique module name/cell name)
      if (transforms.good()) {
        transforms << "\"" << parentcellname << "\"" << "," << cellname << ","
                   << sp0_name << "," << sp1_name << std::endl;
      }
    }

    // Get delta WNS and delta WNS fraction
    double delta_wns = wns - previous_wns;
    double delta_wns_frac = - delta_wns / previous_wns;

    // Print the abs delta WNS in between loops
    if (cur_iter > 0) {
      std::cout << "Delta WNS: " << delta_wns * 1e12 << std::endl;
      std::cout << "Delta WNS frac: " << delta_wns_frac << std::endl;
    }

    // Set effort based on delta WNS
    double iters_remaining = std::max(TARGET_FINISH_ITER - cur_iter, 1.0);
    double target_wns_frac = 1 / iters_remaining;
    double target_wns_frac_err = target_wns_frac - delta_wns_frac;
    double delta_effort = P*target_wns_frac_err + I*target_wns_frac_cum_err;
    target_wns_frac_cum_err += target_wns_frac_err;
    effort = std::max(std::min(effort + delta_effort, 1.0), 0.0);
    paths_per_group = effort * max_paths_per_group;
    paths_per_group += (1 - effort) * min_paths_per_group;
    paths_per_group = std::min(paths_per_group, max_paths_per_group);
    swaps_per_iter = effort * max_swaps_per_iter;
    swaps_per_iter += (1 - effort) * min_swaps_per_iter;
    swaps_per_iter = std::min(swaps_per_iter, max_swaps_per_iter);

    // Print the current iteration and WNS
    std::cout << "Iter " << cur_iter + 1 << " of " << max_iter << std::endl;
    std::cout << "Current WNS: " << -(wns * 1e12) << std::endl;

    // DEBUG: Print the current effort and corresponding variables
    if (DEBUG) {
      std::cout << "******************************" << std::endl;
      std::cout << "Current iter: " << cur_iter << std::endl;
      std::cout << "Target finish iter: " << TARGET_FINISH_ITER << std::endl;
      std::cout << "Iters remaining: " << iters_remaining << std::endl;
      std::cout << "------------------------------" << std::endl;
      std::cout << "Previous WNS: " << -(previous_wns * 1e12) << std::endl;
      std::cout << "Current WNS: " << -(wns * 1e12) << std::endl;
      std::cout << "Delta WNS: " << delta_wns * 1e12 << std::endl;
      std::cout << "Delta WNS frac: " << delta_wns_frac << std::endl;
      std::cout << "Target WNS frac: " << target_wns_frac << std::endl;
      std::cout << "Target WNS frac err: " << target_wns_frac_err << std::endl;
      std::cout << "------------------------------" << std::endl;
      std::cout << "Current effort: " << effort << std::endl;
      std::cout << "Paths per group: " << paths_per_group << std::endl;
      std::cout << "Swaps per iter: " << swaps_per_iter << std::endl;
      std::cout << "******************************" << std::endl;
    }

    // If we are here at the last iteration, we were unable to meet timing
    if (cur_iter == (max_iter - 1)) {
      std::cout << "WARNING: Cannot meet timing constraints!" << std::endl;
      std::cout << "Final WNS: " << -(wns * 1e12) << std::endl;
      std::cout << "Timing optimization partially done!" << std::endl;
      transforms.close();
      return 0;
    }

    // Store previous WNS for delta calculation
    previous_wns = wns;
  }
  
  // Clean up
  transforms.close();
  return 0;
}

}  // namespace SILISIZER

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

#include "sta/Sta.hh"

namespace SILISIZER {

class Silisizer : public sta::Sta {
 public:
  ~Silisizer() {}
  int silisize(int max_timer_iterations = 200, int nb_concurent_paths = 10,
               int nb_initial_concurent_changes = 3,
               int nb_high_effort_concurent_changes = 20);
};
}  // namespace SILISIZER

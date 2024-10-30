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
class dbStaState;
}

namespace sta {
class dbNetwork;
class dbStaReport;
class dbStaCbk;

class dbSta : public sta::Sta {
 public:
  ~dbSta() override;
  void registerStaState(SILISIZER::dbStaState* state);
  void unregisterStaState(SILISIZER::dbStaState* state);

 private:
  std::set<SILISIZER::dbStaState*> sta_states_;
};
}  // namespace sta

namespace SILISIZER {

class dbStaState : public sta::StaState {
 public:
  void init(sta::dbSta* sta);
  ~dbStaState() override;

 protected:
  sta::dbSta* sta_ = nullptr;
};

class Silisizer : public sta::Sta {
 public:
  ~Silisizer() {}
  int silisize();
};
}  // namespace SILISIZER

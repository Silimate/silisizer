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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

%module silisizer

%inline %{

// Silisizer: resize operator-level cells to resolve timing violations
extern int silisize(
                    int max_iter = 200,
                    int paths_per_group = 100,
                    int paths_per_endpoint = 1,
                    int min_swaps_per_iter = 3,
                    int max_swaps_per_iter = 20,
                    double arc_weight_exponent = 1.0 // nb_occur * (arc_delay**arc_weight_exponent)
                    );

%}

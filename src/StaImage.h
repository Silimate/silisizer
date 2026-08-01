// Silisizer: session image save/restore
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

#pragma once

#include <cstddef>
#include <string>

// A session image is a verbatim snapshot of the STA engine's heap plus the
// executable's mutable globals. It only reloads in a process where the
// executable, its dylibs and the heap region land at the same addresses as the
// saving process, so the image is tied to one binary on one OS. Layout is
// pinned by disabling ASLR (see imageEnsureFixedLayout) and by mapping the heap
// region MAP_FIXED.
namespace silisizer {

// Image mode routes global operator new into the snapshottable region. It must
// be decided before the first allocation, so it comes from the environment
// (SILISIZER_STA_IMAGE=1) rather than from parsed arguments.
bool imageModeEnabled();

// Reserves the region and starts routing global operator new into it. Call
// early in main(), before any engine object is created.
void imageArm();
bool imageArmed();

void *imageAlloc(size_t size);
void *imageAllocAligned(size_t size, size_t alignment);
void imageFree(void *ptr);
bool imageOwns(const void *ptr);

// Bytes handed out so far; also the amount of the region that gets written to
// an image file.
size_t imageBytesInUse();

// Re-execs the process with ASLR disabled when image mode needs a fixed address
// layout and the current process does not have one. Does not return in that
// case. Call as the first statement in main().
void imageEnsureFixedLayout(int argc, char *argv[]);

// Writes the region and the executable's mutable globals to path.
bool imageSave(const char *path, std::string &error);

// Maps an image back in. Must run before initSta()/makeComponents(), since it
// installs the saved globals (including Sta::sta_) wholesale.
bool imageRestore(const char *path, std::string &error);

// True between a successful imageRestore() and the end of startup, so main()
// can skip the normal engine construction path.
bool imageRestored();

}  // namespace silisizer

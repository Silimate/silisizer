// Silisizer: session image save/restore
// Copyright (c) 2026, Silimate Inc.
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

#include "StaImage.h"

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#include <mach-o/getsect.h>
#include <spawn.h>
#include <sys/wait.h>
extern char **environ;
// Not in the public headers, but honored by posix_spawn on macOS.
#ifndef _POSIX_SPAWN_DISABLE_ASLR
#define _POSIX_SPAWN_DISABLE_ASLR 0x0100
#endif
#else
#include <link.h>
#include <sys/personality.h>
#endif

#ifndef MAP_NORESERVE
#define MAP_NORESERVE 0
#endif

namespace silisizer {

namespace {

// The region sits well above anything the loader places, so MAP_FIXED there is
// reliable once ASLR is off.
constexpr uintptr_t kRegionBase = 0x200000000000ULL;
constexpr size_t kDefaultRegionSize = 128ULL * 1024 * 1024 * 1024;  // 128 GiB
constexpr size_t kSpanSize = 64 * 1024;
constexpr size_t kAlign = 16;
constexpr size_t kSmallMax = 2048;
constexpr size_t kClassCount = kSmallMax / kAlign + 1;
constexpr size_t kFileAlign = 64 * 1024;
constexpr uint32_t kImageVersion = 1;
constexpr size_t kMaxSegments = 16;
constexpr uint64_t kRegionMagic = 0x53494C495352474EULL;  // "SILISRGN"

// Span table tags. A span is either carved into one small size class, part of a
// large allocation, or on the free list.
constexpr uint64_t kTagShift = 56;
constexpr uint64_t kTagSmall = 1ULL << kTagShift;
constexpr uint64_t kTagLargeHead = 2ULL << kTagShift;
constexpr uint64_t kTagLargeTail = 3ULL << kTagShift;
constexpr uint64_t kTagFreeHead = 4ULL << kTagShift;
constexpr uint64_t kTagFreeTail = 5ULL << kTagShift;
constexpr uint64_t kTagMask = 0xFFULL << kTagShift;
constexpr uint64_t kValueMask = ~kTagMask;

struct FreeBlock {
  FreeBlock *prev;
  FreeBlock *next;
};

// Allocator state lives inside the region so the snapshot captures it along
// with everything it describes.
struct RegionHeader {
  uint64_t magic;
  uint64_t region_size;
  uint64_t bump;  // first never-allocated address
  uint64_t span_table;
  uint64_t span_count;
  void *small_free[kClassCount];
  FreeBlock *large_free;
};

std::atomic<int> g_armed{0};
std::atomic<int> g_region_ready{0};
std::atomic<int> g_restored{0};
std::atomic_flag g_lock = ATOMIC_FLAG_INIT;

class Guard {
 public:
  Guard() {
    while (g_lock.test_and_set(std::memory_order_acquire)) {
    }
  }
  ~Guard() { g_lock.clear(std::memory_order_release); }
};

RegionHeader *header() { return reinterpret_cast<RegionHeader *>(kRegionBase); }

uint64_t *spanTable() {
  return reinterpret_cast<uint64_t *>(header()->span_table);
}

size_t spanIndex(const void *ptr) {
  return (reinterpret_cast<uintptr_t>(ptr) - kRegionBase) / kSpanSize;
}

void *spanAddr(size_t index) {
  return reinterpret_cast<void *>(kRegionBase + index * kSpanSize);
}

size_t regionSizeFromEnv() {
  const char *env = getenv("SILISIZER_STA_IMAGE_REGION_GB");
  if (env) {
    long gb = strtol(env, nullptr, 10);
    if (gb > 0) return static_cast<size_t>(gb) * 1024 * 1024 * 1024;
  }
  return kDefaultRegionSize;
}

// Reserves the region and lays the allocator header and span table at its base.
// Pages fault in only as they are touched.
bool createRegion(size_t region_size) {
  void *mapped = mmap(reinterpret_cast<void *>(kRegionBase), region_size,
                      PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANON | MAP_FIXED | MAP_NORESERVE, -1, 0);
  if (mapped == MAP_FAILED || mapped != reinterpret_cast<void *>(kRegionBase))
    return false;

  RegionHeader *hdr = header();
  hdr->magic = kRegionMagic;
  hdr->region_size = region_size;
  hdr->span_count = region_size / kSpanSize;
  hdr->span_table = kRegionBase + sizeof(RegionHeader);
  hdr->large_free = nullptr;
  for (size_t i = 0; i < kClassCount; i++) hdr->small_free[i] = nullptr;

  uintptr_t after_table = hdr->span_table + hdr->span_count * sizeof(uint64_t);
  hdr->bump = (after_table + kSpanSize - 1) & ~(uintptr_t)(kSpanSize - 1);
  memset(spanTable(), 0, hdr->span_count * sizeof(uint64_t));
  return true;
}

void listRemove(FreeBlock *block) {
  RegionHeader *hdr = header();
  if (block->prev)
    block->prev->next = block->next;
  else
    hdr->large_free = block->next;
  if (block->next) block->next->prev = block->prev;
}

void listPush(FreeBlock *block) {
  RegionHeader *hdr = header();
  block->prev = nullptr;
  block->next = hdr->large_free;
  if (hdr->large_free) hdr->large_free->prev = block;
  hdr->large_free = block;
}

// Head is written last so a single span block reads back as a head.
void markFree(size_t index, size_t spans) {
  uint64_t *table = spanTable();
  if (spans > 1) table[index + spans - 1] = kTagFreeTail | spans;
  table[index] = kTagFreeHead | spans;
  listPush(static_cast<FreeBlock *>(spanAddr(index)));
}

void markLarge(size_t index, size_t spans) {
  uint64_t *table = spanTable();
  if (spans > 1) table[index + spans - 1] = kTagLargeTail | spans;
  table[index] = kTagLargeHead | spans;
}

void *bumpSpans(size_t spans) {
  RegionHeader *hdr = header();
  uintptr_t addr = hdr->bump;
  size_t bytes = spans * kSpanSize;
  if (addr + bytes > kRegionBase + hdr->region_size) return nullptr;
  hdr->bump = addr + bytes;
  return reinterpret_cast<void *>(addr);
}

void *allocLarge(size_t size) {
  RegionHeader *hdr = header();
  size_t spans = (size + kSpanSize - 1) / kSpanSize;
  for (FreeBlock *block = hdr->large_free; block; block = block->next) {
    size_t index = spanIndex(block);
    size_t have = spanTable()[index] & kValueMask;
    if (have < spans) continue;
    listRemove(block);
    if (have > spans) markFree(index + spans, have - spans);
    markLarge(index, spans);
    return block;
  }
  void *fresh = bumpSpans(spans);
  if (!fresh) return nullptr;
  markLarge(spanIndex(fresh), spans);
  return fresh;
}

// Merges with the neighbouring free blocks so repeated vector growth does not
// fragment the region.
void freeLarge(size_t index, size_t spans) {
  RegionHeader *hdr = header();
  uint64_t *table = spanTable();
  size_t bump_span = spanIndex(reinterpret_cast<void *>(hdr->bump));

  size_t next = index + spans;
  if (next < bump_span && (table[next] & kTagMask) == kTagFreeHead) {
    size_t more = table[next] & kValueMask;
    listRemove(static_cast<FreeBlock *>(spanAddr(next)));
    spans += more;
  }
  if (index > 0) {
    uint64_t prev = table[index - 1];
    uint64_t prev_tag = prev & kTagMask;
    size_t more = 0;
    if (prev_tag == kTagFreeTail)
      more = prev & kValueMask;
    else if (prev_tag == kTagFreeHead && (prev & kValueMask) == 1)
      more = 1;
    if (more) {
      size_t head = index - more;
      listRemove(static_cast<FreeBlock *>(spanAddr(head)));
      index = head;
      spans += more;
    }
  }
  markFree(index, spans);
}

void *allocSmall(size_t klass) {
  RegionHeader *hdr = header();
  void *head = hdr->small_free[klass];
  if (head) {
    hdr->small_free[klass] = *reinterpret_cast<void **>(head);
    return head;
  }
  void *span = bumpSpans(1);
  if (!span) return nullptr;
  spanTable()[spanIndex(span)] = kTagSmall | klass;

  size_t obj_size = klass * kAlign;
  size_t count = kSpanSize / obj_size;
  char *base = static_cast<char *>(span);
  // Hand out the first object and thread the rest onto the free list.
  for (size_t i = 1; i < count; i++) {
    void *obj = base + i * obj_size;
    *reinterpret_cast<void **>(obj) = hdr->small_free[klass];
    hdr->small_free[klass] = obj;
  }
  return base;
}

bool ensureRegion() {
  if (g_region_ready.load(std::memory_order_acquire)) return true;
  Guard guard;
  if (g_region_ready.load(std::memory_order_relaxed)) return true;
  if (!createRegion(regionSizeFromEnv())) return false;
  g_region_ready.store(1, std::memory_order_release);
  return true;
}

}  // namespace

bool imageModeEnabled() {
  const char *env = getenv("SILISIZER_STA_IMAGE");
  return env && env[0] == '1';
}

void imageArm() {
  if (!imageModeEnabled()) return;
  if (!ensureRegion()) {
    fprintf(stderr, "Error: could not reserve the session image region at %p.\n",
            reinterpret_cast<void *>(kRegionBase));
    exit(EXIT_FAILURE);
  }
  g_armed.store(1, std::memory_order_release);
}

bool imageArmed() { return g_armed.load(std::memory_order_acquire) != 0; }

bool imageOwns(const void *ptr) {
  uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
  if (addr < kRegionBase) return false;
  if (!g_region_ready.load(std::memory_order_acquire)) return false;
  return addr < kRegionBase + header()->region_size;
}

size_t imageBytesInUse() {
  if (!g_region_ready.load(std::memory_order_acquire)) return 0;
  return header()->bump - kRegionBase;
}

void *imageAlloc(size_t size) {
  if (size == 0) size = 1;
  Guard guard;
  if (size <= kSmallMax) return allocSmall((size + kAlign - 1) / kAlign);
  return allocLarge(size);
}

void *imageAllocAligned(size_t size, size_t alignment) {
  // Large blocks start on a span boundary, so routing over-aligned requests
  // there satisfies any alignment up to a span.
  if (alignment <= kAlign) return imageAlloc(size);
  if (alignment > kSpanSize) return nullptr;
  if (size <= kSmallMax) size = kSmallMax + 1;
  Guard guard;
  return allocLarge(size);
}

void imageFree(void *ptr) {
  if (!ptr) return;
  Guard guard;
  uint64_t entry = spanTable()[spanIndex(ptr)];
  uint64_t tag = entry & kTagMask;
  if (tag == kTagSmall) {
    size_t klass = entry & kValueMask;
    RegionHeader *hdr = header();
    *reinterpret_cast<void **>(ptr) = hdr->small_free[klass];
    hdr->small_free[klass] = ptr;
  } else if (tag == kTagLargeHead) {
    freeLarge(spanIndex(ptr), entry & kValueMask);
  }
  // Anything else is a double free or an interior pointer; dropping it beats
  // corrupting the span table.
}

////////////////////////////////////////////////////////////////
// Fixed address layout
////////////////////////////////////////////////////////////////

namespace {

uintptr_t executableBase() {
#if defined(__APPLE__)
  return reinterpret_cast<uintptr_t>(_dyld_get_image_header(0));
#else
  uintptr_t base = 0;
  dl_iterate_phdr(
      [](struct dl_phdr_info *info, size_t, void *data) {
        *static_cast<uintptr_t *>(data) = info->dlpi_addr;
        return 1;
      },
      &base);
  return base;
#endif
}

bool executablePath(char *buffer, size_t size) {
#if defined(__APPLE__)
  uint32_t len = static_cast<uint32_t>(size);
  return _NSGetExecutablePath(buffer, &len) == 0;
#else
  ssize_t len = readlink("/proc/self/exe", buffer, size - 1);
  if (len <= 0) return false;
  buffer[len] = '\0';
  return true;
#endif
}

// Images hold raw vtable pointers, so they are only valid for the exact binary
// that wrote them.
void buildId(char *out, size_t size) {
  char path[4096] = {0};
  struct stat st;
  if (!executablePath(path, sizeof(path)) || stat(path, &st) != 0) {
    snprintf(out, size, "unknown");
    return;
  }
  snprintf(out, size, "%s:%lld:%lld", path, (long long)st.st_size,
           (long long)st.st_mtime);
}

struct SegmentDesc {
  uint64_t addr;
  uint64_t size;
};

// The executable's mutable globals: STA singletons, registries and Sta::sta_
// all live here and must come back exactly as they were.
size_t collectGlobalSegments(SegmentDesc *segs, size_t max_segs) {
#if defined(__APPLE__)
  size_t count = 0;
  const char *sections[] = {"__data", "__bss", "__common"};
  const struct mach_header_64 *exe =
      reinterpret_cast<const struct mach_header_64 *>(_dyld_get_image_header(0));
  for (const char *section : sections) {
    unsigned long size = 0;
    uint8_t *addr = getsectiondata(exe, "__DATA", section, &size);
    if (addr && size && count < max_segs)
      segs[count++] = {reinterpret_cast<uint64_t>(addr), size};
  }
  return count;
#else
  struct Collect {
    SegmentDesc *segs;
    size_t max_segs;
    size_t count;
  } collect{segs, max_segs, 0};
  dl_iterate_phdr(
      [](struct dl_phdr_info *info, size_t, void *data) {
        Collect *c = static_cast<Collect *>(data);
        for (int i = 0; i < info->dlpi_phnum; i++) {
          const ElfW(Phdr) *phdr = &info->dlpi_phdr[i];
          if (phdr->p_type != PT_LOAD || !(phdr->p_flags & PF_W)) continue;
          if (c->count >= c->max_segs) break;
          c->segs[c->count++] = {info->dlpi_addr + phdr->p_vaddr, phdr->p_memsz};
        }
        return 1;  // main executable only
      },
      &collect);
  return collect.count;
#endif
}

struct ImageHeader {
  char magic[8];
  uint32_t version;
  uint32_t segment_count;
  uint64_t exe_base;
  uint64_t region_base;
  uint64_t region_size;
  uint64_t region_bytes;
  uint64_t region_offset;
  uint64_t globals_offset;
  char build_id[512];
  SegmentDesc segments[kMaxSegments];
};

bool writeAll(int fd, const void *data, size_t size) {
  const char *at = static_cast<const char *>(data);
  while (size > 0) {
    size_t chunk = size > (1u << 30) ? (1u << 30) : size;
    ssize_t wrote = write(fd, at, chunk);
    if (wrote <= 0) return false;
    at += wrote;
    size -= wrote;
  }
  return true;
}

}  // namespace

void imageEnsureFixedLayout(int argc, char *argv[]) {
  (void)argc;
  if (!imageModeEnabled()) return;
  // Either the re-exec already happened or the caller pinned the layout.
  if (getenv("SILISIZER_STA_IMAGE_FIXED")) return;
  setenv("SILISIZER_STA_IMAGE_FIXED", "1", 1);

  char self[4096] = {0};
  if (!executablePath(self, sizeof(self))) return;

#if defined(__APPLE__)
  // macOS has no exec flag for this, so respawn and mirror the child's exit.
  posix_spawnattr_t attr;
  posix_spawnattr_init(&attr);
  posix_spawnattr_setflags(&attr, _POSIX_SPAWN_DISABLE_ASLR);
  pid_t pid = 0;
  int rc = posix_spawn(&pid, self, nullptr, &attr, argv, environ);
  posix_spawnattr_destroy(&attr);
  if (rc != 0) {
    fprintf(stderr, "Error: could not respawn with a fixed address layout: %s\n",
            strerror(rc));
    exit(EXIT_FAILURE);
  }
  int status = 0;
  waitpid(pid, &status, 0);
  exit(WIFEXITED(status) ? WEXITSTATUS(status) : EXIT_FAILURE);
#else
  if (personality(ADDR_NO_RANDOMIZE) == -1) {
    fprintf(stderr, "Error: could not disable ASLR: %s\n", strerror(errno));
    exit(EXIT_FAILURE);
  }
  execv(self, argv);
  fprintf(stderr, "Error: could not re-exec %s: %s\n", self, strerror(errno));
  exit(EXIT_FAILURE);
#endif
}

bool imageSave(const char *path, std::string &error) {
  if (!imageArmed()) {
    error =
        "session images require image mode; set SILISIZER_STA_IMAGE=1 before "
        "starting silisizer";
    return false;
  }

  SegmentDesc segs[kMaxSegments];
  size_t seg_count = collectGlobalSegments(segs, kMaxSegments);
  if (seg_count == 0) {
    error = "could not locate the executable's global data segments";
    return false;
  }

  ImageHeader hdr;
  memset(&hdr, 0, sizeof(hdr));
  memcpy(hdr.magic, "SILISTA1", 8);
  hdr.version = kImageVersion;
  hdr.segment_count = static_cast<uint32_t>(seg_count);
  hdr.exe_base = executableBase();
  hdr.region_base = kRegionBase;
  hdr.region_size = header()->region_size;
  hdr.region_bytes = header()->bump - kRegionBase;
  hdr.globals_offset = sizeof(ImageHeader);
  buildId(hdr.build_id, sizeof(hdr.build_id));
  for (size_t i = 0; i < seg_count; i++) hdr.segments[i] = segs[i];

  uint64_t globals_bytes = 0;
  for (size_t i = 0; i < seg_count; i++) globals_bytes += segs[i].size;
  hdr.region_offset = (hdr.globals_offset + globals_bytes + kFileAlign - 1) &
                      ~(uint64_t)(kFileAlign - 1);

  int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    error = std::string("could not open ") + path + ": " + strerror(errno);
    return false;
  }

  bool ok = writeAll(fd, &hdr, sizeof(hdr));
  for (size_t i = 0; ok && i < seg_count; i++)
    ok = writeAll(fd, reinterpret_cast<const void *>(segs[i].addr), segs[i].size);
  if (ok) ok = lseek(fd, hdr.region_offset, SEEK_SET) != (off_t)-1;
  if (ok)
    ok = writeAll(fd, reinterpret_cast<const void *>(kRegionBase),
                  hdr.region_bytes);
  if (!ok) error = std::string("could not write ") + path + ": " + strerror(errno);
  if (close(fd) != 0 && ok) {
    error = std::string("could not close ") + path + ": " + strerror(errno);
    ok = false;
  }
  return ok;
}

bool imageRestore(const char *path, std::string &error) {
  if (!imageArmed()) {
    error =
        "session images require image mode; set SILISIZER_STA_IMAGE=1 before "
        "starting silisizer";
    return false;
  }

  int fd = open(path, O_RDONLY);
  if (fd < 0) {
    error = std::string("could not open ") + path + ": " + strerror(errno);
    return false;
  }

  ImageHeader hdr;
  if (read(fd, &hdr, sizeof(hdr)) != (ssize_t)sizeof(hdr)) {
    error = std::string("could not read an image header from ") + path;
    close(fd);
    return false;
  }
  if (memcmp(hdr.magic, "SILISTA1", 8) != 0 || hdr.version != kImageVersion) {
    error = std::string(path) + " is not a silisizer session image";
    close(fd);
    return false;
  }

  char current_build[512];
  buildId(current_build, sizeof(current_build));
  if (strcmp(current_build, hdr.build_id) != 0) {
    error = std::string(path) +
            " was written by a different silisizer build; a session image is "
            "only valid for the binary that created it";
    close(fd);
    return false;
  }
  if (hdr.exe_base != executableBase()) {
    error =
        "the executable did not load at the address recorded in the image; "
        "ASLR must be disabled to restore";
    close(fd);
    return false;
  }
  if (hdr.region_base != kRegionBase) {
    error = "the image heap base does not match this build";
    close(fd);
    return false;
  }

  // Map the saved heap straight off disk. Pages fault in only as the session
  // touches them, which is what makes restore fast.
  void *mapped = mmap(reinterpret_cast<void *>(kRegionBase), hdr.region_bytes,
                      PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_FIXED, fd,
                      hdr.region_offset);
  if (mapped != reinterpret_cast<void *>(kRegionBase)) {
    error = std::string("could not map the image heap: ") + strerror(errno);
    close(fd);
    return false;
  }
  // Fresh anonymous space for whatever the restored session allocates next.
  if (hdr.region_bytes < hdr.region_size) {
    void *tail = reinterpret_cast<void *>(kRegionBase + hdr.region_bytes);
    if (mmap(tail, hdr.region_size - hdr.region_bytes, PROT_READ | PROT_WRITE,
             MAP_PRIVATE | MAP_ANON | MAP_FIXED | MAP_NORESERVE, -1,
             0) != tail) {
      error = std::string("could not extend the image heap: ") + strerror(errno);
      close(fd);
      return false;
    }
  }

  if (header()->magic != kRegionMagic) {
    error = "the image heap does not contain a valid allocator header";
    close(fd);
    return false;
  }

  SegmentDesc segs[kMaxSegments];
  size_t seg_count = collectGlobalSegments(segs, kMaxSegments);
  if (seg_count != hdr.segment_count) {
    error = "the image's global segment layout does not match this process";
    close(fd);
    return false;
  }
  off_t at = hdr.globals_offset;
  for (size_t i = 0; i < seg_count; i++) {
    if (segs[i].addr != hdr.segments[i].addr ||
        segs[i].size != hdr.segments[i].size) {
      error = "the image's global segment layout does not match this process";
      close(fd);
      return false;
    }
    ssize_t got =
        pread(fd, reinterpret_cast<void *>(segs[i].addr), segs[i].size, at);
    if (got != (ssize_t)segs[i].size) {
      error = std::string("could not read the image globals: ") + strerror(errno);
      close(fd);
      return false;
    }
    at += segs[i].size;
  }

  // The copy above overwrote this module's own statics with the saved values,
  // so re-assert the ones that describe the current process.
  g_armed.store(1, std::memory_order_release);
  g_region_ready.store(1, std::memory_order_release);
  g_restored.store(1, std::memory_order_release);
  close(fd);
  return true;
}

bool imageRestored() { return g_restored.load(std::memory_order_acquire) != 0; }

}  // namespace silisizer

////////////////////////////////////////////////////////////////
// Global operator new/delete
////////////////////////////////////////////////////////////////

// Routing all C++ allocation through the region is what makes a snapshot self
// contained. Before image mode is armed, and when it is off entirely, these
// fall through to malloc; delete tells the two apart by address.
namespace {

void *allocate(size_t size) {
  if (silisizer::imageArmed()) {
    void *ptr = silisizer::imageAlloc(size);
    if (!ptr) throw std::bad_alloc();
    return ptr;
  }
  void *ptr = malloc(size ? size : 1);
  if (!ptr) throw std::bad_alloc();
  return ptr;
}

void *allocateNoThrow(size_t size) noexcept {
  if (silisizer::imageArmed()) return silisizer::imageAlloc(size);
  return malloc(size ? size : 1);
}

void *allocateAligned(size_t size, size_t alignment) {
  if (silisizer::imageArmed()) {
    void *ptr = silisizer::imageAllocAligned(size, alignment);
    if (!ptr) throw std::bad_alloc();
    return ptr;
  }
  void *ptr = nullptr;
  if (posix_memalign(&ptr, alignment < sizeof(void *) ? sizeof(void *) : alignment,
                     size ? size : 1) != 0)
    throw std::bad_alloc();
  return ptr;
}

void release(void *ptr) noexcept {
  if (!ptr) return;
  if (silisizer::imageOwns(ptr))
    silisizer::imageFree(ptr);
  else
    free(ptr);
}

}  // namespace

void *operator new(size_t size) { return allocate(size); }
void *operator new[](size_t size) { return allocate(size); }
void *operator new(size_t size, const std::nothrow_t &) noexcept {
  return allocateNoThrow(size);
}
void *operator new[](size_t size, const std::nothrow_t &) noexcept {
  return allocateNoThrow(size);
}
void *operator new(size_t size, std::align_val_t align) {
  return allocateAligned(size, static_cast<size_t>(align));
}
void *operator new[](size_t size, std::align_val_t align) {
  return allocateAligned(size, static_cast<size_t>(align));
}

void operator delete(void *ptr) noexcept { release(ptr); }
void operator delete[](void *ptr) noexcept { release(ptr); }
void operator delete(void *ptr, size_t) noexcept { release(ptr); }
void operator delete[](void *ptr, size_t) noexcept { release(ptr); }
void operator delete(void *ptr, const std::nothrow_t &) noexcept { release(ptr); }
void operator delete[](void *ptr, const std::nothrow_t &) noexcept {
  release(ptr);
}
void operator delete(void *ptr, std::align_val_t) noexcept { release(ptr); }
void operator delete[](void *ptr, std::align_val_t) noexcept { release(ptr); }
void operator delete(void *ptr, size_t, std::align_val_t) noexcept {
  release(ptr);
}
void operator delete[](void *ptr, size_t, std::align_val_t) noexcept {
  release(ptr);
}

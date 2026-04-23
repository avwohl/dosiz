//
// bridge.cc — the single translation unit in dosemu that touches dosbox
// internals.
//
// Strict cpmemu-style DOS replacement: dosbox-staging provides the CPU +
// PC hardware + BIOS; dosemu provides DOS.  dosbox's own DOS kernel still
// gets initialised (it's in libdos.a and DOSBOX_Init() wires its section),
// but we override INT 21h at the IVT so dosbox's handler never runs.
//
// Init order (inside run_com):
//   1. DOSBOX_Init()  registers all sections, wires SHELL_Init as startup
//   2. control->Init()  activates modules; dosbox's DOS installs its own
//      INT 21h callback at vector 0x21
//   3. control->SetStartUp(&dosemu_startup)  overrides the shell
//   4. control->StartUp()  calls dosemu_startup, which:
//        a. installs our INT 21h handler, overwriting vector 0x21
//        b. builds a minimal PSP with the command tail at PSP:80h
//        c. loads the .COM file at PSP:0100h
//        d. sets CS:IP / SS:SP / DS / ES
//        e. DOSBOX_RunMachine()  CPU runs until AH=4Ch stops it
//

#include "bridge.h"
#include "debug_settings.hpp"

#define SDL_MAIN_HANDLED
#include <SDL.h>

#include "dosbox.h"
#include "control.h"
#include "cross.h"
#include "callback.h"
#include "cpu.h"
#include "mem.h"
#include "regs.h"
#include "programs.h"
#include "loguru.hpp"

// Four helpers in dosbox-staging's sdlmain.cpp that were file-local.  We
// patched them to external linkage so dosemu can drive the pre-StartUp
// init path without duplicating 300+ lines of [sdl] section setup.
class Section_prop;
void config_add_sdl();
void messages_add_sdl();
void messages_add_command_line();
Section_prop *get_sdl_section();
void DOS_Locale_AddMessages();
void RENDER_AddMessages();

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <ctime>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace dosemu::bridge {

const char *dosbox_version() {
  return DOSBOX_GetVersion();
}

namespace {

constexpr uint16_t PSP_SEG          = 0x0100;
constexpr uint16_t COM_ENTRY_OFFSET = 0x0100;
constexpr uint32_t MAX_COM_SIZE     = 0xFF00;

// (PSP is 256 bytes = 16 paragraphs, so .EXE images sit at psp_seg+0x10.)

// Environment block sits in its own segment below the PSP.  Holds strings
// "VAR=value\0" separated by NUL, terminated by an extra NUL, then a
// little-endian uint16 count followed by argv[0] as ASCIIZ.
constexpr uint16_t ENV_SEG          = 0x0050;  // physical 0x500, start of DOS data area
constexpr uint32_t ENV_BYTES        = 0x0800;  // reserve 2KB -- fits average env

// Initial register snapshot a loader returns to dosemu_startup.
//
// For .COM/.EXE the CS/SS are RM segments.  For LE the loader has
// already installed LDT descriptors + prepared PM state; startup must
// flip CR0.PE and treat cs/ss as LDT selectors and eip/esp as 32-bit.
struct InitialRegs {
  uint16_t cs, ip, ss, sp, ax;
  // LE-only:
  bool     is_pm    = false;
  bool     is_32bit = false;       // D-bit of entry CS descriptor
  uint16_t pm_ds    = 0;           // auto-data selector
  uint32_t pm_eip   = 0;
  uint32_t pm_esp   = 0;
};
// Forward declaration: defined near end of file; needed by AH=4Bh
// handler inside dosemu_int21.
bool load_program_at(const std::string &path, uint16_t psp_seg,
                     InitialRegs &out);

// --- Run-time state, reset by run_com() ------------------------------------

std::string                 s_program;

// Set during load_exe_at when the MZ stub contains a DOS-extender
// signature ("DOS/4G" or similar).  Suppresses our own DPMI
// advertisement and our LE-direct-dispatch path, letting the bound
// extender do its own init.  This is what QEMM/Windows/CWSDPMI-era
// DOS extender host systems did: they advertised DPMI, and bound
// extenders either used it or brought their own -- the two worlds
// coexisted without per-binary config.  We choose the same path by
// sensing which kind of binary we're running.
bool s_extender_bound = false;

// Current DPMI client's CPL (0 = legacy ring-0 clients, 3 = ring-3
// clients entered via DOSEMU_DPMI_RING3 path).  OR'd into the RPL
// field of selectors we hand out via INT 31h AX=0000 so the client
// can use them at its own privilege level.  CWSDPMI's `run_ring`.
uint8_t s_client_cpl = 0;
std::vector<std::string>    s_args;
int                         s_exit_code = 0;

// DOS memory control block (MCB) chain for AH=48/49/4Ah.
//
// Each MCB occupies one 16-byte paragraph and looks like:
//
//   +0  uint8   'M' (0x4D) or 'Z' (0x5A, the arena's last block)
//   +1  uint16  owner PSP segment, 0 = free
//   +3  uint16  size in paragraphs, not counting the MCB itself
//   +5  ...     reserved + 8-byte name (unused here)
//
// The block's usable data starts at MCB_SEG + 1 and runs for `size`
// paragraphs.  Allocation walks the chain, finds a free block big enough,
// splits it; free marks a block free and forward-coalesces with the next.
constexpr uint16_t MCB_ARENA_START = 0x2000;   // first MCB's segment
constexpr uint16_t MCB_ARENA_END   = 0xA000;   // one past end of arena

bool s_mcb_initialised = false;

void mcb_init() {
  const PhysPt m = MCB_ARENA_START * 16u;
  mem_writeb(m + 0, 'Z');                                   // last block
  mem_writew(m + 1, 0);                                     // free
  mem_writew(m + 3, MCB_ARENA_END - MCB_ARENA_START - 1);   // size
  s_mcb_initialised = true;
}

// Walk the chain looking for a free block of at least `need` paragraphs.
// Returns MCB segment, or 0 on failure.  Sets `largest` to the biggest
// free block seen so AH=48h can report it.
uint16_t mcb_find_free(uint16_t need, uint16_t &largest) {
  largest = 0;
  uint16_t seg = MCB_ARENA_START;
  while (true) {
    const PhysPt m = seg * 16u;
    const uint8_t type = mem_readb(m);
    if (type != 'M' && type != 'Z') return 0;    // chain corrupt
    const uint16_t owner = mem_readw(m + 1);
    const uint16_t size  = mem_readw(m + 3);
    if (owner == 0) {
      if (size >= need) return seg;
      if (size > largest) largest = size;
    }
    if (type == 'Z') return 0;
    seg = static_cast<uint16_t>(seg + 1 + size);
  }
}

// Split a block if it's meaningfully larger than `need`.  Updates the
// current MCB and writes a new free MCB after the allocation.
void mcb_split(uint16_t mcb_seg, uint16_t need) {
  const PhysPt m = mcb_seg * 16u;
  const uint16_t cur = mem_readw(m + 3);
  const uint8_t  type = mem_readb(m);
  if (cur <= need + 1) return;          // too small to split profitably
  const uint16_t new_seg = static_cast<uint16_t>(mcb_seg + 1 + need);
  const PhysPt nm = new_seg * 16u;
  mem_writeb(nm + 0, type);             // inherit (possibly 'Z')
  mem_writew(nm + 1, 0);                // free
  mem_writew(nm + 3, cur - need - 1);
  mem_writeb(m + 0, 'M');               // current is no longer last
  mem_writew(m + 3, need);
}

// AH=48h -> returns data segment (MCB+1) or 0 on failure; on failure
// sets *largest_out to the largest free block for the DOS convention.
uint16_t mcb_allocate(uint16_t need, uint16_t &largest_out) {
  if (!s_mcb_initialised) mcb_init();
  const uint16_t seg = mcb_find_free(need, largest_out);
  if (seg == 0) return 0;
  mcb_split(seg, need);
  mem_writew(seg * 16u + 1, PSP_SEG);
  return static_cast<uint16_t>(seg + 1);
}

// AH=49h.  Mark the MCB free, then merge forward if the next block is
// also free.  Returns true on success, false on corrupt chain.
bool mcb_free(uint16_t data_seg) {
  if (!s_mcb_initialised) return false;
  const uint16_t mcb_seg = static_cast<uint16_t>(data_seg - 1);
  const PhysPt m = mcb_seg * 16u;
  const uint8_t type = mem_readb(m);
  if (type != 'M' && type != 'Z') return false;
  mem_writew(m + 1, 0);
  if (type == 'Z') return true;
  const uint16_t size     = mem_readw(m + 3);
  const uint16_t next_seg = static_cast<uint16_t>(mcb_seg + 1 + size);
  const PhysPt nm = next_seg * 16u;
  if (mem_readw(nm + 1) != 0) return true;     // next isn't free
  const uint8_t  next_type = mem_readb(nm);
  const uint16_t next_size = mem_readw(nm + 3);
  mem_writew(m + 3, static_cast<uint16_t>(size + 1 + next_size));
  mem_writeb(m + 0, next_type);                // inherit possibly 'Z'
  return true;
}

// AH=4Ah resize.  Returns the new size (possibly smaller than requested
// if we can't grow enough).  Writes `largest_out` to the max size we
// could have provided on failure.
uint16_t mcb_resize(uint16_t data_seg, uint16_t new_size,
                    uint16_t &largest_out) {
  largest_out = 0;
  if (!s_mcb_initialised) return 0;
  const uint16_t mcb_seg = static_cast<uint16_t>(data_seg - 1);
  const PhysPt m = mcb_seg * 16u;
  const uint8_t type = mem_readb(m);
  if (type != 'M' && type != 'Z') return 0;
  const uint16_t cur = mem_readw(m + 3);
  largest_out = cur;
  if (new_size == cur) return cur;
  if (new_size < cur) { mcb_split(mcb_seg, new_size); return new_size; }
  if (type == 'Z') return cur;                 // cannot grow; last block
  const uint16_t next_seg = static_cast<uint16_t>(mcb_seg + 1 + cur);
  const PhysPt nm = next_seg * 16u;
  if (mem_readw(nm + 1) != 0) return cur;      // next not free
  const uint16_t combined = static_cast<uint16_t>(cur + 1 + mem_readw(nm + 3));
  largest_out = combined;
  if (combined < new_size) return cur;         // not enough even combined
  // Consume next, then split back down if there's slack.
  mem_writeb(m + 0, mem_readb(nm));
  mem_writew(m + 3, combined);
  mcb_split(mcb_seg, new_size);
  return new_size;
}

// ---------------------------------------------------------------------------
// Protected-mode "big" memory arena for DPMI clients.
//
// The MCB arena caps out at ~512KB inside conventional memory.  That's fine
// for small 32-bit clients but a real DOS4G-hosted binary easily needs
// multi-megabyte allocations.  dosbox-staging configures 16MB of linear RAM
// by default; everything above 1MB is untouched by the DOS subsystem in our
// no-shell dosemu setup, so we claim it for DPMI memory services.
//
// The allocator is a simple first-fit free-list in linear bytes (page
// aligned).  Accesses go through dosbox's mem_write* which already serve
// extended memory correctly.  Clients point LDT descriptors at the returned
// linear address via AX=0007 to read/write the block in PM.
//
// We keep this decoupled from dosbox's MEM_AllocatePages because that
// allocator walks the whole page map (including conventional pages that
// overlap our MCB arena), which would cause collisions we'd have to guard
// against anyway.  Owning the [1MB, total_ram) span outright is simpler and
// matches how real DPMI hosts treat their extended-memory pool.
struct PmBlock {
  uint32_t base;     // page-aligned host linear address
  uint32_t size;     // page-aligned byte count
};
std::vector<PmBlock> s_pm_busy;   // allocated blocks, unsorted
bool     s_pm_initialised = false;
uint32_t s_pm_end         = 0;    // 1-past-last valid host byte

// pm_arena for DPMI-allocated linear blocks.  Starts at 1.125MB, leaving
// the first 128KB of extended memory (0x100000..0x11FFFF) for our kernel
// structures (GDT/IDT/LDT/TSS/shims/stacks -- see *_BASE constants below).
// Placing kernel above 1MB is required for ring-3 DPMI: if those tables
// sit in conventional memory, a ring-3 client's 16-bit RM-aliased selectors
// (base + 64KB limit) can reach them and overwrite them -- which is exactly
// what DJGPP's go32 stub does when it zeros its transfer buffer.
constexpr uint32_t PM_ARENA_START = 0x120000u;   // 1.125MB
constexpr uint32_t PM_PAGE        = 4096u;

void pm_init() {
  if (s_pm_initialised) return;
  // MEM_TotalPages() returns 4KB-page count configured via `memsize` in
  // dosbox.  Clamp the upper bound to that so we never issue addresses
  // that dosbox would treat as unmapped.
  const uint32_t total_pages = MEM_TotalPages();
  const uint64_t end = static_cast<uint64_t>(total_pages) * PM_PAGE;
  s_pm_end = (end > 0xFFFFFFFFu) ? 0xFFFFFFFFu : static_cast<uint32_t>(end);
  s_pm_initialised = true;
}

uint32_t pm_round_up(uint32_t bytes) {
  return (bytes + PM_PAGE - 1u) & ~(PM_PAGE - 1u);
}

// First-fit scan across [PM_ARENA_START, s_pm_end).  Returns a host linear
// address, or 0 on out-of-memory.  Zero-filled on allocation.
uint32_t pm_alloc(uint32_t bytes) {
  pm_init();
  if (bytes == 0 || s_pm_end <= PM_ARENA_START) return 0;
  const uint32_t rounded = pm_round_up(bytes);
  if (rounded < bytes) return 0;   // wrap
  std::sort(s_pm_busy.begin(), s_pm_busy.end(),
            [](const PmBlock &a, const PmBlock &b) { return a.base < b.base; });
  uint32_t cursor = PM_ARENA_START;
  uint32_t found  = 0;
  for (const auto &b : s_pm_busy) {
    if (b.base >= cursor && b.base - cursor >= rounded) {
      found = cursor;
      break;
    }
    cursor = b.base + b.size;
  }
  if (found == 0 && s_pm_end > cursor && s_pm_end - cursor >= rounded)
    found = cursor;
  if (found == 0) return 0;
  s_pm_busy.push_back({found, rounded});
  // Zero-fill via mem_writed in 4-byte chunks.  DJGPP and similar
  // clients assume fresh memory is zero-filled (their runtime checks
  // uninitialized struct fields against 0).  Using mem_writed (not
  // mem_writeb) keeps the loop fast enough that dosbox's timer/IRET
  // bookkeeping doesn't desync on multi-MB fills.
  for (uint32_t i = 0; i + 4 <= rounded; i += 4)
    mem_writed(found + i, 0);
  for (uint32_t i = (rounded & ~3u); i < rounded; ++i)
    mem_writeb(found + i, 0);
  return found;
}

bool pm_free(uint32_t base) {
  for (auto it = s_pm_busy.begin(); it != s_pm_busy.end(); ++it) {
    if (it->base == base) { s_pm_busy.erase(it); return true; }
  }
  return false;
}

// Resize in place: shrink always succeeds; grow succeeds if the range
// [base, base + new_rounded) doesn't collide with any other block and
// stays within s_pm_end.  No relocation -- the DPMI spec allows either
// behaviour and the existing MCB resize path also stays in place.
bool pm_resize(uint32_t base, uint32_t new_bytes) {
  pm_init();
  if (new_bytes == 0) return false;
  const uint32_t new_rounded = pm_round_up(new_bytes);
  if (new_rounded < new_bytes) return false;
  PmBlock *me = nullptr;
  for (auto &b : s_pm_busy) if (b.base == base) { me = &b; break; }
  if (!me) return false;
  if (new_rounded <= me->size) { me->size = new_rounded; return true; }
  if (base + new_rounded > s_pm_end) return false;
  for (const auto &b : s_pm_busy) {
    if (&b == me) continue;
    const uint32_t lo = b.base;
    const uint32_t hi = b.base + b.size;
    if (lo < base + new_rounded && hi > base) return false;
  }
  me->size = new_rounded;
  return true;
}

// Per-drive current directory (without drive letter, backslash-separated).
std::map<char, std::string> s_drive_cwd;

// DOS file-handle table.  Handles 0..2 are permanently bound to the host
// stdin/stdout/stderr file descriptors.  User opens get the next free slot
// starting at 5 (DOS conventionally reserves 0..4 for stdin/out/err/aux/prn).
constexpr int FIRST_FILE_HANDLE = 5;
constexpr int MAX_HANDLES       = 20;

struct HostHandle {
  int  fd          = -1;
  bool text_mode   = false;
  // For text-mode reads: an LF in the host file expands to CR LF for the
  // guest.  If CX fills up right after we emitted CR, the trailing LF is
  // held here until the next read call.
  int  read_pending = -1;
};
std::map<uint16_t, HostHandle> s_handles;

// Drive table: letter -> host directory.  Populated from cfg.drives (with
// CWD as a fallback for C: if no drives are specified).
std::map<char, std::string> s_drives;
char                        s_current_drive = 'C';

// Default translation mode for newly-opened files.  Captures
// cfg.default_mode so resolve_path can decide per-file when Auto selects
// by extension.
FileMode                    s_default_mode = FileMode::Auto;

// Per-file / per-pattern mappings from cfg.file_mappings.
std::vector<FileMapping>    s_file_mappings;

// Disk Transfer Area.  DOS programs set this via AH=1Ah and read results of
// findfirst/findnext from it.  Default is PSP:0080h (128-byte area).
uint32_t                    s_dta_linear = PSP_SEG * 16 + 0x80;

// Current-process PSP + env segment, as known by AH=4B.  Default is
// 0 (meaning "use PSP_SEG / ENV_SEG").  AH=4B sets these to the
// child's MCB-allocated segments while the child runs so
// dosemu_dpmi_entry aliases LDT[4]/LDT[5] to the child's memory (a
// hardcoded PSP_SEG would alias the parent's PSP in a nested
// DJGPP-under-DJGPP exec).  Restored on AH=4B cleanup.
uint16_t                    s_current_psp_seg = 0;
uint16_t                    s_current_env_seg = 0;

// State for AH=4Eh/4Fh findfirst/findnext.  Keyed by DTA linear address.
// Unlike a streaming readdir loop, we enumerate the whole directory up
// front so that 8.3 collision numbering is deterministic across runs.
struct FindEntry {
  std::string host_name;    // name on the host
  std::string mangled_8_3;  // DOS view, uppercased + ~N-resolved
};
struct FindState {
  std::vector<FindEntry> entries;  // filtered, already pattern-matched
  size_t                 index;    // next entry to return
  std::string            host_dir;
  uint32_t               token;    // echoed into DTA[4..7]
};
std::map<uint32_t, FindState> s_finds;
uint32_t s_next_find_token = 1;

void close_find_state(FindState &st) {
  st.entries.clear();
  st.index = 0;
}

// --- Host <-> guest helpers -----------------------------------------------

// Read a NUL-terminated DOS path from guest memory at seg:off.
std::string read_dos_string(uint16_t seg, uint16_t off, size_t max = 260) {
  const PhysPt base = seg * 16;
  std::string s;
  for (size_t i = 0; i < max; ++i) {
    const uint8_t c = mem_readb(base + off + i);
    if (c == 0) break;
    s += static_cast<char>(c);
  }
  return s;
}

std::string upper(const std::string &s) {
  std::string u = s;
  for (auto &c : u) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return u;
}

// Translate a DOS path (drive-letter + backslashes) to a host path.  DOS
// is case-insensitive; Linux isn't.  If the exact-case file doesn't
// exist, scan the parent directory for a case-insensitive match on the
// final segment -- resolves "C:\SRC.TXT" -> "/mnt/src.txt" when a real
// program (xcopy, compilers, etc.) uppercases filenames before open.
std::string dos_to_host(const std::string &dos_path) {
  std::string path = dos_path;
  char drive = s_current_drive;

  if (path.size() >= 2 && path[1] == ':') {
    drive = static_cast<char>(std::toupper(static_cast<unsigned char>(path[0])));
    path  = path.substr(2);
  }
  std::replace(path.begin(), path.end(), '\\', '/');

  auto it = s_drives.find(drive);
  const std::string base = (it == s_drives.end()) ? "" : it->second;

  std::string literal;
  if (path.empty())           literal = base;
  else if (path.front() == '/') literal = base + path;
  else if (base.empty())      literal = path;
  else                        literal = base + "/" + path;

  // If the exact path resolves, use it.
  struct stat st;
  if (::stat(literal.c_str(), &st) == 0) return literal;

  // Fall back to a case-insensitive walk segment by segment.  DOS paths
  // are case-insensitive and programs freely uppercase (DJGPP's argv0
  // does this).  On case-sensitive hosts (Linux CI), an exact match
  // fails for any segment whose disk case differs, so each segment is
  // resolved by scanning the parent directory.  On macOS (case-
  // insensitive), the first stat above succeeds and we never get here.
  std::string resolved;
  size_t p = 0;
  if (!literal.empty() && literal[0] == '/') { resolved = ""; p = 1; }
  while (p <= literal.size()) {
    const size_t next = literal.find('/', p);
    const std::string seg = literal.substr(p,
        next == std::string::npos ? std::string::npos : next - p);
    if (seg.empty()) { if (next == std::string::npos) break; p = next + 1; continue; }
    const std::string parent = resolved.empty() ? "/" : resolved;
    const std::string seg_u = upper(seg);
    DIR *d = ::opendir(parent.c_str());
    std::string match;
    if (d) {
      while (struct dirent *ent = ::readdir(d)) {
        if (upper(ent->d_name) == seg_u) { match = ent->d_name; break; }
      }
      ::closedir(d);
    }
    if (match.empty()) {
      // No case-insensitive hit -- return the best-effort path so the
      // caller's open() reports a useful ENOENT.
      return resolved + "/" + literal.substr(p);
    }
    resolved += "/" + match;
    if (next == std::string::npos) break;
    p = next + 1;
  }
  return resolved.empty() ? literal : resolved;
}

// Simple case-insensitive glob with * and ? — enough for DOS patterns.
// Linear glob with '*' / '?'.  Used for each 8.3 half separately below.
bool glob_match_part(const std::string &name_u, const std::string &pat_u) {
  size_t ni = 0, pi = 0;
  size_t star_pi = std::string::npos, star_ni = 0;
  while (ni < name_u.size()) {
    if (pi < pat_u.size() && (pat_u[pi] == '?' || pat_u[pi] == name_u[ni])) {
      ++ni; ++pi;
    } else if (pi < pat_u.size() && pat_u[pi] == '*') {
      star_pi = pi++;
      star_ni = ni;
    } else if (star_pi != std::string::npos) {
      pi = star_pi + 1;
      ni = ++star_ni;
    } else {
      return false;
    }
  }
  while (pi < pat_u.size() && pat_u[pi] == '*') ++pi;
  return pi == pat_u.size();
}

// DOS-semantic glob: split both name and pattern at their last '.',
// match base-vs-base and ext-vs-ext independently.  This is what makes
// "*.*" match "BIN" (empty ext matches pattern's "*") and "*.C" match
// "FOO.C" but not "FOO".
bool glob_match(const std::string &name_u, const std::string &pat_u) {
  auto split = [](const std::string &s) -> std::pair<std::string, std::string> {
    const size_t d = s.find_last_of('.');
    if (d == std::string::npos) return {s, ""};
    return {s.substr(0, d), s.substr(d + 1)};
  };
  auto [nb, ne] = split(name_u);
  auto [pb, pe] = split(pat_u);
  // If the pattern has no dot, match against the full name.
  if (pat_u.find('.') == std::string::npos) {
    return glob_match_part(name_u, pat_u);
  }
  return glob_match_part(nb, pb) && glob_match_part(ne, pe);
}

bool has_wildcard(const std::string &s) {
  return s.find_first_of("*?") != std::string::npos;
}

// Extract the filename from a DOS path (strip drive letter and directories).
std::string basename_dos(const std::string &dos_path) {
  std::string p = dos_path;
  if (p.size() >= 2 && p[1] == ':') p = p.substr(2);
  const size_t slash = p.find_last_of("\\/");
  if (slash != std::string::npos) p = p.substr(slash + 1);
  return p;
}

// Resolve a DOS path against cfg.file_mappings.  An exact (non-wildcard)
// match on the basename replaces both the host path and the mode.  A
// wildcard match only overrides the mode -- the host path still comes from
// the drive mount.  Falls back to dos_to_host() + s_default_mode.
struct Resolved {
  std::string host_path;
  bool        text_mode;
};

// Mangle a host filename into DOS 8.3 form with collision-aware numbering.
// If the upper-cased short form isn't already taken, use it as-is.  If it
// is, or if the base is too long for 8.3, emit BASEPFX~N.EXT with N bumped
// until unique among the names `used` already contains.
std::string mangle_8_3(const std::string &name, std::set<std::string> &used) {
  std::string base, ext;
  const size_t dot = name.find_last_of('.');
  if (dot == std::string::npos) { base = name; ext = ""; }
  else { base = name.substr(0, dot); ext = name.substr(dot + 1); }

  auto up = [](std::string &s) {
    for (auto &c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  };
  up(base);
  up(ext);
  if (ext.size() > 3) ext = ext.substr(0, 3);

  const bool long_form = base.size() > 8;
  if (!long_form) {
    std::string candidate = ext.empty() ? base : base + "." + ext;
    if (used.insert(candidate).second) return candidate;
    // Fell through: short name collided (case insensitive), fall through
    // to ~N resolution.
  }
  for (int n = 1; n < 1000; ++n) {
    const std::string nsuffix = "~" + std::to_string(n);
    size_t prefix_len = 8 - nsuffix.size();
    if (prefix_len > base.size()) prefix_len = base.size();
    const std::string candidate_base = base.substr(0, prefix_len) + nsuffix;
    const std::string candidate = ext.empty() ? candidate_base
                                              : candidate_base + "." + ext;
    if (used.insert(candidate).second) return candidate;
  }
  // Give up: let the last attempt stand even if duplicate.
  return name;
}

// DOS file-time pair: DS:1E format ((h<<11)|(m<<5)|(s/2)), date ((y-1980)<<9|(mo<<5)|d).
void dos_time_from_unix(time_t t, uint16_t &date, uint16_t &dostime) {
  struct tm lt;
  localtime_r(&t, &lt);
  dostime = static_cast<uint16_t>((lt.tm_hour << 11) | (lt.tm_min << 5)
                                  | (lt.tm_sec / 2));
  date    = static_cast<uint16_t>(((lt.tm_year + 1900 - 1980) << 9)
                                  | ((lt.tm_mon + 1) << 5) | lt.tm_mday);
}

// Write a matched entry to the current DTA.
void dta_write_entry(const std::string &host_full, const std::string &mangled_name,
                     uint32_t find_token = 0) {
  struct stat st{};
  ::stat(host_full.c_str(), &st);

  uint16_t dostime = 0, dosdate = 0;
  dos_time_from_unix(st.st_mtime, dosdate, dostime);

  // Zero reserved area (0x00..0x14).
  for (int i = 0; i < 0x15; ++i) mem_writeb(s_dta_linear + i, 0);
  // Magic + token (bytes 0..7): "DS" then 32-bit find state key.
  // Programs that reshuffle the DTA between findfirst and findnext
  // (GNU find does this) need the state key to travel with the data.
  if (find_token) {
    mem_writeb(s_dta_linear + 0, 'D');
    mem_writeb(s_dta_linear + 1, 'S');
    mem_writeb(s_dta_linear + 2, 'F');
    mem_writeb(s_dta_linear + 3, 'N');
    mem_writeb(s_dta_linear + 4, find_token        & 0xFF);
    mem_writeb(s_dta_linear + 5, (find_token >>  8) & 0xFF);
    mem_writeb(s_dta_linear + 6, (find_token >> 16) & 0xFF);
    mem_writeb(s_dta_linear + 7, (find_token >> 24) & 0xFF);
  }
  mem_writeb(s_dta_linear + 0x15, S_ISDIR(st.st_mode) ? 0x10 : 0x20);   // attr
  mem_writeb(s_dta_linear + 0x16, dostime & 0xFF);
  mem_writeb(s_dta_linear + 0x17, (dostime >> 8) & 0xFF);
  mem_writeb(s_dta_linear + 0x18, dosdate & 0xFF);
  mem_writeb(s_dta_linear + 0x19, (dosdate >> 8) & 0xFF);
  const uint32_t size = static_cast<uint32_t>(st.st_size);
  mem_writeb(s_dta_linear + 0x1A,  size        & 0xFF);
  mem_writeb(s_dta_linear + 0x1B, (size >>  8) & 0xFF);
  mem_writeb(s_dta_linear + 0x1C, (size >> 16) & 0xFF);
  mem_writeb(s_dta_linear + 0x1D, (size >> 24) & 0xFF);
  // Filename field is 13 bytes NUL-terminated.
  size_t i = 0;
  for (; i < mangled_name.size() && i < 12; ++i)
    mem_writeb(s_dta_linear + 0x1E + i, static_cast<uint8_t>(mangled_name[i]));
  for (; i < 13; ++i) mem_writeb(s_dta_linear + 0x1E + i, 0);
}

// Split a DOS path like "SUBDIR\*.C" into (dir, pattern) where dir can
// still contain a drive letter (dos_to_host will handle that).
std::pair<std::string, std::string> split_dir_pattern(const std::string &dos_path) {
  const size_t slash = dos_path.find_last_of("\\/");
  if (slash == std::string::npos) return {"", dos_path};
  return {dos_path.substr(0, slash), dos_path.substr(slash + 1)};
}

// Return the next pre-computed entry from st.  Writes DTA and returns
// true, or closes the state and returns false when exhausted.
bool scan_next(FindState &st) {
  if (st.index >= st.entries.size()) {
    close_find_state(st);
    return false;
  }
  const auto &e = st.entries[st.index++];
  dta_write_entry(st.host_dir + "/" + e.host_name, e.mangled_8_3, st.token);
  return true;
}

// Enumerate host_dir upfront: readdir + sort + pre-mangle with collision
// tracking + filter by glob.  Deterministic across runs.
bool build_find_state(const std::string &host_dir,
                      const std::string &pattern_u,
                      FindState &out) {
  DIR *dir = ::opendir(host_dir.c_str());
  if (!dir) return false;

  std::vector<std::string> names;
  while (struct dirent *ent = ::readdir(dir)) {
    const std::string n = ent->d_name;
    if (n == "." || n == "..") continue;
    names.push_back(n);
  }
  ::closedir(dir);
  std::sort(names.begin(), names.end());

  std::set<std::string> used;
  out.entries.clear();
  out.host_dir = host_dir;
  out.index    = 0;
  // Real DOS subdirectories always list "." and ".." first.  Many
  // DOS ports of POSIX tools (GNU find, ls via opendir) rely on
  // that to stat the directory itself -- they call findfirst(".")
  // expecting a hit with attr=DIR.  Include them up front and let
  // glob_match filter when the caller narrows (e.g., "*.C").
  for (const auto *dotname : {".", ".."}) {
    if (glob_match(dotname, pattern_u)) {
      out.entries.push_back({dotname, dotname});
    }
  }
  for (const auto &n : names) {
    const std::string m = mangle_8_3(n, used);
    if (glob_match(m, pattern_u)) {
      out.entries.push_back({n, m});
    }
  }
  return true;
}

// Text-by-extension heuristic for FileMode::Auto.  Extensions commonly
// associated with text content: source, config, data, scripts.
bool ext_is_textual(const std::string &base_u) {
  static const std::vector<std::string> kTextExt = {
    "TXT","ASM","S","C","H","CC","CPP","CXX","HPP","HXX","INC",
    "CFG","INI","BAT","LOG","MD","BAS","PAS","PL","PY","RB","RC",
    "DEF","MAK","MAP","LST","TBL","SRT","DAT","CSV","XML","HTML","HTM",
    "JSON","YAML","YML","SH","JS","TS","TEX","DIF","PATCH","ERR"
  };
  const size_t dot = base_u.find_last_of('.');
  if (dot == std::string::npos) return false;
  const std::string ext = base_u.substr(dot + 1);
  return std::find(kTextExt.begin(), kTextExt.end(), ext) != kTextExt.end();
}

bool mode_is_text(FileMode mode, const std::string &base_u) {
  if (mode == FileMode::Text)   return true;
  if (mode == FileMode::Binary) return false;
  return ext_is_textual(base_u);  // Auto
}

Resolved resolve_path(const std::string &dos_path) {
  const std::string base_u = upper(basename_dos(dos_path));

  // Pass 1: exact (non-wildcard) match -- remap path + mode.
  for (const auto &m : s_file_mappings) {
    if (!has_wildcard(m.pattern) && upper(m.pattern) == base_u) {
      return {m.host_path, mode_is_text(m.mode, base_u)};
    }
  }
  // Pass 2: wildcard -- first match wins, mode-only override.
  FileMode resolved_mode = s_default_mode;
  for (const auto &m : s_file_mappings) {
    if (has_wildcard(m.pattern) && glob_match(base_u, upper(m.pattern))) {
      resolved_mode = m.mode;
      break;
    }
  }
  return {dos_to_host(dos_path), mode_is_text(resolved_mode, base_u)};
}

// Allocate the next free DOS handle for a given host fd.
int allocate_handle(int fd, bool text_mode) {
  for (int h = FIRST_FILE_HANDLE; h < MAX_HANDLES; ++h) {
    if (s_handles.find(h) == s_handles.end()) {
      s_handles[h] = HostHandle{fd, text_mode, -1};
      return h;
    }
  }
  return -1;
}

// --- INT 21h dispatcher ----------------------------------------------------

// Gate-bitness tracker used by set_cf to find the FLAGS word in the
// stacked interrupt frame.  Set by the bitness-specific callback
// wrappers (dosemu_int21_32 etc.) before the real handler runs and
// cleared afterwards.  Defaults to false (16-bit) because the
// original real-mode INT 21h callback is entered with CS still
// 16-bit and no gate prefix is present.
bool s_int_gate_bits32 = false;

void set_cf(bool val) {
  // Flip bit 0 (CF) on the FLAGS word the CPU will pop on IRET.
  // Frame layout is decided by the GATE that dispatched us, not the
  // handler's CS -- our PM callbacks all run with CS = PM_CB_SEL
  // (16-bit regardless), so cpu.code.big is always false in PM even
  // when we got here through a 32-bit gate.  s_int_gate_bits32 is
  // set by the bitness-specific dosbox callback wrapper so we know
  // the real frame layout.
  //
  // Real-mode / 16-bit gate frame:  [SS:SP+4] = 16-bit FLAGS
  // 32-bit gate frame:              [SS:SP+8] = low word of EFLAGS
  //
  // Writing the flag word into the wrong offset clobbers the CS
  // word above it, which on IRETD comes back as "CS | 1" -- RPL=1
  // != old CPL=0 -> "IRET:Outer level:Stack segment not writable".
  // That's what tripped wd.exe repeatedly.
  const PhysPt ss_base    = SegPhys(ss);
  const uint32_t sp_val   = cpu.stack.big ? reg_esp : reg_sp;
  const unsigned flags_off = (cpu.pmode && s_int_gate_bits32) ? 8u : 4u;
  const PhysPt addr = ss_base + sp_val + flags_off;
  uint16_t f = mem_readw(addr);
  if (val) f |=  0x0001;
  else     f &= ~0x0001;
  mem_writew(addr, f);
}

void return_error(uint16_t dos_err) {
  reg_ax = dos_err;
  set_cf(true);
}

// INT 2Fh multiplex: handle the DPMI-present probe (AX=1687h) so programs
// see a DPMI host.  Everything else falls through to dosbox's default
// handler via IRET without modification.  When real DPMI lands, the entry
// point returned here will transition to protected mode; until then, our
// dosemu_dpmi_entry callback returns AX=8001h ("unsupported function")
// with CF=1 so the client fails the mode switch gracefully.
uint16_t                s_dpmi_entry_seg    = 0;
uint16_t                s_dpmi_entry_off    = 0;

// ---- DPMI stage 3: real -> protected-mode switch ------------------------
//
// Layout: a fixed guest segment holds our GDT.  Entries:
//   [0] null descriptor
//   [1] = 0x08  code segment for client's CS    (base = CS * 16, limit 64K, 16-bit)
//   [2] = 0x10  data segment for client's DS    (base = DS * 16, limit 64K, 16-bit)
//   [3] = 0x18  stack segment for client's SS   (base = SS * 16, limit 64K, 16-bit)
//   [4] = 0x20  extra segment for client's ES   (base = ES * 16, limit 64K, 16-bit)
// Descriptor selectors are the byte offsets (RPL=0, TI=0 for GDT).
//
// After the switch the client runs with those selectors in CS/DS/SS/ES.
// Client IP is unchanged (linear address same as in real mode, reached
// via the new selector).
//
// Limitations to be addressed in stages 4-7:
//   - only four fixed descriptors; INT 31h AX=0000/0001 (alloc/free LDT)
//     not implemented
//   - no IDT: any INT from PM will triple-fault.  INT 21h reflection
//     back to real mode is stage 5.
//   - no memory allocation: INT 31h AX=0501 et al. return "unsupported".
//   - 32-bit (AX=1) entry handled like 16-bit for now.
// Kernel structure bases sit above 1MB so a ring-3 client's 16-bit
// RM-aliased selectors (max reach = seg_base + 0xFFFF <= 0x10FFEF) cannot
// overwrite them.  Pre-refactor these were at 0x18000-0x1F000 in
// conventional memory; DJGPP/go32 corrupted our IDT during startup.
constexpr uint32_t GDT_BASE        = 0x100000u;
constexpr uint32_t IDT_BASE        = 0x102000u;
constexpr uint32_t LDT_BASE        = 0x104000u;
constexpr uint32_t PM_SHIM_BASE    = 0x106000u;
constexpr uint32_t PM_CB_STACK_BASE= 0x108000u;
constexpr uint32_t TSS_BASE        = 0x109000u;
// GDT layout: 16 entries.  First 9 are the historical ring-0 DPMI
// selectors.  Remaining 7 are additions for CWSDPMI-style ring-3
// operation (DOSEMU_DPMI_RING3=1): TSS for inter-ring transitions,
// ring-3 aliases of code/data/stack, ring-3 scratch stack.  The
// ring-3 entries are populated at DPMI-entry time only when the
// flag is set; ring-0 path ignores them.
constexpr uint16_t GDT_LIMIT  = 0x7F;         // 16 entries * 8 bytes - 1
constexpr uint16_t PM_CS_SEL   = 0x08;        // ring-0 client code (legacy)
constexpr uint16_t PM_DS_SEL   = 0x10;        // ring-0 client data
constexpr uint16_t PM_SS_SEL   = 0x18;        // ring-0 client stack
constexpr uint16_t PM_ES_SEL   = 0x20;        // ring-0 PSP/ES
constexpr uint16_t PM_CB_SEL   = 0x28;        // CB_SEG 16-bit code
constexpr uint16_t PM_LDT_SEL  = 0x30;        // GDT[6]: LDT descriptor
constexpr uint16_t PM_SHIM_SEL = 0x38;        // GDT[7]: 32-bit reflection shim code
constexpr uint16_t PM_CB_STACK = 0x40;        // GDT[8]: PM scratch stack (callbacks)
constexpr uint32_t PM_CB_STACK_SIZE = 0x1000;
// Ring-3 DPMI additions (GDT slots 9..15).  Selector values include
// the DPL-3 encoding: `idx*8 | 3`.  For example PM_CS3_SEL = 0x50+3
// = 0x53; the RPL is baked in so the selector itself specifies ring.
constexpr uint16_t PM_TSS_SEL  = 0x48;        // GDT[9]:  TSS descriptor (DPL=0)
// Unused symbols kept for GDT-layout documentation: the ring-3 code/
// data/stack/ES aliases live in dedicated slots so it's obvious which
// GDT index holds which selector, even though we read/write them via
// the table rather than these names.
[[maybe_unused]] constexpr uint16_t PM_CS3_SEL  = 0x53;  // GDT[10]: ring-3 code (DPL=3)
[[maybe_unused]] constexpr uint16_t PM_DS3_SEL  = 0x5B;  // GDT[11]: ring-3 data (DPL=3)
[[maybe_unused]] constexpr uint16_t PM_SS3_SEL  = 0x63;  // GDT[12]: ring-3 stack (DPL=3)
[[maybe_unused]] constexpr uint16_t PM_ES3_SEL  = 0x6B;  // GDT[13]: ring-3 ES/PSP (DPL=3)
// GDT[14] = ring-3 CB alias; LRET target for user_exception_return.
constexpr uint16_t PM_CB3_SEL  = 0x73;        // GDT[14]: CB_SEG at DPL=3
// GDT[15] = ring-3 data alias of the host exception-handler stack.
// CWSDPMI-style: the client's SS/ESP may be corrupt at the moment the
// exception fires, so we never trust them -- the trampoline switches
// SS:ESP to this private stack before dispatching to the user handler.
constexpr uint16_t PM_EXC_STACK_SEL = 0x7B;   // GDT[15]: DPL=3 data

// TSS lives at fixed physical address.  Only SS0:ESP0 field matters
// for our ring-3 use (ring-3 -> ring-0 stack switch on interrupts).
constexpr uint32_t TSS_SIZE  = 104;            // 32-bit TSS minimum
// Ring-0 stack: reuse CB_STACK area (4KB) that's otherwise used for
// AX=0303 callback scratch.  During a ring-3-client ring transition
// the CPU will load SS=PM_CB_STACK, ESP=PM_CB_STACK_BASE+PM_CB_STACK_SIZE.
[[maybe_unused]] constexpr uint32_t PM_RING0_STACK_TOP = PM_CB_STACK_BASE + PM_CB_STACK_SIZE;

// 32-bit PM reflection shims.  Each slot is an 8-byte 16-bit code
// sequence that re-invokes the dosbox native callback for a real-mode
// IVT vector and then does IRETD instead of plain IRET, so a 32-bit
// interrupt gate's 12-byte EIP/CS/EFLAGS frame is correctly unwound.
// Shim layout (6 meaningful bytes, padded to 8):
//   FE 38 cb_lo cb_hi 66 CF 90 90
// The `FE 38 LL HH` is dosbox's native-callback opcode with the cb_num
// copied from the existing RM stub the real-mode IVT points at.
constexpr uint16_t PM_SHIM_SLOT_BYTES = 16;   // fits err-code discard + IRETD
constexpr uint16_t PM_SHIM_TOTAL      = 256 * PM_SHIM_SLOT_BYTES;

constexpr uint16_t IDT_LIMIT  = 0x7FF;        // 256 entries * 8 bytes - 1

// LDT for DPMI client-allocated descriptors (INT 31h AX=0000/0001/0002).
// 256 entries * 8 bytes = 2KB, placed at LDT_BASE above 1MB.
// Index 0 in the LDT is reserved null, so client selectors start at 0x000C
// (idx=1, TI=1, RPL=0 -- ring-0 default; DOSEMU_DPMI_RING3=1 promotes
// selector RPLs to 3 so clients run at ring 3).
constexpr uint16_t LDT_BYTES  = 256u * 8u;    // 2KB
constexpr uint16_t LDT_COUNT  = 256;

// In-use bitmap: 1 bit per LDT slot, bit set = allocated.  LDT slot 0 is
// treated as permanently reserved (null descriptor) so it always reads
// as allocated, keeping "first-fit from index 1" logic simple.
uint8_t s_ldt_in_use[(LDT_COUNT + 7) / 8];

// AX=0002 "segment to descriptor" must return the same selector for
// repeated conversions of the same real-mode segment, per DPMI spec.
// A sparse map real-mode-segment (0..0xFFFF) -> LDT index caches that.
// 0 means "not yet aliased" (LDT slot 0 is reserved so never a valid
// cached target).  128KB static; fine for a host process.
uint16_t s_seg2desc_cache[65536];

// Real pointer of our INT 21h callback.  Captured in dosemu_startup and
// used to build an IDT interrupt gate when transitioning to PM, so INT 21h
// from protected-mode clients routes through the same host-C++ handler
// (memory accesses use SegPhys(), which returns the descriptor base in PM
// and seg*16 in RM -- transparent to the caller).
//
// Two callbacks exist because the IRET at the end of the stub is
// size-sensitive: a 32-bit DPMI entry installs a 32-bit IDT gate which
// pushes a 32-bit EIP/CS/EFLAGS frame, and the stub must end in IRETD
// (`66 CF`) to pop it correctly; the 16-bit path ends in plain IRET
// (`CF`).  Both stubs run in the 16-bit compatibility selector 0x28 and
// dispatch to the same native dosemu_int21 handler.
uint16_t s_int21_cb_seg    = 0;
uint16_t s_int21_cb_off    = 0;
uint16_t s_int21_cb32_seg  = 0;
uint16_t s_int21_cb32_off  = 0;

// INT 31h (DPMI services) callback addresses -- both bitnesses, same as
// INT 21h.  Needed because real DPMI clients call INT 31h from PM; the
// stage-1 `Set_RealVec(0x31)` covers only the real-mode IVT.
uint16_t s_int31_cb_off    = 0;
uint16_t s_int31_cb32_off  = 0;
uint16_t s_le_exc_cb16_off = 0;    // CB_IRET  shared handler
uint16_t s_le_exc_cb32_off = 0;    // CB_IRETD shared handler

// Per-vector PM exception trampoline offsets, and the
// user_exception_return trampoline -- all in CB_SEG.  Built at startup
// so AX=0203 just has to install the corresponding IDT gate pointing at
// the per-vector trampoline.  The trampoline constructs the CWSDPMI-
// style 8-dword exception frame on the client's stack so DJGPP's
// exception_handler sees its expected layout.
uint16_t s_pm_exc_cb32_off[32] = {};
uint16_t s_pm_exc_ret_off      = 0;    // user_exception_return trampoline

// Forward decls for LDT helpers defined deeper in the file, used by
// dpmi_enter_pm_mode's ring-3 starter-set.
void write_ldt_descriptor(int idx, uint32_t base, uint32_t limit,
                          uint8_t access, bool bits32 = false);
inline void ldt_set(uint16_t idx, bool v);

void write_gdt_descriptor(int idx, uint32_t base, uint32_t limit,
                          uint8_t access, bool bits32 = false) {
  const PhysPt p = GDT_BASE + idx * 8u;
  mem_writeb(p + 0, limit & 0xFF);
  mem_writeb(p + 1, (limit >> 8) & 0xFF);
  mem_writeb(p + 2, base & 0xFF);
  mem_writeb(p + 3, (base >> 8) & 0xFF);
  mem_writeb(p + 4, (base >> 16) & 0xFF);
  mem_writeb(p + 5, access);
  // Byte 6 high nibble: G=0 (byte granular), D=1 for 32-bit segments.
  const uint8_t flags = bits32 ? 0x40 : 0x00;
  mem_writeb(p + 6, ((limit >> 16) & 0x0F) | flags);
  mem_writeb(p + 7, (base >> 24) & 0xFF);
}

// Write an interrupt gate to IDT[idx].  Target selector:offset is where
// control transfers when INT idx fires in PM.  Type byte 0x86 = 16-bit
// interrupt gate, 0x8E = 32-bit interrupt gate.
void write_idt_gate(int idx, uint16_t sel, uint32_t off, bool bits32) {
  const PhysPt p = IDT_BASE + idx * 8u;
  mem_writeb(p + 0, off & 0xFF);
  mem_writeb(p + 1, (off >> 8) & 0xFF);
  mem_writeb(p + 2, sel & 0xFF);
  mem_writeb(p + 3, (sel >> 8) & 0xFF);
  mem_writeb(p + 4, 0);
  // Gate access byte: P=1, DPL=3, S=0, type (8E = 32-bit int gate,
  // 86 = 16-bit).  DPL=3 so ring-3 clients can software-INT into
  // the gate.  For CPU-delivered exceptions (not caused by an INT
  // instruction) DPL doesn't matter.
  mem_writeb(p + 5, bits32 ? 0xEE : 0xE6);
  mem_writeb(p + 6, (off >> 16) & 0xFF);
  mem_writeb(p + 7, (off >> 24) & 0xFF);
}

// Shared PM setup: build GDT, IDT, and CPU_LGDT/CPU_LIDT.  Caller
// supplies the client's RM segment values so GDT[1..4] can alias them.
// Used by both the DPMI entry (retf-into-PM path) and le_launch_pm
// (no-caller path).  Caller is responsible for:
//   - Resetting the LDT (if needed).
//   - Flipping CR0.PE.
//   - Issuing CPU_LLDT.
//   - Loading segment selectors.
void pm_setup_gdt_and_idt(bool bits32, uint16_t client_cs,
                          uint16_t client_ds, uint16_t client_ss,
                          uint16_t client_es) {
  // Enable A20 so writes to our kernel structures (GDT/IDT/LDT/TSS/
  // shims/stack at 0x100000+) actually land there instead of wrapping
  // to 0x00000-0x0FFFF.  Default DOS state has A20 disabled (hence the
  // "A20 wrap" required by real-mode 8086 compat).  Real DPMI hosts
  // enable A20 during their init for the same reason.  Without this,
  // our CPU_LIDT points at 0x100000 but writes to that address actually
  // corrupt the RM IVT / BIOS data area.
  MEM_A20_Enable(true);

  write_gdt_descriptor(0, 0,              0,      0);                 // null
  write_gdt_descriptor(1, client_cs * 16, 0xFFFF, 0x9A, bits32);      // code
  write_gdt_descriptor(2, client_ds * 16, 0xFFFF, 0x92);              // data
  write_gdt_descriptor(3, client_ss * 16, 0xFFFF, 0x92);              // stack
  write_gdt_descriptor(4, client_es * 16, 0xFFFF, 0x92);              // es
  write_gdt_descriptor(5, 0xF0000, 0xFFFF, 0x9A);                    // cb
  write_gdt_descriptor(6, LDT_BASE, LDT_BYTES - 1, 0x82);
  write_gdt_descriptor(7, PM_SHIM_BASE, PM_SHIM_TOTAL - 1, 0x9A);
  write_gdt_descriptor(8, PM_CB_STACK_BASE, PM_CB_STACK_SIZE - 1, 0x92,
                       /*bits32=*/true);  // D=1 so reg_esp is 32-bit
  // Ring-3 DPMI descriptors.  Populated unconditionally so the GDT
  // table has consistent state, but only ACTIVATED (via segment
  // loads + CPU_JMP) on the DOSEMU_DPMI_RING3 path.  Existing ring-0
  // clients ignore these slots.
  // TSS: 32-bit TSS (access byte 0x89 = present, DPL=0, system, type 9).
  write_gdt_descriptor(9, TSS_BASE, TSS_SIZE - 1, 0x89);
  // Ring-3 code: access 0xFA = P=1, DPL=3, S=1, type=1010 (code, r).
  write_gdt_descriptor(10, client_cs * 16, 0xFFFF, 0xFA, bits32);
  // Ring-3 data: access 0xF2 = P=1, DPL=3, S=1, type=0010 (data, rw).
  write_gdt_descriptor(11, client_ds * 16, 0xFFFF, 0xF2);
  // Ring-3 stack (alias of ring-0 stack for now).
  write_gdt_descriptor(12, client_ss * 16, 0xFFFF, 0xF2);
  // Ring-3 ES (PSP alias).
  write_gdt_descriptor(13, client_es * 16, 0xFFFF, 0xF2);
  // GDT[14] = ring-3 alias of the CB_SEG (0xF000) code segment.  Used
  // as user_exception_return's CS when the CWSDPMI-style exception
  // trampoline hands off to the user handler: the handler (ring-3)
  // LRETs through (PM_CB3_SEL:s_pm_exc_ret_off); LRET can't transfer
  // to lower-DPL CS, so user_ret_CS must be DPL=3.  Access 0xFA =
  // P=1, DPL=3, S=1, type=1010 (code, readable, non-conforming).
  write_gdt_descriptor(14, 0xF0000, 0xFFFF, 0xFA);
  // GDT[15] = ring-3 data alias of the PM callback stack.  Used by
  // the PM-exception trampoline as a known-good stack regardless of
  // how mangled the client's SS:ESP is at fault time.
  write_gdt_descriptor(15, PM_CB_STACK_BASE, PM_CB_STACK_SIZE - 1, 0xF2);

  // TSS body: zero-fill, then set SS0 / ESP0 so the CPU can switch
  // stacks when transitioning ring 3 -> ring 0 on an interrupt.
  // ESP0 is an offset WITHIN the SS0 segment (stack top).
  for (uint32_t i = 0; i < TSS_SIZE; ++i)
    mem_writeb(TSS_BASE + i, 0);
  mem_writed(TSS_BASE + 4, PM_CB_STACK_SIZE);     // ESP0 = 0x1000 (top)
  mem_writew(TSS_BASE + 8, PM_CB_STACK);          // SS0 = PM_CB_STACK

  CPU_LGDT(GDT_LIMIT, GDT_BASE);

  const uint16_t int21_cb_off = bits32 ? s_int21_cb32_off : s_int21_cb_off;
  const uint16_t int31_cb_off = bits32 ? s_int31_cb32_off : s_int31_cb_off;
  for (int i = 0; i < 256; ++i) write_idt_gate(i, 0, 0, bits32);

  for (int v = 0; v < 256; ++v) {
    if (v == 0x21 || v == 0x31) continue;
    const uint32_t ivt = mem_readd(static_cast<uint32_t>(v) * 4u);
    const uint16_t seg = (ivt >> 16) & 0xFFFF;
    const uint16_t off = ivt & 0xFFFF;
    if (seg != CB_SEG) continue;
    if (!bits32) {
      write_idt_gate(v, PM_CB_SEL, off, false);
      continue;
    }
    const PhysPt stub = static_cast<PhysPt>(seg) * 16u + off;
    int scan = -1;
    for (int i = 0; i < 5; ++i) {
      if (mem_readb(stub + i) == 0xFE && mem_readb(stub + i + 1) == 0x38) {
        scan = i; break;
      }
    }
    if (scan < 0) continue;
    const uint8_t cb_lo = mem_readb(stub + scan + 2);
    const uint8_t cb_hi = mem_readb(stub + scan + 3);
    const uint16_t slot_off = v * PM_SHIM_SLOT_BYTES;
    const PhysPt shim = PM_SHIM_BASE + slot_off;
    // Exceptions 8, 10..14, 17 push a 4-byte error code AFTER
    // EIP/CS/EFLAGS in 32-bit mode.  If we run the plain shim
    // (FE 38 LL HH; 66 CF) on one of those, our IRETD pops the
    // error code as EIP, the real EIP as CS, and the real CS as
    // EFLAGS -- garbage, triggering "IRET:Illegal descriptor type
    // 0x0".  For those vectors, prepend `add esp, 4` (66 83 C4 04)
    // to pop the error code before the native call + IRETD.
    const bool has_err = (v == 8 || (v >= 10 && v <= 14) || v == 17);
    int i = 0;
    if (has_err) {
      mem_writeb(shim + i++, 0x66);   // 32-bit operand-size prefix
      mem_writeb(shim + i++, 0x83);   // add r/m32, imm8
      mem_writeb(shim + i++, 0xC4);   // modrm: ESP
      mem_writeb(shim + i++, 0x04);   // +4
    }
    mem_writeb(shim + i++, 0xFE);   // native-call marker byte 1
    mem_writeb(shim + i++, 0x38);   // native-call marker byte 2
    mem_writeb(shim + i++, cb_lo);
    mem_writeb(shim + i++, cb_hi);
    mem_writeb(shim + i++, 0x66);   // 32-bit prefix -> IRETD
    mem_writeb(shim + i++, 0xCF);   // IRETD
    for (; i < PM_SHIM_SLOT_BYTES; ++i) mem_writeb(shim + i, 0x90);
    write_idt_gate(v, PM_SHIM_SEL, slot_off, true);
  }
  if (int21_cb_off)
    write_idt_gate(0x21, PM_CB_SEL, int21_cb_off, bits32);
  if (int31_cb_off)
    write_idt_gate(0x31, PM_CB_SEL, int31_cb_off, bits32);
  CPU_LIDT(IDT_LIMIT, IDT_BASE);
}

Bitu dosemu_dpmi_entry() {
  // Called via FAR CALL from real-mode DPMI clients after AX=1687h.
  // Stack layout at entry:  [SP] = client IP, [SP+2] = client CS
  // (the stub's CB is 'retf', so we must leave those words there -- but we
  // overwrite CS with our PM code selector so retf takes us into PM).
  //
  // The switch is followed up by full DPMI 0.9 support: LDT descriptor
  // allocation, INT 21h reflection through the PM IDT, linear memory
  // allocation (MCB + pm_arena tier), RM-callback registration,
  // exception dispatch, virtual IF state, and mode-switch primitives
  // for calling back into real mode.  See dosemu_int31 for the full
  // sub-function set and the DPMI_* test fixtures for per-sub-function
  // verification.

  const uint16_t client_cs = mem_readb(SegValue(ss) * 16u + reg_sp + 2)
                           | (mem_readb(SegValue(ss) * 16u + reg_sp + 3) << 8);
  // Client IP is the word below (unused here -- we don't relocate it).

  // AX on entry: 0 = 16-bit PM, 1 = 32-bit PM.
  const bool bits32 = (reg_ax == 1);

  pm_setup_gdt_and_idt(bits32, client_cs, SegValue(ds),
                       SegValue(ss), SegValue(es));

  // Zero the LDT so every unallocated slot reads as a not-present
  // descriptor (access byte 0), then reset the in-use bitmap to match.
  for (uint32_t off = 0; off < LDT_BYTES; ++off)
    mem_writeb(LDT_BASE + off, 0);
  for (auto &b : s_ldt_in_use) b = 0;
  s_ldt_in_use[0] |= 0x01;   // slot 0 reserved (null)
  for (auto &c : s_seg2desc_cache) c = 0;

  // Default PM exception handling: catch-all le_exc gate for vectors
  // 0x00-0x0F that terminates + logs.  Clients override per-vector via
  // AX=0203; for 32-bit clients the AX=0203 handler upgrades the gate
  // to our CWSDPMI-style trampoline (see s_pm_exc_cb32_off / the
  // dosemu_pm_exc_dispatch dispatcher).  16-bit clients keep the
  // direct-to-handler gate since their IRET frame is the plain
  // CPU-pushed 6-byte form.
  const uint16_t exc_cb = bits32 ? s_le_exc_cb32_off : s_le_exc_cb16_off;
  if (exc_cb) {
    for (int v = 0; v < 0x10; ++v)
      write_idt_gate(v, PM_CB_SEL, exc_cb, bits32);
  }

  // Ring-3 DPMI entry.  This is what real hosts (CWSDPMI, Windows,
  // QEMM) give clients -- CPL=3 with the host at CPL=0, so exceptions
  // cleanly transition through the TSS.  We now default to this for
  // 32-bit clients because every real-world DPMI binary (DJGPP, DOS4GW
  // programs via the host) expects it; legacy in-tree fixtures that
  // were written against the old ring-0 path can opt back in with
  // DOSEMU_DPMI_RING0.  DOSEMU_DPMI_RING3 is still honoured for
  // backward compatibility with any scripts that set it.
  const bool want_ring3 = bits32 && !dosemu::g_debug.dpmi_ring0;
  if (want_ring3) {
    // CWSDPMI-style ring-3 entry: allocate LDT slots for the client's
    // CS/DS/SS/ES aliases (not GDT slots -- real DPMI hosts put
    // client selectors in the LDT, so stub code that checks TI bit
    // gets the expected answer).  Slots 1..4 are a conventional
    // "starter set" per CWSDPMI's l_acode / l_adata / l_apsp.
    const uint16_t client_ip = mem_readw(SegValue(ss) * 16u + reg_sp);
    // These bases MUST be uint32_t.  uint16_t truncates for any RM segment
    // value >= 0x1000 (since seg*16 then >= 0x10000) -- observed with
    // DJGPP go32-v2 whose ES at DPMI entry was 0x2001, giving a truncated
    // LDT[4] base of 0x0010 (= 0x20010 & 0xFFFF).  That sent DJGPP libc's
    // psp_selector:0x2C reads into the RM IVT instead of the PSP, producing
    // the 0xF4 DS-load mystery documented in the prior session.
    const uint32_t cs_base  = client_cs  * 16u;
    const uint32_t ds_base  = SegValue(ds) * 16u;
    const uint32_t ss_base  = SegValue(ss) * 16u;
    const uint32_t es_base  = SegValue(es) * 16u;
    if (dosemu::g_debug.dpmi_trace) {
      std::fprintf(stderr,
          "[dpmi-entry] cs=%04x ip=%04x ds=%04x ss=%04x es=%04x fs=%04x gs=%04x sp=%04x -> bases "
          "cs=%08x ds=%08x ss=%08x es=%08x\n",
          (unsigned)client_cs, (unsigned)client_ip,
          (unsigned)SegValue(ds), (unsigned)SegValue(ss),
          (unsigned)SegValue(es),
          (unsigned)SegValue(fs), (unsigned)SegValue(gs),
          (unsigned)reg_sp,
          cs_base, ds_base, ss_base, es_base);
    }

    CPU_SET_CRX(0, 0x00000001);      // PE=1
    CPU_LLDT(PM_LDT_SEL);
    CPU_LTR(PM_TSS_SEL);
    s_client_cpl = 3;

    // Zero-init LDT bitmap + reserve slots 1..4 for starter-set.
    for (auto &b : s_ldt_in_use) b = 0;
    s_ldt_in_use[0] |= 0x01;   // slot 0 reserved (null)
    for (auto &c : s_seg2desc_cache) c = 0;
    for (uint32_t off = 0; off < LDT_BYTES; ++off)
      mem_writeb(LDT_BASE + off, 0);
    // CWSDPMI convention (CONTROL.C line 469): client CS is always
    // 16-bit regardless of the AX=1 "32-bit DPMI" entry.  The
    // initial selector just aliases the caller's RM CS; stubs that
    // want 32-bit code alloc their own 32-bit LDT descriptor via
    // AX=0000 and far-jump to it.  DJGPP's go32 stub does exactly
    // this.  Our earlier attempt to honour "bits32" and hand out a
    // 32-bit CS up front misreads the stub's 16-bit setup code
    // (observed #UD at go32 stub offset 0x28b).
    write_ldt_descriptor(1, cs_base, 0xFFFF, 0xFA, false);
    ldt_set(1, true);
    // Slot 2: client data  (DPL=3, data r/w) -- 0xF2 access
    // Data selectors match the AX=1 bit (32-bit if 32-bit PM).
    write_ldt_descriptor(2, ds_base, 0xFFFF, 0xF2, true);
    ldt_set(2, true);
    // Slot 3: client stack (DPL=3, data r/w, 32-bit for big stack)
    write_ldt_descriptor(3, ss_base, 0xFFFF, 0xF2, true);
    ldt_set(3, true);
    // Slot 4: PSP selector (DPL=3, data r/w, 16-bit).
    // DPMI 0.9 spec says LDT[4] aliases client's ES at DPMI-entry
    // time, but DJGPP libc's _setup_environment uses stubinfo.
    // psp_selector (= LDT[4] alias of ES at entry) expecting it to
    // alias the actual PSP.  go32-v2's env-parsing code changes ES
    // to the env segment before DPMI entry, so the pure-spec behavior
    // would give LDT[4] an env alias, not a PSP alias.
    //
    // We override: LDT[4] always aliases the CURRENT PSP segment.
    // For a top-level program that's PSP_SEG; for a child spawned
    // via AH=4B it's child_psp.  AH=4B publishes this via
    // s_current_psp_seg so dpmi_entry from the child still gets a
    // correct PSP alias (a hardcoded PSP_SEG would alias the parent).
    const uint16_t cur_psp_seg = s_current_psp_seg ? s_current_psp_seg : PSP_SEG;
    write_ldt_descriptor(4, cur_psp_seg * 16u, 0xFFFF, 0xF2);
    ldt_set(4, true);
    // Also replace PSP[0x2C] (RM env segment) with a PM selector
    // aliasing the env block, since DJGPP treats that word as a
    // selector (after movedata'ing it out of PSP).  Allocate LDT[5]
    // for the env alias.  The env seg for a child is cur_env_seg.
    const uint16_t cur_env_seg = s_current_env_seg ? s_current_env_seg : ENV_SEG;
    write_ldt_descriptor(5, cur_env_seg * 16u, 0xFFFF, 0xF2);
    ldt_set(5, true);
    const uint16_t ldt_cs = (1 << 3) | 0x04 | 0x03;   // 0x0F
    const uint16_t ldt_ds = (2 << 3) | 0x04 | 0x03;   // 0x17
    const uint16_t ldt_ss = (3 << 3) | 0x04 | 0x03;   // 0x1F
    const uint16_t ldt_es = (4 << 3) | 0x04 | 0x03;   // 0x27 (PSP alias)
    const uint16_t ldt_env = (5 << 3) | 0x04 | 0x03;  // 0x2F (env alias)
    mem_writew(cur_psp_seg * 16u + 0x2C, ldt_env);

    // Stage IRETD frame on ring-0 scratch stack.
    CPU_SetSegGeneral(ss, PM_CB_STACK);
    reg_esp = PM_CB_STACK_SIZE - 32;
    CPU_SetSegGeneral(ds, ldt_ds);
    CPU_SetSegGeneral(es, ldt_es);

    const PhysPt frame = SegPhys(ss) + reg_esp;
    mem_writed(frame + 0,  client_ip);
    mem_writed(frame + 4,  ldt_cs);
    // EFLAGS with IOPL=3 + reserved bit 1.  DJGPP and other DPMI
    // clients sprinkle CLI/STI throughout their libc (e.g. exceptn.S
    // line 1ACC, disabling interrupts before restoring FS/GS).  At
    // ring-3 with IOPL<3 those trigger #GP; CWSDPMI avoids this by
    // running clients with IOPL=3 so the opcodes are legal but
    // (because paging/segmentation still isolates) have no effect on
    // real interrupt delivery.  Mimic that here: EFLAGS = 0x3002.
    mem_writed(frame + 8,  0x00003002u);
    // Preserve the client's RM SP into PM instead of hardcoding
    // 0xFFFC.  For a nested DJGPP child, SS base = child_ds_base
    // (e.g. 0x20110) and a hardcoded 0xFFFC lands the PM stack at
    // 0x3010C -- which sits INSIDE the 60KB scratch buffer the
    // child allocates (base 0x24120) for its COFF-load AH=3F read.
    // The read clobbers the stack and RET on return gets 0.
    // Using client_sp keeps the stack just past the stub image
    // where the MZ loader placed it.
    const uint32_t client_sp = reg_sp;
    mem_writed(frame + 12, client_sp);
    mem_writed(frame + 16, ldt_ss);

    CPU_IRET(true, 0);
    reg_ax = 0;
    return CBRET_NONE;
  }

  // Replace the stacked return-address CS with our PM code selector so the
  // stub's retf lands in PM rather than at an invalid real-mode segment.
  const PhysPt stacked_cs = SegValue(ss) * 16u + reg_sp + 2;
  mem_writeb(stacked_cs,     PM_CS_SEL & 0xFF);
  mem_writeb(stacked_cs + 1, (PM_CS_SEL >> 8) & 0xFF);

  // Flip CR0.PE -- CPU is now in protected mode.  dosbox's CPU core
  // respects the CR0 write.
  CPU_SET_CRX(0, 0x00000001);      // PE=1, all other bits off

  // LLDT is #UD in real mode; issue it now that PE=1 is set.  Points
  // LDTR at GDT[6] = our LDT.  Subsequent INT 31h AX=0000/0001/0002
  // allocate/free/convert descriptors inside this LDT.
  CPU_LLDT(PM_LDT_SEL);

  // Load PM selectors into DS/SS/ES (CS gets set by the retf).
  CPU_SetSegGeneral(ds, PM_DS_SEL);
  CPU_SetSegGeneral(ss, PM_SS_SEL);
  CPU_SetSegGeneral(es, PM_ES_SEL);

  reg_ax = 0;                       // success
  set_cf(false);
  return CBRET_NONE;
}

// XMS driver entry point (callable as a far call by clients that
// obtained the address via INT 2F AX=4310h).  Dispatches on AH:
//   00h  Get XMS version     -> AX=0x0300 (XMS 3.0), BX=driver ver,
//                              DX=0 (no HMA).
//   08h  Query free XMS      -> AX=largest-free-KB, DX=total-KB
//   09h  Allocate EMB        -> AX=1 (OK), DX=handle (we hand back
//                              a pm_alloc block)
//   0Ah  Free EMB            -> AX=1, frees the block
//   0Bh  Move EMB            -> AX=1, copies bytes
//   0Ch  Lock EMB            -> AX=1, DX:BX=linear address
//   0Dh  Unlock EMB          -> AX=1
//   0Eh  Get EMB info        -> AX=1, BH=#handles, DX=size
// Everything else: AX=0 (error), BL=80h (function not implemented).
// Enough for DOS/4GW's init to succeed -- it uses XMS for its
// transfer buffer and for swap.  We map XMS handles directly to
// pm_arena linear addresses.
std::map<uint16_t, uint32_t> s_xms_handles;   // handle -> linear base
std::map<uint16_t, uint32_t> s_xms_sizes;     // handle -> size (KB)
uint16_t s_xms_next_handle = 1;
bool     s_hma_allocated = false;  // HMA (FFFF:0010..FFFF:FFFF) claimed

Bitu dosemu_xms_driver() {
  const uint8_t fn = reg_ah;
  switch (fn) {
    case 0x00:
      reg_ax = 0x0300;   // XMS 3.0
      reg_bx = 0;
      reg_dx = 1;        // bit 0 = HMA exists (we keep A20 on permanently)
      return CBRET_NONE;

    // --- HMA + A20 control ---
    //
    // HMA is the "high memory area" -- the 64 KB-16 B window from
    // FFFF:0010 to FFFF:FFFF that becomes addressable once A20 is
    // enabled.  We force A20 on at startup (bridge.cc:1031) and never
    // turn it off, so HMA is always reachable; only arbitrate which
    // single client owns it.

    case 0x01: {  // Request HMA (DX = minimum bytes needed, 0xFFFF = DOS)
      if (s_hma_allocated) {
        reg_ax = 0; reg_bl = 0x92;  // HMA already in use
        return CBRET_NONE;
      }
      s_hma_allocated = true;
      reg_ax = 1;
      return CBRET_NONE;
    }
    case 0x02: {  // Release HMA
      if (!s_hma_allocated) {
        reg_ax = 0; reg_bl = 0x93;  // HMA not allocated
        return CBRET_NONE;
      }
      s_hma_allocated = false;
      reg_ax = 1;
      return CBRET_NONE;
    }
    case 0x03:   // Global Enable A20
    case 0x05:   // Local Enable A20
      MEM_A20_Enable(true);
      reg_ax = 1;
      return CBRET_NONE;
    case 0x04:   // Global Disable A20  (no-op; we need A20 on for LDT/IDT)
    case 0x06:   // Local Disable A20
      reg_ax = 1;
      return CBRET_NONE;
    case 0x07:   // Query A20 state
      reg_ax = MEM_A20_Enabled() ? 1 : 0;
      reg_bl = 0;
      return CBRET_NONE;
    case 0x08: {
      // Free extended memory in KB.  pm_arena covers [1MB, memsize).
      pm_init();
      const uint32_t total_kb = (s_pm_end - PM_ARENA_START) / 1024u;
      reg_ax = total_kb & 0xFFFF;
      reg_dx = total_kb & 0xFFFF;
      reg_bl = 0;
      return CBRET_NONE;
    }
    case 0x09: {
      // Allocate EMB: DX = size in KB.
      const uint32_t bytes = static_cast<uint32_t>(reg_dx) * 1024u;
      const uint32_t base  = pm_alloc(bytes ? bytes : 4096u);
      if (base == 0) {
        reg_ax = 0; reg_bl = 0xA0;   // all extended memory allocated
        return CBRET_NONE;
      }
      const uint16_t h = s_xms_next_handle++;
      s_xms_handles[h] = base;
      s_xms_sizes[h]   = reg_dx;
      reg_ax = 1;
      reg_dx = h;
      return CBRET_NONE;
    }
    case 0x0A: {
      auto it = s_xms_handles.find(reg_dx);
      if (it == s_xms_handles.end()) {
        reg_ax = 0; reg_bl = 0xA2;  // invalid handle
        return CBRET_NONE;
      }
      pm_free(it->second);
      s_xms_handles.erase(it);
      s_xms_sizes.erase(reg_dx);
      reg_ax = 1;
      return CBRET_NONE;
    }
    case 0x0B: {
      // Move memory: DS:SI = EMM struct (length, srch, srcoff,
      // dsth, dstoff).  For pm-backed handles or conventional segs
      // (handle = 0), compute physical addresses and memcpy.
      const PhysPt s = SegPhys(ds) + reg_si;
      const uint32_t len  = mem_readd(s + 0);
      const uint16_t sh   = mem_readw(s + 4);
      const uint32_t soff = mem_readd(s + 6);
      const uint16_t dh   = mem_readw(s + 10);
      const uint32_t doff = mem_readd(s + 12);
      // For handle=0 ("source/dest is real-mode memory"), XMS spec
      // packs the 32-bit offset field as a seg:off far pointer
      // (low word = offset, high word = segment).  Decoding it as
      // a flat linear address would aim writes at MB-scale garbage
      // addresses.  FreeCOM's XMS_Swap build depends on this.
      auto rm_far_to_linear = [](uint32_t packed) -> uint32_t {
        const uint32_t seg = (packed >> 16) & 0xFFFF;
        const uint32_t off = packed & 0xFFFF;
        return seg * 16u + off;
      };
      uint32_t slin, dlin;
      if (sh == 0) slin = rm_far_to_linear(soff);
      else {
        auto it = s_xms_handles.find(sh);
        if (it == s_xms_handles.end()) { reg_ax = 0; reg_bl = 0xA3; return CBRET_NONE; }
        slin = it->second + soff;
      }
      if (dh == 0) dlin = rm_far_to_linear(doff);
      else {
        auto it = s_xms_handles.find(dh);
        if (it == s_xms_handles.end()) { reg_ax = 0; reg_bl = 0xA5; return CBRET_NONE; }
        dlin = it->second + doff;
      }
      for (uint32_t i = 0; i < len; ++i)
        mem_writeb(dlin + i, mem_readb(slin + i));
      reg_ax = 1;
      return CBRET_NONE;
    }
    case 0x0C: {
      // Lock EMB: DX = handle.  Return DX:BX = linear.
      auto it = s_xms_handles.find(reg_dx);
      if (it == s_xms_handles.end()) {
        reg_ax = 0; reg_bl = 0xA2; return CBRET_NONE;
      }
      reg_dx = (it->second >> 16) & 0xFFFF;
      reg_bx = it->second & 0xFFFF;
      reg_ax = 1;
      return CBRET_NONE;
    }
    case 0x0D:
      reg_ax = 1;   // Unlock: no-op (we don't page)
      return CBRET_NONE;
    case 0x0E: {
      auto it = s_xms_sizes.find(reg_dx);
      if (it == s_xms_sizes.end()) {
        reg_ax = 0; reg_bl = 0xA2; return CBRET_NONE;
      }
      reg_bh = 1;       // 1 lock count (pretend)
      reg_bl = 0;       // 0 free handles (don't care)
      reg_dx = it->second & 0xFFFF;
      reg_ax = 1;
      return CBRET_NONE;
    }
    default:
      reg_ax = 0;
      reg_bl = 0x80;    // function not implemented
      return CBRET_NONE;
  }
}

uint16_t s_xms_driver_seg = 0;
uint16_t s_xms_driver_off = 0;

Bitu dosemu_int2f() {
  // INT 2F AX=1600..160A: Windows enhanced-mode detection.
  // AL=0 means "Windows not running".  Without an explicit response,
  // DOS/4GW's runtime sees whatever AL was on entry and misbehaves.
  // Narrow range so 0x16xx unrelated calls (esp. 0x1687 DPMI probe)
  // don't fall into this bucket.
  if (reg_ax >= 0x1600 && reg_ax <= 0x160A) {
    reg_ax = 0;
    return CBRET_NONE;
  }
  if (reg_ax == 0x4300) {
    // XMS installation check: AL=80h = installed.
    reg_al = 0x80;
    return CBRET_NONE;
  }
  if (reg_ax == 0x4310) {
    // Get XMS driver entry point -> ES:BX
    SegSet16(es, s_xms_driver_seg);
    reg_bx = s_xms_driver_off;
    return CBRET_NONE;
  }
  if (reg_ax == 0x1687) {
    // Some DOS extenders (DOS/4GW, DOS/16M, PMODE/W) bring their own
    // protected-mode machinery and do their own PM init when no host
    // DPMI is present.  Auto-detected at load time via the MZ stub's
    // DOS/4G signature (see s_extender_bound).  Matches how QEMM-era
    // memory managers coexisted with bound extenders.
    if ((s_extender_bound && !dosemu::g_debug.force_dpmi)
        || dosemu::g_debug.no_dpmi) {
      reg_ax = 0xFFFF;        // DPMI not present
      return CBRET_NONE;
    }
    // DPMI detection response:
    //   AX = 0    (DPMI host present)
    //   BX = 1    (flags: bit 0 = 32-bit DPMI available)
    //   CL = 3    (CPU type: 386)
    //   DH:DL = 0:90h  (DPMI 0.90 -- what DJGPP expects)
    //   SI = 0    (paragraphs of private data required by host)
    //   ES:DI = real-mode entry-point for the switch.
    //
    // SI=0 is important for DJGPP go32-v2: when SI>0 the stub does
    // AH=48 (alloc) and sets ES = allocated block before the DPMI
    // mode-switch far-call, making stubinfo.psp_selector alias that
    // allocated block (not the PSP).  DJGPP libc's _setup_environment
    // reads PSP[0x2C] via psp_selector and fails.  With SI=0 the stub
    // leaves ES alone, so ES at mode-switch = PSP_SEG (set by DOS at
    // program load), and psp_selector correctly aliases the PSP.
    reg_ax = 0;
    reg_bx = 1;
    reg_cl = 3;
    reg_dh = 0;
    reg_dl = 0x5A;      // 0x5A = 90
    reg_si = 0;
    SegSet16(es, s_dpmi_entry_seg);
    reg_di = s_dpmi_entry_off;
    return CBRET_NONE;
    // NOTE: after this fix, PSP[0x2C] still holds a raw RM segment, not
    // a PM selector.  DJGPP libc's _setup_environment treats the 2
    // bytes at PSP[0x2C] as a DPMI selector.  See pm_rewrite_psp_env()
    // which we call at DPMI entry to convert ENV_SEG into a selector.
  }
  // Leave AX unchanged for sub-functions we don't handle; IRET returns.
  return CBRET_NONE;
}

// INT 16h (BIOS keyboard services).  Programs like deltree/debug read user
// input this way rather than via INT 21h.  We plumb host stdin to AH=00
// (wait for key) and AH=01 (check for key).  A single-byte peek buffer
// gives AH=01 the "ready without consuming" semantic the BIOS ABI expects.
int s_int16_peek = -1;

Bitu dosemu_int16() {
  switch (reg_ah) {
    case 0x00:
    case 0x10: {          // wait for key; AH=scancode, AL=ASCII
      uint8_t c = 0;
      if (s_int16_peek >= 0) {
        c = static_cast<uint8_t>(s_int16_peek);
        s_int16_peek = -1;
      } else {
        const ssize_t n = ::read(STDIN_FILENO, &c, 1);
        if (n <= 0) c = 0x1B;  // EOF -> ESC so interactive progs give up
      }
      if (c == '\n') c = '\r';
      reg_al = c;
      reg_ah = (c >= 'a' && c <= 'z') ? (c - 'a' + 0x1E)
             : (c >= 'A' && c <= 'Z') ? (c - 'A' + 0x1E)
             : (c == 0x0D)          ? 0x1C
             : (c == 0x1B)          ? 0x01
             : 0x01;
      return CBRET_NONE;
    }
    case 0x01:
    case 0x11: {          // check key: ZF=1 if none, else ZF=0 + AX=key
      if (s_int16_peek < 0) {
        fd_set rd; FD_ZERO(&rd); FD_SET(STDIN_FILENO, &rd);
        struct timeval zero = {0, 0};
        if (::select(STDIN_FILENO + 1, &rd, nullptr, nullptr, &zero) > 0) {
          uint8_t c;
          if (::read(STDIN_FILENO, &c, 1) == 1) {
            if (c == '\n') c = '\r';
            s_int16_peek = c;
          }
        }
      }
      if (s_int16_peek < 0) {
        CALLBACK_SZF(true);
        return CBRET_NONE;
      }
      const uint8_t c = static_cast<uint8_t>(s_int16_peek);
      reg_al = c;
      reg_ah = (c >= 'a' && c <= 'z') ? (c - 'a' + 0x1E)
             : (c >= 'A' && c <= 'Z') ? (c - 'A' + 0x1E)
             : 0x01;
      CALLBACK_SZF(false);
      return CBRET_NONE;
    }
    case 0x02:            // get shift state: report no modifiers
      reg_al = 0;
      return CBRET_NONE;
    default:
      return CBRET_NONE;
  }
}

// AX=0302 pushes an IRET frame pointing at this stop callback before
// transferring control to the client's real-mode procedure.  When the
// procedure IRETs it pops IP:CS:FLAGS back onto our stub; the FE 38
// native dispatch below returns CBRET_STOP which unwinds the nested
// DOSBOX_RunMachine, handing control back to the AX=0302 handler.
Bitu dosemu_rm_stop() { return CBRET_STOP; }

// LE exception handler -- per-vector dispatch.  Installed as PM IDT
// gate targets for vectors 0x00..0x1F on the LE launch path.  We
// can't recover from a client PM fault (no virtual-memory fault-in,
// no client-provided handler like DPMI AX=0203), so we log the
// fault frame and terminate cleanly instead of letting dosbox abort
// at the "Gate Selector points to illegal descriptor" cascade.
//
// The frame layout differs by vector:
//   Error-code vectors (#DF, #TS, #NP, #SS, #GP, #PF, #AC) push a
//   4-byte error code before [EIP, CS, EFLAGS].
//   The rest push only [EIP, CS, EFLAGS].
bool le_exc_has_error_code(int vec) {
  // Intel SDM vol 3 table 6-1.  #AC (17) also pushes an error code
  // but is rarely used in pre-emptive contexts; included for
  // completeness.
  switch (vec) {
    case 8: case 10: case 11: case 12: case 13: case 14: case 17:
      return true;
    default:
      return false;
  }
}

const char *le_exc_name(int vec) {
  switch (vec) {
    case 0:  return "#DE  divide-by-zero";
    case 1:  return "#DB  debug";
    case 2:  return "NMI";
    case 3:  return "#BP  breakpoint";
    case 4:  return "#OF  overflow";
    case 5:  return "#BR  bound-range";
    case 6:  return "#UD  invalid opcode";
    case 7:  return "#NM  device-not-available";
    case 8:  return "#DF  double-fault";
    case 10: return "#TS  invalid TSS";
    case 11: return "#NP  segment-not-present";
    case 12: return "#SS  stack-segment fault";
    case 13: return "#GP  general-protection";
    case 14: return "#PF  page-fault";
    case 16: return "#MF  x87 FPU";
    case 17: return "#AC  alignment-check";
    case 18: return "#MC  machine-check";
    case 19: return "#XM  SIMD FPU";
    default: return "reserved/unknown";
  }
}

// Parse and log a 32-bit-gate exception frame (used when the LE
// client entered PM with a 32-bit CS).  Frame layout: [err] EIP CS
// EFLAGS, all dwords.  CS's low word is the faulting selector.
void dosemu_le_exc_dump32(int vec) {
  const PhysPt sp = SegPhys(ss) + reg_esp;
  const bool has_err = le_exc_has_error_code(vec);
  const uint32_t err = has_err ? mem_readd(sp) : 0;
  const uint32_t off = has_err ? 4 : 0;
  const uint32_t fault_eip = mem_readd(sp + off);
  const uint16_t fault_cs  = mem_readw(sp + off + 4);
  const uint32_t fault_efl = mem_readd(sp + off + 8);
  std::fprintf(stderr,
      "dosemu: LE client exception 0x%02x (%s) -- terminating.\n"
      "  fault at CS:EIP = %04x:%08x  EFLAGS = %08x\n",
      vec, le_exc_name(vec), fault_cs, fault_eip, fault_efl);
  if (has_err)
    std::fprintf(stderr, "  error code = 0x%08x\n", err);
}

// Parse and log a 16-bit-gate exception frame.  Frame layout:
// [err] IP CS FLAGS, all words.
void dosemu_le_exc_dump16(int vec) {
  const PhysPt sp = SegPhys(ss) + reg_esp;
  const bool has_err = le_exc_has_error_code(vec);
  const uint16_t err = has_err ? mem_readw(sp) : 0;
  const uint32_t off = has_err ? 2 : 0;
  const uint16_t fault_ip  = mem_readw(sp + off);
  const uint16_t fault_cs  = mem_readw(sp + off + 2);
  const uint16_t fault_efl = mem_readw(sp + off + 4);
  std::fprintf(stderr,
      "dosemu: LE client exception 0x%02x (%s) -- terminating.\n"
      "  fault at CS:IP = %04x:%04x  FLAGS = %04x\n",
      vec, le_exc_name(vec), fault_cs, fault_ip, fault_efl);
  if (has_err)
    std::fprintf(stderr, "  error code = 0x%04x\n", err);
}

// vec < 0 means "unknown -- shared callback, don't have per-vector
// info".  Prints "unknown" in the log but still dumps the frame,
// which identifies the fault CS:EIP (and error code if this turns
// out to be an error-code vector -- we print both interpretations).
Bitu dosemu_le_exc_handle32(int vec) {
  if (vec < 0) {
    // Without vector info, dump the top 5 dwords so either
    // interpretation (with or without error code) is readable.
    const PhysPt sp = SegPhys(ss) + reg_esp;
    std::fprintf(stderr,
        "dosemu: LE client PM exception (vector unknown) -- terminating.\n"
        "  SS:ESP = %04x:%08x  stack = %08x %08x %08x %08x %08x\n"
        "  (no-error-code frame = EIP CS EFLAGS; error-code frame = ERR EIP CS EFLAGS)\n",
        SegValue(ss), reg_esp,
        mem_readd(sp), mem_readd(sp + 4), mem_readd(sp + 8),
        mem_readd(sp + 12), mem_readd(sp + 16));
  } else {
    dosemu_le_exc_dump32(vec);
  }
  std::fprintf(stderr,
      "  EAX=%08x EBX=%08x ECX=%08x EDX=%08x\n"
      "  ESI=%08x EDI=%08x EBP=%08x\n"
      "  DS=%04x ES=%04x FS=%04x GS=%04x\n",
      reg_eax, reg_ebx, reg_ecx, reg_edx,
      reg_esi, reg_edi, reg_ebp,
      SegValue(ds), SegValue(es), SegValue(fs), SegValue(gs));
  s_exit_code = 1;
  shutdown_requested = true;
  return CBRET_STOP;
}

Bitu dosemu_le_exc_handle16(int vec) {
  if (vec < 0) {
    const PhysPt sp = SegPhys(ss) + reg_esp;
    std::fprintf(stderr,
        "dosemu: LE client PM exception (vector unknown) -- terminating.\n"
        "  SS:SP = %04x:%04x  stack = %04x %04x %04x %04x %04x %04x %04x %04x\n"
        "  (no-error-code frame = IP CS FLAGS; error-code frame = ERR IP CS FLAGS)\n",
        SegValue(ss), reg_sp,
        mem_readw(sp), mem_readw(sp + 2), mem_readw(sp + 4), mem_readw(sp + 6),
        mem_readw(sp + 8), mem_readw(sp + 10), mem_readw(sp + 12), mem_readw(sp + 14));
  } else {
    dosemu_le_exc_dump16(vec);
  }
  std::fprintf(stderr,
      "  AX=%04x BX=%04x CX=%04x DX=%04x "
      "SI=%04x DI=%04x BP=%04x\n"
      "  DS=%04x ES=%04x FS=%04x GS=%04x\n",
      reg_ax, reg_bx, reg_cx, reg_dx,
      reg_si, reg_di, reg_bp,
      SegValue(ds), SegValue(es), SegValue(fs), SegValue(gs));
  s_exit_code = 1;
  shutdown_requested = true;
  return CBRET_STOP;
}

// One-size-fits-all dosbox callbacks, invoked by all 32 vectors of
// the matching gate bitness.  Vector is unknown at handler entry so
// we log "unknown vector" plus the stacked frame; root-causing any
// specific exception still needs the fault CS:EIP which we do have.
Bitu dosemu_le_exc_any32() { return dosemu_le_exc_handle32(-1); }
Bitu dosemu_le_exc_any16() { return dosemu_le_exc_handle16(-1); }
RealPt s_rm_stop_ptr = 0;

// No-op callback handed back by AX=0305/0306 as state-save and raw-
// mode-switch routine addresses.  Real hosts provide routines that
// actually save/restore or switch modes; we hand back legal RETF
// stubs so clients that merely *probe* and discard the addresses
// get a success answer.  Clients that actually try to call them
// and expect side effects will not see any, but the primary use
// (init-time probe + fall back to INT 2Fh/1687h) works.
Bitu dosemu_noop_retf() { return CBRET_NONE; }
RealPt s_noop_retf_ptr = 0;

// PM exception handler table forward-declared here; the trampoline
// below needs to read it.  Populated by AX=0203; defaulted to all-zero
// (selector=0 means "no user handler; fall back to terminate").
struct ExcHandler { uint16_t sel; uint32_t off; };
extern ExcHandler s_pm_exc[32];

// --- DPMI AX=0203 exception dispatch (CWSDPMI-compatible frame) ------
//
// DJGPP and other DPMI clients install PM exception handlers via
// AX=0203 and assume CWSDPMI's specific stack layout when the handler
// is entered: a 32-byte frame of
//   [SP+0]  user_exception_return_EIP
//   [SP+4]  user_exception_return_CS
//   [SP+8]  err code
//   [SP+12] EIP (of faulting insn)
//   [SP+16] CS
//   [SP+20] EFLAGS
//   [SP+24] outer ESP
//   [SP+28] outer SS
// The +0/+4 slot (user_exception_return) is what the handler LRETs
// through to unwind; the +8..+28 part is the CPU-pushed state the
// handler can inspect/modify before returning.
// A naive "install IDT gate directly at the user handler" dispatch only
// delivers the CPU-pushed 12/16-byte frame -- DJGPP reads garbage at
// [ebp+4]/[ebp+8] and eventually LRETs to it.
//
// Our implementation: one per-vector trampoline callback that (a) reads
// the CPU-pushed frame, (b) constructs the CWSDPMI-style 32-byte frame
// + 12-byte IRETD frame on the client's stack, (c) lets the callback
// stub's IRETD pop the IRETD frame and jump to the user handler with
// the CWSDPMI frame sitting at [SP+0] beneath.  When the user handler
// finishes and does LRET, control lands at user_exception_return
// (another trampoline) which restores the outer SS:ESP/CS:EIP/EFLAGS
// and IRETDs back to the faulting instruction (potentially modified).
bool pm_exc_has_err_code(int vec) {
  return vec == 8 || (vec >= 10 && vec <= 14) || vec == 17;
}

Bitu dosemu_pm_exc_dispatch(int vec) {
  // Runs at ring-0 via the IDT gate (PM_CB_SEL has DPL=0).  The
  // ring-3 client was at cpl=3 when the exception fired, so CPU did a
  // ring-change dispatch: ring-0 SS:ESP is our PM_CB_STACK scratch,
  // and the pushed frame is 20 bytes (plus 4 for err-code excs):
  //   [ring0_ESP + 0]  err   (if has_err)
  //   [ring0_ESP + 4]  EIP     (or +0 if no err)
  //   [ring0_ESP + 8]  CS      (or +4)
  //   [ring0_ESP +12]  EFLAGS  (or +8)
  //   [ring0_ESP +16]  outer ESP   (or +12)
  //   [ring0_ESP +20]  outer SS    (or +16)
  const bool has_err = pm_exc_has_err_code(vec);
  const uint32_t r0_esp = reg_esp;          // ring-0 ESP at callback entry
  const PhysPt r0_fp = SegPhys(ss) + r0_esp;

  uint32_t err, eip, cs_val, eflags, outer_esp;
  uint16_t outer_ss;
  if (has_err) {
    err       = mem_readd(r0_fp + 0);
    eip       = mem_readd(r0_fp + 4);
    cs_val    = mem_readd(r0_fp + 8);
    eflags    = mem_readd(r0_fp + 12);
    outer_esp = mem_readd(r0_fp + 16);
    outer_ss  = static_cast<uint16_t>(mem_readd(r0_fp + 20) & 0xFFFF);
  } else {
    err       = 0;
    eip       = mem_readd(r0_fp + 0);
    cs_val    = mem_readd(r0_fp + 4);
    eflags    = mem_readd(r0_fp + 8);
    outer_esp = mem_readd(r0_fp + 12);
    outer_ss  = static_cast<uint16_t>(mem_readd(r0_fp + 16) & 0xFFFF);
  }

  if (dosemu::g_debug.exc_trace) {
    static int seq = 0;
    std::fprintf(stderr,
        "[exc#%d] vec=%d cs:eip=%04x:%08x err=0x%x outer_ss:esp=%04x:%08x eflags=%08x cpl=%u\n",
        seq++, vec, (unsigned)cs_val, eip, err,
        (unsigned)outer_ss, outer_esp, eflags, (unsigned)cpu.cpl);
    // Raw ring-0 stack dump.
    std::fprintf(stderr, "[exc#] r0_fp=%08x r0_esp=%08x ring0 at r0_fp..+28:",
                 (unsigned)r0_fp, (unsigned)r0_esp);
    for (int i = 0; i <= 28; i += 4)
      std::fprintf(stderr, " %08x", (unsigned)mem_readd(r0_fp + i));
    std::fprintf(stderr, "\n");
    std::fprintf(stderr, "[exc#] CPU Segs.val[cs]=%04x phys=%08x cpl=%u\n",
                 (unsigned)SegValue(cs), (unsigned)SegPhys(cs), (unsigned)cpu.cpl);
    // Dump the faulting instruction bytes.  cs_val may be a PM selector,
    // so resolve via the descriptor table (routes GDT vs LDT by TI bit).
    Descriptor dcs_dump;
    if (cpu.gdt.GetDescriptor(cs_val, dcs_dump)) {
      const uint32_t cs_base = dcs_dump.GetBase();
      std::fprintf(stderr, "[exc#] cs.base=%08x bytes@eip:", cs_base);
      for (int i = 0; i < 16; ++i)
        std::fprintf(stderr, " %02x", mem_readb(cs_base + eip + i));
      std::fprintf(stderr, "\n");
    }
    // Dump outer SS area too
    Descriptor dss_dump;
    if (cpu.gdt.GetDescriptor(outer_ss, dss_dump)) {
      const uint32_t ss_base = dss_dump.GetBase();
      std::fprintf(stderr, "[exc#] ss.base=%08x ss.limit=%08x words@esp-8:",
                   ss_base, (unsigned)dss_dump.GetLimit());
      for (int i = 0; i < 8; ++i) {
        std::fprintf(stderr, " %04x",
            (unsigned)mem_readw(ss_base + (outer_esp - 8 + i*2)));
      }
      std::fprintf(stderr, "\n");
    }
  }

  // Recursion guard.  If the user handler's exit cleanup re-faults
  // on the same vector at the same (or nearby) EIP, we're in an
  // infinite-dispatch loop that'd never terminate otherwise
  // (DJGPP's default SIGSEGV handler reports + _exit()s, but if
  // _exit itself #GPs, we end up right back here).  CWSDPMI has
  // an equivalent check via locked_count > 5 in EXPHDLR.C.
  static int recursive_fault_count = 0;
  static uint32_t last_fault_eip = 0;
  static uint16_t last_fault_cs  = 0;
  if (cs_val == last_fault_cs && eip == last_fault_eip) {
    if (++recursive_fault_count > 4) {
      std::fprintf(stderr,
          "dosemu: PM exception dispatcher in recursive-fault loop "
          "(vec=%d cs:eip=%04x:%08x err=0x%x) -- terminating\n",
          vec, (unsigned)cs_val, (unsigned)eip, (unsigned)err);
      return CBRET_STOP;
    }
  } else {
    recursive_fault_count = 0;
    last_fault_eip = eip;
    last_fault_cs  = cs_val;
  }

  const uint16_t user_sel = s_pm_exc[vec].sel;
  const uint32_t user_off = s_pm_exc[vec].off;

  if (user_sel == 0) {
    std::fprintf(stderr,
        "dosemu: PM exception vec=%d at %04x:%08x err=0x%x "
        "(no user handler installed, terminating)\n",
        vec, (unsigned)cs_val, eip, err);
    return CBRET_STOP;
  }

  // Build the CWSDPMI exception frame on our private exception stack
  // (PM_EXC_STACK_SEL, ring-3 alias of PM_CB_STACK_BASE), NOT on the
  // client's outer stack: the client's SS:ESP may be corrupt at fault
  // time (e.g. an IRET that popped garbage SS=0 is exactly the failure
  // mode that triggers this path in the first place).  The frame goes
  // at the top of our private stack so the handler's own pushes grow
  // downward without clobbering the frame.
  const uint16_t handler_ss = PM_EXC_STACK_SEL;
  const uint32_t handler_esp_base = PM_CB_STACK_SIZE - 32u;   // top-of-stack
  const uint32_t client_new_esp   = handler_esp_base;
  {
    // PM_CB_STACK_BASE is the linear base; write frame directly at the
    // linear address (saves a descriptor lookup).
    const PhysPt sbase = PM_CB_STACK_BASE;
    mem_writed(sbase + client_new_esp + 0,  s_pm_exc_ret_off);
    mem_writed(sbase + client_new_esp + 4,  PM_CB3_SEL);    // DPL=3 CS
    mem_writed(sbase + client_new_esp + 8,  err);
    mem_writed(sbase + client_new_esp + 12, eip);
    mem_writed(sbase + client_new_esp + 16, cs_val);
    mem_writed(sbase + client_new_esp + 20, eflags);
    mem_writed(sbase + client_new_esp + 24, outer_esp);
    mem_writed(sbase + client_new_esp + 28, outer_ss);
  }

  // Rewrite the ring-change IRETD frame on our ring-0 stack so the
  // stub's 66 CF IRETDs into the user handler (ring-3), with the
  // client's new SS:ESP pointing at the CWSDPMI frame above.  For
  // err-code exceptions, the CPU placed the err code at r0_fp+0;
  // IRETD doesn't pop err code, so we skip it by writing the IRETD
  // frame at r0_fp+4 and advancing reg_esp by 4.  For non-err-code
  // exceptions the IRETD frame starts at r0_fp directly.
  const uint32_t iret_off = has_err ? 4u : 0u;
  const uint16_t user_cs_rpl3 = (user_sel & 0xFFFC) | 3;
  mem_writed(r0_fp + iret_off + 0,  user_off);
  mem_writed(r0_fp + iret_off + 4,  user_cs_rpl3);
  mem_writed(r0_fp + iret_off + 8,  eflags & ~0x200u);   // IF=0
  mem_writed(r0_fp + iret_off + 12, client_new_esp);
  mem_writed(r0_fp + iret_off + 16, handler_ss);
  if (has_err) reg_esp = r0_esp + 4u;

  return CBRET_NONE;
}

Bitu dosemu_pm_exc_ret() {
  // User handler has just done LRET after processing the exception.
  // Stack at entry (after the handler's LRET popped user_ret_EIP/CS):
  //   [SP+0]  err         (discard)
  //   [SP+4]  EIP         (possibly modified to resume elsewhere)
  //   [SP+8]  CS
  //   [SP+12] EFLAGS
  //   [SP+16] outer_ESP
  //   [SP+20] outer_SS
  // Restore SS:ESP to outer, push (EIP/CS/EFLAGS) on the outer stack
  // for the stub's IRETD, which will then jump back to the faulting
  // (or handler-chosen-resume) CS:EIP.
  const PhysPt ss_base = SegPhys(ss);
  const uint32_t esp = cpu.stack.big ? reg_esp : reg_sp;
  const PhysPt fp = ss_base + esp;

  const uint32_t eip    = mem_readd(fp + 4);
  const uint32_t cs_val = mem_readd(fp + 8);
  const uint32_t eflags = mem_readd(fp + 12);
  const uint32_t oesp   = mem_readd(fp + 16);
  const uint16_t oss    = static_cast<uint16_t>(mem_readd(fp + 20) & 0xFFFF);

  // Switch to the client's outer stack and lay down a 12-byte IRETD
  // frame for the stub to pop.
  CPU_SetSegGeneral(ss, oss);
  uint32_t new_esp = oesp - 12u;
  mem_writed(SegPhys(ss) + new_esp + 0, eip);
  mem_writed(SegPhys(ss) + new_esp + 4, cs_val);
  mem_writed(SegPhys(ss) + new_esp + 8, eflags);
  if (cpu.stack.big) reg_esp = new_esp;
  else               reg_sp  = new_esp & 0xFFFF;

  return CBRET_NONE;
}

// Per-vector trampolines so each exception vector has its own callback
// entry the IDT gate can point at.  Each knows its vector statically
// and just delegates to dosemu_pm_exc_dispatch(N).
#define PMEXC_TRAMP(n) \
  Bitu dosemu_pm_exc_##n() { return dosemu_pm_exc_dispatch(n); }
PMEXC_TRAMP(0)  PMEXC_TRAMP(1)  PMEXC_TRAMP(2)  PMEXC_TRAMP(3)
PMEXC_TRAMP(4)  PMEXC_TRAMP(5)  PMEXC_TRAMP(6)  PMEXC_TRAMP(7)
PMEXC_TRAMP(8)  PMEXC_TRAMP(9)  PMEXC_TRAMP(10) PMEXC_TRAMP(11)
PMEXC_TRAMP(12) PMEXC_TRAMP(13) PMEXC_TRAMP(14) PMEXC_TRAMP(15)
PMEXC_TRAMP(16) PMEXC_TRAMP(17) PMEXC_TRAMP(18) PMEXC_TRAMP(19)
PMEXC_TRAMP(20) PMEXC_TRAMP(21) PMEXC_TRAMP(22) PMEXC_TRAMP(23)
PMEXC_TRAMP(24) PMEXC_TRAMP(25) PMEXC_TRAMP(26) PMEXC_TRAMP(27)
PMEXC_TRAMP(28) PMEXC_TRAMP(29) PMEXC_TRAMP(30) PMEXC_TRAMP(31)
#undef PMEXC_TRAMP

using PmExcFn = Bitu(*)();
static PmExcFn s_pm_exc_tramps[32] = {
  dosemu_pm_exc_0,  dosemu_pm_exc_1,  dosemu_pm_exc_2,  dosemu_pm_exc_3,
  dosemu_pm_exc_4,  dosemu_pm_exc_5,  dosemu_pm_exc_6,  dosemu_pm_exc_7,
  dosemu_pm_exc_8,  dosemu_pm_exc_9,  dosemu_pm_exc_10, dosemu_pm_exc_11,
  dosemu_pm_exc_12, dosemu_pm_exc_13, dosemu_pm_exc_14, dosemu_pm_exc_15,
  dosemu_pm_exc_16, dosemu_pm_exc_17, dosemu_pm_exc_18, dosemu_pm_exc_19,
  dosemu_pm_exc_20, dosemu_pm_exc_21, dosemu_pm_exc_22, dosemu_pm_exc_23,
  dosemu_pm_exc_24, dosemu_pm_exc_25, dosemu_pm_exc_26, dosemu_pm_exc_27,
  dosemu_pm_exc_28, dosemu_pm_exc_29, dosemu_pm_exc_30, dosemu_pm_exc_31,
};

// AX=0303/0304 real-mode callback pool.  Each slot corresponds to a
// CB_RETF callback installed during dosemu_startup.  When RM code
// FAR-CALLs the callback's RM address, its native handler looks up
// the slot, switches to PM, invokes the client's PM procedure with
// the standard RealModeCallStructure populated from RM context, then
// switches back.  32 slots suffice for typical clients (DJGPP
// installs 2-4; CWSDPMI maybe 8).
constexpr int RM_CALLBACK_COUNT = 32;
struct RmCallback {
  bool     allocated   = false;
  uint16_t pm_cs       = 0;
  uint32_t pm_eip      = 0;
  uint16_t struct_sel  = 0;
  uint32_t struct_off  = 0;
  RealPt   rm_addr     = 0;  // captured Get_RealPointer
};
RmCallback s_rm_callbacks[RM_CALLBACK_COUNT];

// Linear address of a PM selector, used to locate the
// RealModeCallStructure from its {sel, off} PM pointer.  The
// selector's descriptor lives in our GDT (TI=0) or LDT (TI=1); read
// the base out of the raw 8 bytes.
PhysPt pm_selector_linear(uint16_t sel, uint32_t off) {
  const uint16_t idx = sel >> 3;
  const PhysPt p = (sel & 0x4) ? (LDT_BASE + idx * 8u)
                               : (GDT_BASE + idx * 8u);
  const uint32_t base = mem_readb(p + 2)
                      | (mem_readb(p + 3) << 8)
                      | (mem_readb(p + 4) << 16)
                      | (mem_readb(p + 7) << 24);
  return static_cast<PhysPt>(base + off);
}

Bitu do_rm_callback(int idx);  // forward

// Macro-generated per-slot trampolines so each callback is an
// individually-addressable function pointer (dosbox's callback table
// dispatches by callback_number, and each HandlerObject gets its own
// number).  Each trampoline knows its slot index statically.
#define RMCB_TRAMP(n) Bitu rmcb_##n() { return do_rm_callback(n); }
RMCB_TRAMP(0)  RMCB_TRAMP(1)  RMCB_TRAMP(2)  RMCB_TRAMP(3)
RMCB_TRAMP(4)  RMCB_TRAMP(5)  RMCB_TRAMP(6)  RMCB_TRAMP(7)
RMCB_TRAMP(8)  RMCB_TRAMP(9)  RMCB_TRAMP(10) RMCB_TRAMP(11)
RMCB_TRAMP(12) RMCB_TRAMP(13) RMCB_TRAMP(14) RMCB_TRAMP(15)
RMCB_TRAMP(16) RMCB_TRAMP(17) RMCB_TRAMP(18) RMCB_TRAMP(19)
RMCB_TRAMP(20) RMCB_TRAMP(21) RMCB_TRAMP(22) RMCB_TRAMP(23)
RMCB_TRAMP(24) RMCB_TRAMP(25) RMCB_TRAMP(26) RMCB_TRAMP(27)
RMCB_TRAMP(28) RMCB_TRAMP(29) RMCB_TRAMP(30) RMCB_TRAMP(31)
#undef RMCB_TRAMP
using RmcbFn = Bitu(*)();
RmcbFn s_rmcb_table[RM_CALLBACK_COUNT] = {
  rmcb_0, rmcb_1, rmcb_2, rmcb_3, rmcb_4, rmcb_5, rmcb_6, rmcb_7,
  rmcb_8, rmcb_9, rmcb_10, rmcb_11, rmcb_12, rmcb_13, rmcb_14, rmcb_15,
  rmcb_16, rmcb_17, rmcb_18, rmcb_19, rmcb_20, rmcb_21, rmcb_22, rmcb_23,
  rmcb_24, rmcb_25, rmcb_26, rmcb_27, rmcb_28, rmcb_29, rmcb_30, rmcb_31,
};

// AX=0303 callback native dispatcher.  We're here because RM code
// FAR-CALLed the RM address of slot `idx`.  CPU is in real mode.
// Sequence:
//   1. Snapshot RM register + segment state.
//   2. Populate the client's RealModeCallStructure (PM pointer stored
//      at 0303-time) with the RM context.
//   3. Swap IDTR to our PM IDT, set CR0.PE=1.
//   4. Load PM CS via manual descriptor cache refresh, DS/ES=struct_sel
//      so client code can access the struct via ES:(E)DI, SS=scratch
//      PM stack selector (GDT[8]).
//   5. Push a 16-bit far-return address pointing at our RM-stop
//      callback (accessed via PM_CB_SEL which covers CB_SEG).
//   6. reg_eip = pm_eip; DOSBOX_RunMachine recursively.
//   7. Client's PM callback RETFs to our stop callback, which returns
//      CBRET_STOP and unwinds the nested machine.
//   8. CR0.PE=0, restore RM IDTR.
//   9. Copy the (possibly-mutated) struct back into reg_* and RM seg
//      values so the RM caller sees the effects.
//  10. Return CBRET_NONE; the stub's trailing `CB` (RETF) byte
//      returns to the RM caller.
//
// 16-bit PM callbacks only this session -- a 32-bit callback's RETF
// pops 6-byte frame so we'd need to push CS as a dword-padded word.
Bitu do_rm_callback(int idx) {
  if (idx < 0 || idx >= RM_CALLBACK_COUNT) return CBRET_NONE;
  const RmCallback cb = s_rm_callbacks[idx];
  if (!cb.allocated) return CBRET_NONE;

  const PhysPt rmcs = pm_selector_linear(cb.struct_sel, cb.struct_off);
  mem_writed(rmcs + 0x00, reg_edi);
  mem_writed(rmcs + 0x04, reg_esi);
  mem_writed(rmcs + 0x08, reg_ebp);
  mem_writed(rmcs + 0x10, reg_ebx);
  mem_writed(rmcs + 0x14, reg_edx);
  mem_writed(rmcs + 0x18, reg_ecx);
  mem_writed(rmcs + 0x1C, reg_eax);
  mem_writew(rmcs + 0x20, static_cast<uint16_t>(reg_flags & 0xFFFF));
  mem_writew(rmcs + 0x22, SegValue(es));
  mem_writew(rmcs + 0x24, SegValue(ds));
  mem_writew(rmcs + 0x26, SegValue(fs));
  mem_writew(rmcs + 0x28, SegValue(gs));
  mem_writew(rmcs + 0x2A, static_cast<uint16_t>(reg_eip & 0xFFFF));
  mem_writew(rmcs + 0x2C, SegValue(cs));
  mem_writew(rmcs + 0x2E, static_cast<uint16_t>(reg_esp & 0xFFFF));
  mem_writew(rmcs + 0x30, SegValue(ss));

  const uint32_t saved_cr0 = cpu.cr0;
  const uint16_t saved_cs  = SegValue(cs);
  const uint32_t saved_eip = reg_eip;
  const uint16_t saved_ss  = SegValue(ss);
  const uint32_t saved_esp = reg_esp;
  const Bitu saved_idt_b = CPU_SIDT_base();
  const Bitu saved_idt_l = CPU_SIDT_limit();

  CPU_LIDT(IDT_LIMIT, IDT_BASE);
  CPU_SET_CRX(0, saved_cr0 | 1u);

  bool cb_big = false;
  {
    Descriptor desc;
    cpu.gdt.GetDescriptor(cb.pm_cs, desc);
    Segs.val[cs]  = cb.pm_cs;
    Segs.phys[cs] = desc.GetBase();
    cb_big        = desc.Big() > 0;
    cpu.code.big  = cb_big;
  }
  CPU_SetSegGeneral(ds, cb.struct_sel);
  CPU_SetSegGeneral(es, cb.struct_sel);
  CPU_SetSegGeneral(ss, PM_CB_STACK);
  reg_edi = cb.struct_off;
  reg_esp = PM_CB_STACK_SIZE - 16;

  // Push RETF frame pointing at our RM-stop callback.  Size depends
  // on the callback's CS bitness: 16-bit RETF pops 4 bytes (IP:CS),
  // 32-bit RETF pops 8 bytes (EIP:CS-zero-padded).
  const uint16_t stop_off = static_cast<uint16_t>(s_rm_stop_ptr & 0xFFFF);
  if (cb_big) {
    reg_esp -= 8;
    mem_writed(SegPhys(ss) + reg_esp + 0, stop_off);
    mem_writew(SegPhys(ss) + reg_esp + 4, PM_CB_SEL);
    mem_writew(SegPhys(ss) + reg_esp + 6, 0);
  } else {
    reg_esp -= 4;
    mem_writew(SegPhys(ss) + reg_esp + 0, stop_off);
    mem_writew(SegPhys(ss) + reg_esp + 2, PM_CB_SEL);
  }

  reg_eip = cb.pm_eip;
  DOSBOX_RunMachine();

  CPU_SET_CRX(0, saved_cr0);
  CPU_LIDT(saved_idt_l, saved_idt_b);

  reg_edi = mem_readd(rmcs + 0x00);
  reg_esi = mem_readd(rmcs + 0x04);
  reg_ebp = mem_readd(rmcs + 0x08);
  reg_ebx = mem_readd(rmcs + 0x10);
  reg_edx = mem_readd(rmcs + 0x14);
  reg_ecx = mem_readd(rmcs + 0x18);
  reg_eax = mem_readd(rmcs + 0x1C);
  reg_flags = mem_readw(rmcs + 0x20) | (reg_flags & 0xFFFF0000);
  SegSet16(es, mem_readw(rmcs + 0x22));
  SegSet16(ds, mem_readw(rmcs + 0x24));
  SegSet16(fs, mem_readw(rmcs + 0x26));
  SegSet16(gs, mem_readw(rmcs + 0x28));
  SegSet16(ss, saved_ss);
  reg_esp = saved_esp;
  // CS/EIP were changed during the PM callback (via manual cache
  // write + RunMachine).  The CPU loop resumes at (Segs.phys[cs] +
  // reg_eip) after we return, and that has to be the stub's own
  // trailing `CB` (RETF) byte so the RM caller gets its return
  // address.  Restore both.
  SegSet16(cs, saved_cs);
  reg_eip = saved_eip;
  return CBRET_NONE;
}

// Virtual interrupt state for AX=0900/0901/0902.  Mirrors IF visibility
// from the client's perspective -- real CPU IF is under its own control
// via STI/CLI.  We don't forward IF changes to dosbox's real IRQ
// masking (no real IRQs wire in from the host), so this is a pure
// shadow state the client can round-trip through 0900/0901/0902.
bool s_virtual_if = true;    // clients assume interrupts enabled at start

// Nested-process state for AH=4Bh (Load and Execute).  When a parent
// calls AH=4B, our handler pushes one of these onto process_stack,
// switches the CPU to the child, and invokes DOSBOX_RunMachine
// recursively.  When the child does AH=4Ch, the nested-aware exit
// handler returns CBRET_STOP to unwind that RunMachine and our AH=4B
// handler pops the frame back off to restore parent state.
struct ProcessState {
  uint16_t cs, ds, ss, es, fs, gs;
  uint32_t eip, esp, ebp;
  uint32_t eax, ebx, ecx, edx, esi, edi;
  uint32_t eflags;
  uint16_t child_data_seg;   // MCB data seg of child's memory, for mcb_free
  uint16_t child_env_seg;    // MCB data seg of child's env copy (0 = none)
  uint16_t child_exit_code;  // populated by AH=4C before CBRET_STOP
};
std::vector<ProcessState> s_process_stack;

// Set by AH=4Ch when a child process exits while nested inside a
// simulate-RM-INT.  The simulate-RM-INT return path checks this and
// propagates CBRET_STOP all the way out so AH=4Bh's RunMachine
// unwinds cleanly, instead of trying to IRET back to the child's
// now-dead PM context (which fails with "Illegal descriptor type").
bool s_child_exit_pending = false;

// Last terminated child's exit info for AH=4Dh.  Low byte = exit
// code from AH=4Ch, high byte = termination type (we always report
// 0 = normal).  Updated by AH=4Bh after restoring the parent; read
// (and effectively latched, spec allows consumption) by AH=4Dh.
uint16_t s_last_child_exit = 0;

// PM exception handler table for AX=0202/0203.  (Forward-declared
// earlier so the per-vector dispatch trampolines can reference it.)
// Populated by AX=0203; AX=0202 reads it back.  Dispatch happens in
// dosemu_pm_exc_dispatch which uses the CWSDPMI stack layout that
// DJGPP's exception_handler assumes.
ExcHandler s_pm_exc[32] = {};

// INT 31h (DPMI) — stage 4 subset.
//
//   AX=0400  Get DPMI version.
//   AX=0006  Get segment base address (BX=selector -> CX:DX=base).
//   AX=0007  Set segment base address (BX=selector, CX:DX=base).
//
// Everything else still returns CF=1 / AX=8001h ("unsupported DPMI
// function").  AX=0006/0007 read and write the base field of a GDT
// descriptor in the GDT we installed at GDT_BASE; selectors outside
// GDT_LIMIT return AX=8022h (invalid selector).  LDT management (LDT
// descriptor allocation) is still stage 4 proper and not wired up.
// Descriptor helpers shared by AX=0000/0001/0002 and AX=0006/0007.
// `selector_table_base` returns LDT_BASE if TI=1, GDT_BASE otherwise.
// `selector_is_valid` rejects selectors whose index is out of range for
// the chosen table (GDT: 8 entries, LDT: 256 entries), or slot 0 for
// LDT (reserved null).
inline PhysPt selector_table_base(uint16_t sel) {
  return (sel & 0x4) ? LDT_BASE : GDT_BASE;
}
inline bool selector_is_valid(uint16_t sel) {
  const uint16_t idx = sel >> 3;
  if (sel & 0x4) {
    if (idx == 0 || idx >= LDT_COUNT) return false;
  } else {
    if ((idx * 8u + 7u) > GDT_LIMIT) return false;
  }
  return true;
}

// Write a data/code descriptor into LDT[idx].  Base/limit/access mirror
// write_gdt_descriptor; used by AX=0000 (installs a present-but-empty
// slot) and AX=0002 (installs a real-mode-segment alias).
void write_ldt_descriptor(int idx, uint32_t base, uint32_t limit,
                          uint8_t access, bool bits32) {
  if (dosemu::g_debug.ldt_trace) {
    std::fprintf(stderr,
        "[ldt-write] LDT[%d] sel=0x%04x base=0x%08x limit=0x%08x access=0x%02x "
        "bits32=%d (client cs:eip=%04x:%08x)\n",
        idx, (unsigned)((idx << 3) | 0x07), base, limit, access, bits32,
        (unsigned)SegValue(cs), (unsigned)reg_eip);
  }
  const PhysPt p = LDT_BASE + idx * 8u;
  mem_writeb(p + 0, limit & 0xFF);
  mem_writeb(p + 1, (limit >> 8) & 0xFF);
  mem_writeb(p + 2, base & 0xFF);
  mem_writeb(p + 3, (base >> 8) & 0xFF);
  mem_writeb(p + 4, (base >> 16) & 0xFF);
  mem_writeb(p + 5, access);
  const uint8_t flags = bits32 ? 0x40 : 0x00;
  mem_writeb(p + 6, ((limit >> 16) & 0x0F) | flags);
  mem_writeb(p + 7, (base >> 24) & 0xFF);
}

// LDT allocation bitmap helpers.
inline bool ldt_bit(uint16_t idx) {
  return (s_ldt_in_use[idx >> 3] >> (idx & 7)) & 1;
}
inline void ldt_set(uint16_t idx, bool v) {
  if (v) s_ldt_in_use[idx >> 3] |=  (1u << (idx & 7));
  else   s_ldt_in_use[idx >> 3] &= ~(1u << (idx & 7));
}

// Find a run of `count` consecutive free slots starting at or after 1.
// Returns the first index, or 0 if no run is available (0 doubles as
// "not found" because slot 0 is reserved and always reads as in-use).
uint16_t ldt_find_run(uint16_t count) {
  uint16_t start = 0;
  uint16_t have  = 0;
  for (uint16_t i = 1; i < LDT_COUNT; ++i) {
    if (ldt_bit(i)) { have = 0; start = 0; continue; }
    if (have == 0) start = i;
    if (++have == count) return start;
  }
  return 0;
}

Bitu dosemu_int31() {
  if (dosemu::g_debug.trace) {
    std::fprintf(stderr, "[int31] AX=%04x BX=%04x CX=%04x DX=%04x EDI=%08x ESI=%08x\n",
                 reg_ax, reg_bx, reg_cx, reg_dx, reg_edi, reg_esi);
  }
  switch (reg_ax) {

    case 0x0400: {  // Get DPMI version
      // Advertise DPMI 0.90 (matches the INT 2Fh/1687h detection response,
      // which reports DH:DL = 0:5Ah = "DPMI 0.90").  Clients that want
      // DPMI 1.0 extensions call AX=0401 (Get Capabilities) separately;
      // we handle the 1.0 sub-functions CWSDPMI implements without
      // bumping the version.
      reg_ax = 0x005A;         // major 0, minor 0x5A (= 90 decimal)
      reg_bx = 0x0002;         // flags: bit 1 = reflect real-mode INTs
      reg_cl = 3;              // CPU type: 386
      reg_dh = 0x08;           // master PIC base interrupt
      reg_dl = 0x70;           // slave PIC base interrupt
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0401: {  // DPMI 1.0: Get DPMI Capabilities
      // Output: AX = host capability flags (bit0=paging, bit1=device
      //              mapping, bit2=conventional-mem mapping, bit3=demand
      //              zero-fill, bit4=write-protect on client, bit5=host
      //              can exception-handle).
      //         CX = 0, BX = 0 (reserved)
      //         DH:DL = host version (major.minor)
      //         ES:EDI = 128-byte buffer that receives the host vendor
      //                  string: [major.byte][minor.byte][name ASCIIZ].
      // We don't page, don't demand-fill, don't enforce write-protect.
      // We *do* dispatch PM exceptions to client handlers (bit 5).
      // Mirror CWSDPMI's buffer layout: 2-byte version + ASCIIZ vendor.
      reg_ax = 0x0020;            // bit 5 only = exception handling
      reg_bx = 0;
      reg_cx = 0;
      reg_dh = 0;
      reg_dl = 0x5A;              // 0.90 host
      const PhysPt buf = SegPhys(es) + reg_edi;
      mem_writeb(buf + 0, 0);     // major
      mem_writeb(buf + 1, 1);     // minor
      const char name[] = "dosemu";
      for (size_t i = 0; i < sizeof(name); ++i)
        mem_writeb(buf + 2 + i, name[i]);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0000: {  // Allocate LDT descriptors
      // Input:  CX = number of descriptors (1..LDT_COUNT-1)
      // Output: AX = base selector; subsequent selectors are +8 apart.
      // Error:  CF=1, AX=8011h (descriptor unavailable).
      const uint16_t count = reg_cx;
      if (count == 0 || count >= LDT_COUNT) {
        reg_ax = 0x8021; set_cf(true); return CBRET_NONE;
      }
      const uint16_t start = ldt_find_run(count);
      if (start == 0) {
        reg_ax = 0x8011; set_cf(true); return CBRET_NONE;
      }
      // Access byte: 0x92 for ring-0 client, 0xF2 for ring-3.
      const uint8_t access = 0x92 | (s_client_cpl << 5);
      for (uint16_t i = 0; i < count; ++i) {
        ldt_set(start + i, true);
        write_ldt_descriptor(start + i, 0, 0, access);
      }
      reg_ax = (start << 3) | 0x04 | s_client_cpl;   // TI=1, RPL=client
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0001: {  // Free LDT descriptor
      // Input: BX = selector.  Rejects GDT selectors (TI=0) and slot 0.
      //
      // Per DPMI spec the host may or may not clear the descriptor
      // bytes on free.  CWSDPMI leaves them intact and just marks
      // the slot as available for re-allocation; DJGPP's runtime
      // relies on that -- it frees selectors during cleanup and then
      // re-uses them (loading DS with the freed value as part of a
      // state-restore path) without re-issuing AX=0007/0008/0009.
      // If we had zeroed the descriptor (P=0, etc.), that reload
      // would #GP.  Match CWSDPMI: update only the in-use bitmap
      // and the seg-to-descriptor cache, leave the descriptor
      // bytes alone.
      if (!(reg_bx & 0x4) || !selector_is_valid(reg_bx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      const uint16_t idx = reg_bx >> 3;
      if (!ldt_bit(idx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      ldt_set(idx, false);
      // Drop any segment-to-descriptor cache entry that aliased this idx.
      for (uint32_t s = 0; s < 65536; ++s) {
        if (s_seg2desc_cache[s] == idx) s_seg2desc_cache[s] = 0;
      }
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0002: {  // Segment to descriptor
      // Input:  BX = real-mode segment value
      // Output: AX = LDT selector aliasing that segment (base = BX*16,
      //         limit = 64K-1, data r/w).  Repeated calls with the same
      //         BX must return the same selector per DPMI spec.
      const uint16_t seg = reg_bx;
      if (s_seg2desc_cache[seg] != 0) {
        // DPMI spec: identity semantics -- same RM seg returns same
        // selector across calls.  RPL must still match the client's
        // CPL so ring-3 clients don't get a RPL=0 selector back (which
        // would refuse to load into SS; would load into DS/ES/etc. but
        // only because DPL=3 happens to satisfy MAX(CPL,RPL) <= DPL).
        reg_ax = (s_seg2desc_cache[seg] << 3) | 0x04 | s_client_cpl;
        set_cf(false);
        return CBRET_NONE;
      }
      const uint16_t idx = ldt_find_run(1);
      if (idx == 0) {
        reg_ax = 0x8011; set_cf(true); return CBRET_NONE;
      }
      ldt_set(idx, true);
      // Access byte 0x92 + DPL: 0x92 for ring-0, 0xF2 for ring-3.
      const uint8_t access = 0x92 | (s_client_cpl << 5);
      write_ldt_descriptor(idx, seg * 16u, 0xFFFF, access);
      s_seg2desc_cache[seg] = idx;
      reg_ax = (idx << 3) | 0x04 | s_client_cpl;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0003: {  // Get selector increment
      reg_ax = 8;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0008: {  // Set Segment Limit
      // Input:  BX = selector, CX:DX = new limit (byte count or page
      //         count depending on granularity).
      // We accept a byte limit in CX:DX and auto-switch to page
      // granularity (G=1) when it exceeds 0xFFFFF.  Descriptor access
      // byte and base are preserved.
      if (!selector_is_valid(reg_bx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      uint32_t limit = (static_cast<uint32_t>(reg_cx) << 16) | reg_dx;
      const bool need_g = (limit > 0xFFFFF);
      if (need_g) limit = (limit >> 12);  // 4KB pages
      const uint16_t idx = reg_bx >> 3;
      const PhysPt p = selector_table_base(reg_bx) + idx * 8u;
      mem_writeb(p + 0, limit & 0xFF);
      mem_writeb(p + 1, (limit >> 8) & 0xFF);
      // Preserve bits 4-7 (D/AVL) of flags nibble; rewrite bits 0-3
      // (limit high) and bit 7 (G).
      uint8_t flags = mem_readb(p + 6);
      flags &= 0x60;                                 // keep D, AVL
      flags |= (limit >> 16) & 0x0F;
      if (need_g) flags |= 0x80;                     // G=1
      mem_writeb(p + 6, flags);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0009: {  // Set Descriptor Access Rights
      // Input:  BX = selector, CX = access rights word
      //   CL = access byte (bit 7 P, bits 5-6 DPL, bit 4 S,
      //        bits 0-3 type)
      //   CH = bits 4-7 of flags nibble (G, D, AVL -- high of byte 6)
      if (!selector_is_valid(reg_bx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      const uint16_t idx = reg_bx >> 3;
      const PhysPt p = selector_table_base(reg_bx) + idx * 8u;
      mem_writeb(p + 5, reg_cl);
      uint8_t flags = mem_readb(p + 6);
      flags = (flags & 0x0F) | (reg_ch & 0xF0);
      mem_writeb(p + 6, flags);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x000A: {  // Create Alias Descriptor
      // Input:  BX = source selector (must be code or data)
      // Output: AX = new LDT selector with the same base and limit
      //         but always readable+writable data (access = 0x92).
      if (!selector_is_valid(reg_bx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      const uint16_t src_idx = reg_bx >> 3;
      const PhysPt src = selector_table_base(reg_bx) + src_idx * 8u;
      const uint32_t base = mem_readb(src + 2)
                          | (mem_readb(src + 3) << 8)
                          | (mem_readb(src + 4) << 16)
                          | (mem_readb(src + 7) << 24);
      const uint32_t limit_lo = mem_readb(src + 0) | (mem_readb(src + 1) << 8);
      const uint32_t limit_hi = mem_readb(src + 6) & 0x0F;
      const uint32_t limit    = limit_lo | (limit_hi << 16);
      const uint16_t new_idx = ldt_find_run(1);
      if (new_idx == 0) {
        reg_ax = 0x8011; set_cf(true); return CBRET_NONE;
      }
      ldt_set(new_idx, true);
      // Access with DPL matching client CPL (0x92 ring-0 / 0xF2 ring-3).
      const uint8_t access = 0x92 | (s_client_cpl << 5);
      write_ldt_descriptor(new_idx, base, limit, access);
      reg_ax = (new_idx << 3) | 0x04 | s_client_cpl;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x000B: {  // Get Descriptor (raw 8 bytes)
      // Input:  BX = selector, ES:(E)DI = destination buffer
      // Copies the 8 descriptor bytes verbatim.  Client can decode
      // them however it wants without going through 0006/0008/0009.
      if (!selector_is_valid(reg_bx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      const uint16_t idx = reg_bx >> 3;
      const PhysPt src = selector_table_base(reg_bx) + idx * 8u;
      const PhysPt dst = SegPhys(es) + reg_edi;
      for (int i = 0; i < 8; ++i) mem_writeb(dst + i, mem_readb(src + i));
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x000C: {  // Set Descriptor (raw 8 bytes)
      // Input:  BX = selector, ES:(E)DI = source buffer
      // Writes the 8 bytes verbatim into the LDT (or GDT) slot.
      // Caller is responsible for a valid access byte / flags.
      if (!selector_is_valid(reg_bx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      const uint16_t idx = reg_bx >> 3;
      const PhysPt dst = selector_table_base(reg_bx) + idx * 8u;
      const PhysPt src = SegPhys(es) + reg_edi;
      for (int i = 0; i < 8; ++i) mem_writeb(dst + i, mem_readb(src + i));
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x000D: {  // DPMI 1.0: Allocate Specific LDT Descriptor
      // Input:  BX = specific selector to allocate (must be LDT,
      //              currently free, and RPL field unset).
      // Output: CF=0 on success; CF=1 AX=8011 if unavailable.
      // Used by clients that need a selector at a fixed value (e.g.
      // for self-modifying code or DOS interop with a specific
      // value).  CWSDPMI limits this to slots 0..15 with TI=1; we
      // allow anywhere in the LDT bitmap.
      if (!(reg_bx & 0x4)) {       // must be LDT (TI=1)
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      const uint16_t idx = reg_bx >> 3;
      if (idx == 0 || idx >= LDT_COUNT || ldt_bit(idx)) {
        reg_ax = 0x8011; set_cf(true); return CBRET_NONE;
      }
      ldt_set(idx, true);
      write_ldt_descriptor(idx, 0, 0, 0x92);    // placeholder data r/w
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0100: {  // Allocate DOS Memory Block
      // Input:  BX = paragraphs to allocate
      // Output: AX = real-mode segment, DX = PM selector aliasing it
      // Error:  CF=1, AX=8011/8013, BX = largest free paragraphs
      //
      // This is the DPMI hybrid-allocator: the returned memory must be
      // addressable both in RM (via the segment) and PM (via the
      // selector).  We delegate to the DOS MCB chain for the bytes and
      // to the LDT allocator for a selector, then wire the selector's
      // descriptor to cover the MCB region.
      if (reg_bx == 0) {
        reg_ax = 0x8021; set_cf(true); return CBRET_NONE;
      }
      uint16_t largest = 0;
      const uint16_t data_seg = mcb_allocate(reg_bx, largest);
      if (data_seg == 0) {
        reg_ax = 0x8013;
        reg_bx = largest;        // spec: return largest available on OOM
        set_cf(true);
        return CBRET_NONE;
      }
      const uint16_t ldt_idx = ldt_find_run(1);
      if (ldt_idx == 0) {
        mcb_free(data_seg);      // roll back the MCB alloc
        reg_ax = 0x8011;
        set_cf(true);
        return CBRET_NONE;
      }
      ldt_set(ldt_idx, true);
      // Limit covers the allocated region exactly: bx paragraphs * 16 - 1.
      const uint32_t limit_bytes = (static_cast<uint32_t>(reg_bx) << 4) - 1u;
      // Access byte's DPL matches client CPL: 0x92 ring-0, 0xF2 ring-3.
      const uint8_t access = 0x92 | (s_client_cpl << 5);
      write_ldt_descriptor(ldt_idx, data_seg * 16u, limit_bytes, access);
      // Record the alias so AX=0002 on the same RM seg returns the
      // same selector (DPMI identity semantics).
      s_seg2desc_cache[data_seg] = ldt_idx;
      reg_ax = data_seg;
      reg_dx = (ldt_idx << 3) | 0x04 | s_client_cpl;   // TI=1, RPL=client
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0102: {  // Resize DOS Memory Block
      // Input:  BX = selector (returned by AX=0100)
      //         DX = new size in paragraphs
      // Output: on failure CF=1, AX=error code, BX = largest available
      if (!(reg_bx & 0x4) || !selector_is_valid(reg_bx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      const uint16_t idx = reg_bx >> 3;
      if (!ldt_bit(idx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      const PhysPt p = LDT_BASE + idx * 8u;
      const uint32_t base = mem_readb(p + 2)
                          | (mem_readb(p + 3) << 8)
                          | (mem_readb(p + 4) << 16)
                          | (mem_readb(p + 7) << 24);
      const uint16_t data_seg = static_cast<uint16_t>(base >> 4);
      uint16_t largest = 0;
      const uint16_t got = mcb_resize(data_seg, reg_dx, largest);
      if (got != reg_dx) {
        reg_ax = 0x8013;
        reg_bx = largest;
        set_cf(true);
        return CBRET_NONE;
      }
      // Update the descriptor's limit to match the new size.
      const uint32_t new_limit_bytes = (static_cast<uint32_t>(reg_dx) << 4) - 1u;
      mem_writeb(p + 0, new_limit_bytes & 0xFF);
      mem_writeb(p + 1, (new_limit_bytes >> 8) & 0xFF);
      const uint8_t flags = mem_readb(p + 6);
      mem_writeb(p + 6, (flags & 0xF0) | ((new_limit_bytes >> 16) & 0x0F));
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0101: {  // Free DOS Memory Block
      // Input:  DX = selector returned by AX=0100.
      if (!(reg_dx & 0x4) || !selector_is_valid(reg_dx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      const uint16_t idx = reg_dx >> 3;
      if (!ldt_bit(idx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      // Recover the RM segment this selector was aliasing from the
      // descriptor's base, then free both the MCB and the LDT slot.
      const PhysPt p = LDT_BASE + idx * 8u;
      const uint32_t base = mem_readb(p + 2)
                          | (mem_readb(p + 3) << 8)
                          | (mem_readb(p + 4) << 16)
                          | (mem_readb(p + 7) << 24);
      const uint16_t data_seg = static_cast<uint16_t>(base >> 4);
      mcb_free(data_seg);
      s_seg2desc_cache[data_seg] = 0;
      ldt_set(idx, false);
      write_ldt_descriptor(idx, 0, 0, 0);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0202: {  // Get Protected Mode Exception Handler
      // Input:  BL = exception number (0..31)
      // Output: CX:(E)DX = current handler
      if (reg_bl >= 32) {
        reg_ax = 0x8021; set_cf(true); return CBRET_NONE;
      }
      reg_cx  = s_pm_exc[reg_bl].sel;
      reg_edx = s_pm_exc[reg_bl].off;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0203: {  // Set Protected Mode Exception Handler
      if (reg_bl >= 32) {
        reg_ax = 0x8021; set_cf(true); return CBRET_NONE;
      }
      s_pm_exc[reg_bl].sel = reg_cx;
      s_pm_exc[reg_bl].off = reg_edx;
      // IDT gate wiring depends on the user handler's CS bitness:
      //
      //   32-bit handler  -> point the gate at our per-vector CWSDPMI
      //                      trampoline.  CPU dispatches to the
      //                      trampoline, which constructs the 32-byte
      //                      CWSDPMI exception frame on the client's
      //                      stack before jumping to the user handler.
      //                      Required by DJGPP-style clients that LRET
      //                      through user_exception_return and assume
      //                      [SP+12..+28] = err/EIP/CS/EFLAGS/ESP/SS.
      //
      //   16-bit handler  -> direct gate-to-handler; CPU-pushed 6-byte
      //                      IRET frame matches what pre-CWSDPMI 16-bit
      //                      DPMI clients (and our DPMI_EXC fixture)
      //                      expect.  These clients IRET directly.
      //
      // Bitness is read from the user handler's CS descriptor (D bit).
      Descriptor cs_desc;
      bool handler_bits32 = false;
      if (cpu.gdt.GetDescriptor(reg_cx, cs_desc)) {
        handler_bits32 = cs_desc.Big() != 0;
      }
      if (handler_bits32 && s_pm_exc_cb32_off[reg_bl]) {
        write_idt_gate(reg_bl, PM_CB_SEL, s_pm_exc_cb32_off[reg_bl],
                       /*bits32=*/true);
      } else {
        write_idt_gate(reg_bl, reg_cx, reg_edx, handler_bits32);
      }
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0305: {  // Get State Save/Restore Addresses
      // Returns buffer size (AX, paragraphs-or-bytes depending on
      // caller) and two far pointers to save/restore stubs.  With
      // no virtualization to save, we return 0 bytes and point both
      // routines at a no-op RETF callback.
      reg_ax = 0;                                    // buffer size
      reg_bx = static_cast<uint16_t>((s_noop_retf_ptr >> 16) & 0xFFFF);
      reg_cx = static_cast<uint16_t>(s_noop_retf_ptr & 0xFFFF);
      reg_si = static_cast<uint16_t>((s_noop_retf_ptr >> 16) & 0xFFFF);
      reg_edi = static_cast<uint16_t>(s_noop_retf_ptr & 0xFFFF);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0303: {  // Allocate Real Mode Callback Address
      // Input:  DS:(E)SI = PM callback procedure (selector:offset)
      //         ES:(E)DI = client's RealModeCallStructure (sel:off)
      // Output: CX:DX    = real-mode seg:off that RM code can FAR-CALL
      //                    to invoke the registered PM procedure.
      int slot = -1;
      for (int i = 0; i < RM_CALLBACK_COUNT; ++i) {
        if (!s_rm_callbacks[i].allocated) { slot = i; break; }
      }
      if (slot < 0) { reg_ax = 0x8015; set_cf(true); return CBRET_NONE; }
      RmCallback &cb = s_rm_callbacks[slot];
      cb.allocated  = true;
      cb.pm_cs      = SegValue(ds);
      cb.pm_eip     = reg_esi;
      cb.struct_sel = SegValue(es);
      cb.struct_off = reg_edi;
      reg_cx = static_cast<uint16_t>((cb.rm_addr >> 16) & 0xFFFF);
      reg_dx = static_cast<uint16_t>(cb.rm_addr & 0xFFFF);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0304: {  // Free Real Mode Callback Address
      // Input: CX:DX = RM address of a callback allocated by 0303.
      const RealPt target = (static_cast<uint32_t>(reg_cx) << 16) | reg_dx;
      for (int i = 0; i < RM_CALLBACK_COUNT; ++i) {
        if (s_rm_callbacks[i].allocated &&
            s_rm_callbacks[i].rm_addr == target) {
          s_rm_callbacks[i].allocated = false;
          set_cf(false);
          return CBRET_NONE;
        }
      }
      reg_ax = 0x8024;               // invalid callback
      set_cf(true);
      return CBRET_NONE;
    }

    case 0x0306: {  // Get Raw Mode Switch Addresses
      reg_bx = static_cast<uint16_t>((s_noop_retf_ptr >> 16) & 0xFFFF);
      reg_cx = static_cast<uint16_t>(s_noop_retf_ptr & 0xFFFF);
      reg_si = static_cast<uint16_t>((s_noop_retf_ptr >> 16) & 0xFFFF);
      reg_edi = static_cast<uint16_t>(s_noop_retf_ptr & 0xFFFF);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0600:   // Lock Linear Region (no-op -- no paging)
    case 0x0601:   // Unlock Linear Region (no-op)
    case 0x0602:   // Mark RM Region As Pageable (no-op)
    case 0x0603:   // Mark RM Region As Unpageable (no-op)
    case 0x0702:   // Mark Page As Demand Paging Candidate (no-op)
    case 0x0703: { // Discard Page Contents (no-op)
      // Input BX:CX = linear addr, SI:DI = size.  Ignored: physical
      // memory is never paged out in our model, so the "locked"
      // promise is trivially satisfied and pageability marks have
      // no bearing.
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0801: { // Free Physical Address Mapping (no-op)
      // We don't track mappings (AX=0800 is identity pass-through),
      // so there's nothing to free.
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0B00:   // Set Debug Watchpoint
    case 0x0B01:   // Clear Debug Watchpoint
    case 0x0B02:   // Get State of Debug Watchpoint
    case 0x0B03: { // Reset Debug Watchpoint
      // Debug registers (DR0-DR7) aren't wired through.  We accept
      // the request so init code proceeds, but we can't actually
      // deliver a breakpoint interrupt.  AX=0B00 returns a fake
      // watchpoint handle (always 0); others acknowledge.
      reg_bx = 0;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0604: {  // Get Page Size
      reg_bx = 0;
      reg_cx = 4096;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0800: {  // Physical Address Mapping
      // Input:  BX:CX = physical address, SI:DI = size (ignored -- we
      //                don't enforce any range on the mapping).
      // Output: BX:CX = linear address
      // Without a paging layer physical == linear, so we just echo
      // the input.  Callers use this to get a linear address for
      // framebuffer / memory-mapped-device ranges they already know
      // the physical of.
      set_cf(false);
      return CBRET_NONE;
    }

    // AX=0204/0205 "PM interrupt vector get/set" per DPMI 0.9 spec.
    // (Earlier versions of this file mislabelled 0204/0205 as RM-IVT
    // and used 0210/0212 for PM; this commit renumbers to match the
    // spec.  Fixtures updated alongside.)
    case 0x0204: {  // Get Protected Mode Interrupt Vector
      const PhysPt gate = IDT_BASE + reg_bl * 8u;
      const uint32_t off = mem_readb(gate + 0)
                         | (mem_readb(gate + 1) << 8)
                         | (mem_readb(gate + 6) << 16)
                         | (mem_readb(gate + 7) << 24);
      const uint16_t sel = mem_readb(gate + 2)
                         | (mem_readb(gate + 3) << 8);
      reg_cx  = sel;
      reg_edx = off;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0205: {  // Set Protected Mode Interrupt Vector
      const bool bits32 = cpu.code.big;
      write_idt_gate(reg_bl, reg_cx, reg_edx, bits32);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0900: {  // Get and Disable Virtual Interrupt State
      reg_al = s_virtual_if ? 1 : 0;     // AL = previous state
      s_virtual_if = false;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0901: {  // Get and Enable Virtual Interrupt State
      reg_al = s_virtual_if ? 1 : 0;     // AL = previous state
      s_virtual_if = true;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0902: {  // Get Virtual Interrupt State
      reg_al = s_virtual_if ? 1 : 0;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0200: {  // Get Real Mode Interrupt Vector
      const PhysPt v = static_cast<uint32_t>(reg_bl) * 4u;
      reg_dx = mem_readw(v);
      reg_cx = mem_readw(v + 2);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0201: {  // Set Real Mode Interrupt Vector
      const PhysPt v = static_cast<uint32_t>(reg_bl) * 4u;
      mem_writew(v,     reg_dx);
      mem_writew(v + 2, reg_cx);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0500: {  // Get free memory information
      // Writes a 0x30-byte buffer at ES:(E)DI.  Most fields we report as
      // 0xFFFFFFFF ("not supported") except the largest-free-block byte
      // count which we derive from the MCB chain and the handful of
      // fields a minimal client will check.
      const PhysPt buf = SegPhys(es) + reg_edi;
      // Walk the MCB chain to find the largest free block in bytes.
      // Lazy-init the chain: a DPMI client that calls AX=0500 before
      // doing any AH=48/0501 allocation would otherwise see garbage.
      if (!s_mcb_initialised) mcb_init();
      uint32_t largest_paras = 0;
      for (uint16_t seg = MCB_ARENA_START; seg < MCB_ARENA_END; ) {
        const PhysPt m = seg * 16u;
        const uint8_t  type = mem_readb(m);
        const uint16_t owner = mem_readw(m + 1);
        const uint16_t size  = mem_readw(m + 3);
        if (owner == 0 && size > largest_paras) largest_paras = size;
        if (type == 'Z') break;
        seg = static_cast<uint16_t>(seg + 1 + size);
      }
      const uint32_t largest_bytes = largest_paras * 16u;
      auto put32 = [&](uint32_t off, uint32_t v) {
        mem_writed(buf + off, v);
      };
      put32(0x00, largest_bytes);                    // largest available block
      put32(0x04, largest_bytes / 4096);             // max unlocked page alloc
      put32(0x08, largest_bytes / 4096);             // max locked page alloc
      put32(0x0C, 0xFFFFFFFF);                       // linear address space pages
      put32(0x10, largest_bytes / 4096);             // free pages
      put32(0x14, 0xFFFFFFFF);                       // free physical pages
      put32(0x18, 0xFFFFFFFF);                       // total physical pages
      put32(0x1C, 0xFFFFFFFF);                       // free page-file pages
      put32(0x20, 0xFFFFFFFF);                       // total page-file pages
      for (uint32_t off = 0x24; off < 0x30; off += 4)
        put32(off, 0xFFFFFFFF);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0006: {  // Get segment base address (GDT or LDT)
      if (!selector_is_valid(reg_bx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      const uint16_t idx = reg_bx >> 3;
      const PhysPt p = selector_table_base(reg_bx) + idx * 8u;
      const uint32_t base = mem_readb(p + 2)
                          | (mem_readb(p + 3) << 8)
                          | (mem_readb(p + 4) << 16)
                          | (mem_readb(p + 7) << 24);
      reg_cx = (base >> 16) & 0xFFFF;
      reg_dx = base & 0xFFFF;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0007: {  // Set segment base address (GDT or LDT)
      if (!selector_is_valid(reg_bx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      const uint32_t base = (static_cast<uint32_t>(reg_cx) << 16)
                          | reg_dx;
      const uint16_t idx = reg_bx >> 3;
      const PhysPt p = selector_table_base(reg_bx) + idx * 8u;
      mem_writeb(p + 2, base & 0xFF);
      mem_writeb(p + 3, (base >> 8) & 0xFF);
      mem_writeb(p + 4, (base >> 16) & 0xFF);
      mem_writeb(p + 7, (base >> 24) & 0xFF);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0501: {  // Allocate memory block
      // Input:  BX:CX = size in bytes (BX high, CX low)
      // Output: BX:CX = linear address of block
      //         SI:DI = handle (opaque; must round-trip to 0x0502/0x0503)
      // Error:  CF=1, AX=8012h (out of memory)
      //
      // Two-tier backing store:
      //   Tier 1 (preferred): pm_arena above 1MB.  Clients that make
      //           multiple AX=0501 requests and extend a single
      //           selector's limit to cover "their arena" get a
      //           contiguous linear region that doesn't alias our
      //           kernel structures (which sit at 0x100000..0x10FFFF,
      //           below pm_arena's 0x120000 start).  Without this,
      //           DJGPP-style clients allocate 700KB across 3 blocks,
      //           set base=<first-block> + limit=1.5MB, and write at
      //           DS:0xE3FF8 -- which lands on our LDT at 0x104028
      //           whenever the first block came from the MCB arena in
      //           conventional memory.  Handle = SI:DI = high:low of
      //           host linear base (SI always >= 0x0012 because base
      //           >= 0x120000).
      //   Tier 2 (fallback): MCB arena in conventional memory.  Used
      //           when pm_arena is saturated.  Handle = SI:DI = 0:
      //           mcb_data_seg.  The two handle encodings are
      //           distinguishable by SI (0 vs >=0x12).
      const uint32_t bytes = (static_cast<uint32_t>(reg_bx) << 16) | reg_cx;
      if (bytes == 0) {
        reg_ax = 0x8021;       // invalid value
        set_cf(true);
        return CBRET_NONE;
      }
      // Tier 1: pm_arena.
      const uint32_t linear = pm_alloc(bytes);
      if (linear != 0) {
        reg_bx = (linear >> 16) & 0xFFFF;
        reg_cx = linear & 0xFFFF;
        reg_si = (linear >> 16) & 0xFFFF;
        reg_di = linear & 0xFFFF;
        set_cf(false);
        return CBRET_NONE;
      }
      // Tier 2: fall back to MCB for small requests if pm_arena full.
      const uint32_t paras32 = (bytes + 15u) >> 4;
      if (paras32 > 0 && paras32 <= 0xFFFFu) {
        uint16_t largest = 0;
        const uint16_t data_seg = mcb_allocate(
            static_cast<uint16_t>(paras32), largest);
        if (data_seg != 0) {
          const uint32_t mcb_linear = static_cast<uint32_t>(data_seg) * 16u;
          reg_bx = (mcb_linear >> 16) & 0xFFFF;
          reg_cx = mcb_linear & 0xFFFF;
          reg_si = 0;
          reg_di = data_seg;
          set_cf(false);
          return CBRET_NONE;
        }
      }
      reg_ax = 0x8012;
      set_cf(true);
      return CBRET_NONE;
    }

    case 0x0502: {  // Free memory block
      // Input: SI:DI = handle.  SI=0 means MCB tier; non-zero SI means
      // SI:DI is the linear base of a pm_arena block.
      if (reg_si == 0) {
        if (reg_di < MCB_ARENA_START || reg_di >= MCB_ARENA_END) {
          reg_ax = 0x8023; set_cf(true); return CBRET_NONE;
        }
        if (!mcb_free(reg_di)) { reg_ax = 0x8023; set_cf(true); return CBRET_NONE; }
        set_cf(false);
        return CBRET_NONE;
      }
      const uint32_t base = (static_cast<uint32_t>(reg_si) << 16) | reg_di;
      if (!pm_free(base)) { reg_ax = 0x8023; set_cf(true); return CBRET_NONE; }
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0302: {  // Call Real Mode Procedure With IRET Frame
      // Same mode-switch machinery as 0300/0301; the difference is
      // how we enter the RM procedure:
      //   - 0301 pushes a far return (CS:IP) and the callee RETFs.
      //   - 0302 pushes an IRET frame (FLAGS:CS:IP) and the callee
      //     IRETs.
      // We inline the RunMachine call (rather than reusing
      // CALLBACK_RunRealFar) so we can push the extra FLAGS word
      // above CS:IP.  Target is struct.CS:IP; the pushed return
      // address points at our rm_stop callback.
      const PhysPt rmcs = SegPhys(es) + reg_edi;

      const uint32_t saved_cr0  = cpu.cr0;
      const uint16_t saved_cs   = SegValue(cs);
      const uint16_t saved_ds   = SegValue(ds);
      const uint16_t saved_ss   = SegValue(ss);
      const uint16_t saved_es   = SegValue(es);
      const uint16_t saved_fs   = SegValue(fs);
      const uint16_t saved_gs   = SegValue(gs);
      const uint32_t saved_esp  = reg_esp;
      const uint32_t saved_ebp  = reg_ebp;
      const uint32_t saved_eax  = reg_eax;
      const uint32_t saved_ebx  = reg_ebx;
      const uint32_t saved_ecx  = reg_ecx;
      const uint32_t saved_edx  = reg_edx;
      const uint32_t saved_esi  = reg_esi;
      const uint32_t saved_edi  = reg_edi;

      const uint32_t s_edi = mem_readd(rmcs + 0x00);
      const uint32_t s_esi = mem_readd(rmcs + 0x04);
      const uint32_t s_ebp = mem_readd(rmcs + 0x08);
      const uint32_t s_ebx = mem_readd(rmcs + 0x10);
      const uint32_t s_edx = mem_readd(rmcs + 0x14);
      const uint32_t s_ecx = mem_readd(rmcs + 0x18);
      const uint32_t s_eax = mem_readd(rmcs + 0x1C);
      const uint16_t s_flags_in = mem_readw(rmcs + 0x20);
      const uint16_t s_es  = mem_readw(rmcs + 0x22);
      const uint16_t s_ds  = mem_readw(rmcs + 0x24);
      const uint16_t s_fs  = mem_readw(rmcs + 0x26);
      const uint16_t s_gs  = mem_readw(rmcs + 0x28);
      const uint16_t s_ip  = mem_readw(rmcs + 0x2A);
      const uint16_t s_cs  = mem_readw(rmcs + 0x2C);
      const uint16_t s_sp  = mem_readw(rmcs + 0x2E);
      const uint16_t s_ss  = mem_readw(rmcs + 0x30);

      const Bitu saved_idt_base  = CPU_SIDT_base();
      const Bitu saved_idt_limit = CPU_SIDT_limit();
      CPU_LIDT(0x3FF, 0);

      CPU_SET_CRX(0, saved_cr0 & ~1u);

      reg_eax = s_eax; reg_ebx = s_ebx; reg_ecx = s_ecx; reg_edx = s_edx;
      reg_esi = s_esi; reg_edi = s_edi; reg_ebp = s_ebp;
      SegSet16(ds, s_ds); SegSet16(es, s_es);
      SegSet16(fs, s_fs); SegSet16(gs, s_gs);
      if (s_ss != 0) {
        SegSet16(ss, s_ss);
        reg_esp = s_sp;
      } else {
        SegSet16(ss, 0x0050);
        reg_esp = 0x0F00;
      }

      // Push the IRET frame for our stop callback onto the RM stack:
      //   [SS:SP+0] = IP of stop callback
      //   [SS:SP+2] = CS of stop callback
      //   [SS:SP+4] = FLAGS to give the callee on entry
      reg_esp = (reg_esp - 6) & 0xFFFF;
      real_writew(SegValue(ss), reg_sp + 0,
                  static_cast<uint16_t>(s_rm_stop_ptr & 0xFFFF));
      real_writew(SegValue(ss), reg_sp + 2,
                  static_cast<uint16_t>((s_rm_stop_ptr >> 16) & 0xFFFF));
      real_writew(SegValue(ss), reg_sp + 4, s_flags_in);

      // Jump to the client's procedure and let the nested machine run.
      reg_eip = s_ip;
      SegSet16(cs, s_cs);
      DOSBOX_RunMachine();

      const uint32_t r_eax = reg_eax;
      const uint32_t r_ebx = reg_ebx;
      const uint32_t r_ecx = reg_ecx;
      const uint32_t r_edx = reg_edx;
      const uint32_t r_esi = reg_esi;
      const uint32_t r_edi = reg_edi;
      const uint32_t r_ebp = reg_ebp;
      const uint16_t r_flags = static_cast<uint16_t>(reg_flags & 0xFFFF);
      const uint16_t r_es = SegValue(es);
      const uint16_t r_ds = SegValue(ds);
      const uint16_t r_fs = SegValue(fs);
      const uint16_t r_gs = SegValue(gs);

      CPU_SET_CRX(0, saved_cr0);
      CPU_LIDT(saved_idt_limit, saved_idt_base);

      CPU_SetSegGeneral(ds, saved_ds);
      CPU_SetSegGeneral(ss, saved_ss);
      CPU_SetSegGeneral(es, saved_es);
      CPU_SetSegGeneral(fs, saved_fs);
      CPU_SetSegGeneral(gs, saved_gs);
      {
        Descriptor cs_desc;
        cpu.gdt.GetDescriptor(saved_cs, cs_desc);
        Segs.val[cs]  = saved_cs;
        Segs.phys[cs] = cs_desc.GetBase();
      }

      reg_eax = saved_eax; reg_ebx = saved_ebx;
      reg_ecx = saved_ecx; reg_edx = saved_edx;
      reg_esi = saved_esi; reg_edi = saved_edi;
      reg_ebp = saved_ebp; reg_esp = saved_esp;

      mem_writed(rmcs + 0x00, r_edi);
      mem_writed(rmcs + 0x04, r_esi);
      mem_writed(rmcs + 0x08, r_ebp);
      mem_writed(rmcs + 0x10, r_ebx);
      mem_writed(rmcs + 0x14, r_edx);
      mem_writed(rmcs + 0x18, r_ecx);
      mem_writed(rmcs + 0x1C, r_eax);
      mem_writew(rmcs + 0x20, r_flags);
      mem_writew(rmcs + 0x22, r_es);
      mem_writew(rmcs + 0x24, r_ds);
      mem_writew(rmcs + 0x26, r_fs);
      mem_writew(rmcs + 0x28, r_gs);

      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0301: {  // Call Real Mode Procedure With Far Return Frame
      // Input:
      //   BH        = flags (reserved 0)
      //   CX        = words to copy PM stack -> RM stack (ignored here)
      //   ES:(E)DI -> RealModeCallStructure (same layout as 0300).
      //     The CS:IP fields at +2A/+2C name the RM routine to call.
      // The host pushes a "stop" far-return address, far-calls the
      // target; the routine is expected to RETF, landing on the stop
      // callback which unwinds the nested emulator loop.
      //
      // Mechanism mirrors AX=0300's mode-switch round-trip: save PM
      // state, swap IDTR to RM IVT, clear PE, load RM regs/segments
      // from the struct, CALLBACK_RunRealFar, snapshot results,
      // restore PE + PM IDTR, reload PM segments + CS cache, write
      // results back.
      const PhysPt rmcs = SegPhys(es) + reg_edi;

      const uint32_t saved_cr0  = cpu.cr0;
      const uint16_t saved_cs   = SegValue(cs);
      const uint16_t saved_ds   = SegValue(ds);
      const uint16_t saved_ss   = SegValue(ss);
      const uint16_t saved_es   = SegValue(es);
      const uint16_t saved_fs   = SegValue(fs);
      const uint16_t saved_gs   = SegValue(gs);
      const uint32_t saved_esp  = reg_esp;
      const uint32_t saved_ebp  = reg_ebp;
      const uint32_t saved_eax  = reg_eax;
      const uint32_t saved_ebx  = reg_ebx;
      const uint32_t saved_ecx  = reg_ecx;
      const uint32_t saved_edx  = reg_edx;
      const uint32_t saved_esi  = reg_esi;
      const uint32_t saved_edi  = reg_edi;

      const uint32_t s_edi = mem_readd(rmcs + 0x00);
      const uint32_t s_esi = mem_readd(rmcs + 0x04);
      const uint32_t s_ebp = mem_readd(rmcs + 0x08);
      const uint32_t s_ebx = mem_readd(rmcs + 0x10);
      const uint32_t s_edx = mem_readd(rmcs + 0x14);
      const uint32_t s_ecx = mem_readd(rmcs + 0x18);
      const uint32_t s_eax = mem_readd(rmcs + 0x1C);
      const uint16_t s_es  = mem_readw(rmcs + 0x22);
      const uint16_t s_ds  = mem_readw(rmcs + 0x24);
      const uint16_t s_fs  = mem_readw(rmcs + 0x26);
      const uint16_t s_gs  = mem_readw(rmcs + 0x28);
      const uint16_t s_ip  = mem_readw(rmcs + 0x2A);
      const uint16_t s_cs  = mem_readw(rmcs + 0x2C);
      const uint16_t s_sp  = mem_readw(rmcs + 0x2E);
      const uint16_t s_ss  = mem_readw(rmcs + 0x30);

      const Bitu saved_idt_base  = CPU_SIDT_base();
      const Bitu saved_idt_limit = CPU_SIDT_limit();
      CPU_LIDT(0x3FF, 0);

      CPU_SET_CRX(0, saved_cr0 & ~1u);
      std::fprintf(stderr, "[simrm-postcr0 0301] s_eax=%08x cr0=%08x\n",
          (unsigned)s_eax, (unsigned)cpu.cr0);

      reg_eax = s_eax; reg_ebx = s_ebx; reg_ecx = s_ecx; reg_edx = s_edx;
      reg_esi = s_esi; reg_edi = s_edi; reg_ebp = s_ebp;
      SegSet16(ds, s_ds); SegSet16(es, s_es);
      SegSet16(fs, s_fs); SegSet16(gs, s_gs);
      if (s_ss != 0) {
        SegSet16(ss, s_ss);
        reg_esp = s_sp;
      } else {
        SegSet16(ss, 0x0050);
        reg_esp = 0x0F00;
      }

      CALLBACK_RunRealFar(s_cs, s_ip);

      const uint32_t r_eax = reg_eax;
      const uint32_t r_ebx = reg_ebx;
      const uint32_t r_ecx = reg_ecx;
      const uint32_t r_edx = reg_edx;
      const uint32_t r_esi = reg_esi;
      const uint32_t r_edi = reg_edi;
      const uint32_t r_ebp = reg_ebp;
      const uint16_t r_flags = static_cast<uint16_t>(reg_flags & 0xFFFF);
      const uint16_t r_es = SegValue(es);
      const uint16_t r_ds = SegValue(ds);
      const uint16_t r_fs = SegValue(fs);
      const uint16_t r_gs = SegValue(gs);

      CPU_SET_CRX(0, saved_cr0);
      CPU_LIDT(saved_idt_limit, saved_idt_base);

      CPU_SetSegGeneral(ds, saved_ds);
      CPU_SetSegGeneral(ss, saved_ss);
      CPU_SetSegGeneral(es, saved_es);
      CPU_SetSegGeneral(fs, saved_fs);
      CPU_SetSegGeneral(gs, saved_gs);
      {
        Descriptor cs_desc;
        cpu.gdt.GetDescriptor(saved_cs, cs_desc);
        Segs.val[cs]  = saved_cs;
        Segs.phys[cs] = cs_desc.GetBase();
      }

      reg_eax = saved_eax; reg_ebx = saved_ebx;
      reg_ecx = saved_ecx; reg_edx = saved_edx;
      reg_esi = saved_esi; reg_edi = saved_edi;
      reg_ebp = saved_ebp; reg_esp = saved_esp;

      mem_writed(rmcs + 0x00, r_edi);
      mem_writed(rmcs + 0x04, r_esi);
      mem_writed(rmcs + 0x08, r_ebp);
      mem_writed(rmcs + 0x10, r_ebx);
      mem_writed(rmcs + 0x14, r_edx);
      mem_writed(rmcs + 0x18, r_ecx);
      mem_writed(rmcs + 0x1C, r_eax);
      mem_writew(rmcs + 0x20, r_flags);
      mem_writew(rmcs + 0x22, r_es);
      mem_writew(rmcs + 0x24, r_ds);
      mem_writew(rmcs + 0x26, r_fs);
      mem_writew(rmcs + 0x28, r_gs);

      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0300: {  // Simulate Real Mode Interrupt
      // Input:
      //   BL        = interrupt number
      //   BH        = flags (bit 0 = reset interrupt controller; unused)
      //   CX        = words to copy from PM stack to RM stack (we ignore
      //               -- minimal implementation uses a fresh RM stack)
      //   ES:(E)DI -> RealModeCallStructure (50 bytes):
      //     +00 EDI  +04 ESI  +08 EBP  +0C reserved
      //     +10 EBX  +14 EDX  +18 ECX  +1C EAX
      //     +20 flags (word)  +22 ES  +24 DS  +26 FS  +28 GS
      //     +2A IP            +2C CS  +2E SP  +30 SS
      //
      // The struct is updated in place with register values on return.
      //
      // Mode-switch flow: save PM state, clear CR0.PE, load RM
      // registers + segments from the struct, call CALLBACK_RunRealInt
      // (which sets CS:IP to a pre-built RM stub that does `INT N` +
      // stop callback, then runs DOSBOX_RunMachine recursively), copy
      // results back to the struct, set CR0.PE=1, reload PM selectors
      // via CPU_SetSegGeneral (refreshes each descriptor cache from
      // the GDT/LDT -- crucial for CS, which otherwise retains its
      // RM-interpreted base of selector*16 from the RM phase).
      const uint8_t intnum = reg_bl;
      // Pin the struct's linear address before we touch segments.
      const PhysPt rmcs = SegPhys(es) + reg_edi;
      if (dosemu::g_debug.simrm_trace) {
        std::fprintf(stderr,
            "[simrm] INT %02x ax=%08x cx=%08x dx=%08x ds=%04x es=%04x (PM ds=%04x(b=%08x) ss=%04x(b=%08x) es=%04x(b=%08x))\n",
            (unsigned)intnum,
            (unsigned)mem_readd(rmcs + 0x1C),
            (unsigned)mem_readd(rmcs + 0x18), (unsigned)mem_readd(rmcs + 0x14),
            (unsigned)mem_readw(rmcs + 0x24), (unsigned)mem_readw(rmcs + 0x22),
            (unsigned)SegValue(ds), (unsigned)SegPhys(ds),
            (unsigned)SegValue(ss), (unsigned)SegPhys(ss),
            (unsigned)SegValue(es), (unsigned)SegPhys(es));
      }

      if (dosemu::g_debug.stackwatch) {
        const uint32_t ss_lin = SegPhys(ss) + reg_esp;
        std::fprintf(stderr, "[simrm-enter] INT %02x client SS:SP=%04x:%08x lin=%08x val=%04x\n",
            (unsigned)intnum, (unsigned)SegValue(ss), (unsigned)reg_esp,
            ss_lin, (unsigned)mem_readw(ss_lin));
      }
      // Snapshot PM state (segments + full register file + stack).
      const uint32_t saved_cr0  = cpu.cr0;
      const uint16_t saved_cs   = SegValue(cs);
      if (dosemu::g_debug.simrm_trace) {
        std::fprintf(stderr,
            "[simrm-entry] PM cs:eip=%04x:%08x cr0=%08x cpl=%u\n",
            (unsigned)saved_cs, (unsigned)reg_eip, (unsigned)saved_cr0,
            (unsigned)cpu.cpl);
      }
      const uint16_t saved_ds   = SegValue(ds);
      const uint16_t saved_ss   = SegValue(ss);
      const uint16_t saved_es   = SegValue(es);
      const uint16_t saved_fs   = SegValue(fs);
      const uint16_t saved_gs   = SegValue(gs);
      const uint32_t saved_esp  = reg_esp;
      const uint32_t saved_ebp  = reg_ebp;
      const uint32_t saved_eax  = reg_eax;
      const uint32_t saved_ebx  = reg_ebx;
      const uint32_t saved_ecx  = reg_ecx;
      const uint32_t saved_edx  = reg_edx;
      const uint32_t saved_esi  = reg_esi;
      const uint32_t saved_edi  = reg_edi;

      // Load RM register context from the struct.
      const uint32_t s_edi = mem_readd(rmcs + 0x00);
      const uint32_t s_esi = mem_readd(rmcs + 0x04);
      const uint32_t s_ebp = mem_readd(rmcs + 0x08);
      const uint32_t s_ebx = mem_readd(rmcs + 0x10);
      const uint32_t s_edx = mem_readd(rmcs + 0x14);
      const uint32_t s_ecx = mem_readd(rmcs + 0x18);
      const uint32_t s_eax = mem_readd(rmcs + 0x1C);
      const uint16_t s_es  = mem_readw(rmcs + 0x22);
      const uint16_t s_ds  = mem_readw(rmcs + 0x24);
      const uint16_t s_fs  = mem_readw(rmcs + 0x26);
      const uint16_t s_gs  = mem_readw(rmcs + 0x28);
      const uint16_t s_sp  = mem_readw(rmcs + 0x2E);
      const uint16_t s_ss  = mem_readw(rmcs + 0x30);

      // Swap IDTR to the RM IVT (base=0, limit=0x3FF) for the RM
      // round-trip.  On a 386, real-mode INT dispatch uses IDTR (not
      // a fixed IVT); ours points at the PM IDT at 0x1A000, so the
      // RM INT would read our 8-byte gate descriptors as 4-byte
      // seg:off pairs and jump to garbage.  Restored below before
      // re-entering PM.
      const Bitu saved_idt_base  = CPU_SIDT_base();
      const Bitu saved_idt_limit = CPU_SIDT_limit();
      CPU_LIDT(0x3FF, 0);

      // Clear CR0.PE.  Now in real mode.
      CPU_SET_CRX(0, saved_cr0 & ~1u);

      // Load RM register + segment context from the struct.  SegSet16
      // unconditionally sets Segs.phys[seg] = val << 4 (RM semantics),
      // which is what we want here.
      reg_eax = s_eax; reg_ebx = s_ebx; reg_ecx = s_ecx; reg_edx = s_edx;
      reg_esi = s_esi; reg_edi = s_edi; reg_ebp = s_ebp;
      SegSet16(ds, s_ds); SegSet16(es, s_es);
      SegSet16(fs, s_fs); SegSet16(gs, s_gs);
      if (s_ss != 0) {
        SegSet16(ss, s_ss);
        reg_esp = s_sp;
      } else {
        // Scratch stack for when client leaves SS:SP zero.
        // Must NOT overlap the MZ image, PSP, or any region the client
        // may touch.  We sit at 0x9000:0xFFxx = linear 0x9FFxx -- the
        // top of conventional memory, below the EBDA/VGA BIOS region
        // but above any typical DOS program's arena end.  DJGPP go32-v2
        // calls AX=0300 with SS=0 for its sim-RM INT 21h, and the
        // ensuing INT pushes FLAGS/CS/IP onto this stack.  A previous
        // choice of 0x0050:0x0F00 = linear 0x1400 landed INSIDE the
        // stub image itself (stub loads at 0x1100-0x18FF), corrupting
        // its code and making .data load never happen.
        SegSet16(ss, 0x9000);
        reg_esp = 0xFFF0;
      }

      CALLBACK_RunRealInt(intnum);

      // If the RM INT terminated a child via AH=4C, don't try to
      // resume PM here -- the child is dead.  Propagate CBRET_STOP
      // so the outer child RunMachine (from AH=4B) unwinds.  AH=4B's
      // restore path will flip CR0.PE=1 via CPU_SetSegGeneral when
      // it reloads the parent's selectors.
      if (s_child_exit_pending) {
        CPU_LIDT(saved_idt_limit, saved_idt_base);
        return CBRET_STOP;
      }

      // Snapshot results before reloading PM state clobbers them.
      const uint32_t r_eax = reg_eax;
      const uint32_t r_ebx = reg_ebx;
      const uint32_t r_ecx = reg_ecx;
      const uint32_t r_edx = reg_edx;
      const uint32_t r_esi = reg_esi;
      const uint32_t r_edi = reg_edi;
      const uint32_t r_ebp = reg_ebp;
      const uint16_t r_flags = static_cast<uint16_t>(reg_flags & 0xFFFF);
      const uint16_t r_es = SegValue(es);
      const uint16_t r_ds = SegValue(ds);
      const uint16_t r_fs = SegValue(fs);
      const uint16_t r_gs = SegValue(gs);

      // Back to protected mode + restore PM IDTR.
      CPU_SET_CRX(0, saved_cr0);
      CPU_LIDT(saved_idt_limit, saved_idt_base);

      // Reload PM selectors so their descriptor caches come from the
      // GDT/LDT again (CALLBACK_RunRealInt left each cached base set
      // as selector*16 -- its RM interpretation).
      CPU_SetSegGeneral(ds, saved_ds);
      CPU_SetSegGeneral(ss, saved_ss);
      CPU_SetSegGeneral(es, saved_es);
      CPU_SetSegGeneral(fs, saved_fs);
      CPU_SetSegGeneral(gs, saved_gs);
      // CS can't be loaded with a plain MOV; manually refresh its
      // cached base from the saved selector's GDT descriptor so the
      // next instruction fetch (of the shim's CF / IRETD) uses the
      // PM base.  The IRET itself will then do a proper CS load with
      // cpu.code.big update via the kernel code path.
      {
        Descriptor cs_desc;
        cpu.gdt.GetDescriptor(saved_cs, cs_desc);
        Segs.val[cs]  = saved_cs;
        Segs.phys[cs] = cs_desc.GetBase();
      }

      // Restore host register file (the PM client hasn't "seen" any
      // of the register changes we made during the RM call).
      reg_eax = saved_eax; reg_ebx = saved_ebx;
      reg_ecx = saved_ecx; reg_edx = saved_edx;
      reg_esi = saved_esi; reg_edi = saved_edi;
      reg_ebp = saved_ebp; reg_esp = saved_esp;

      // Write results back to the struct (SS:SP and CS:IP aren't
      // updated on an INT -- those are call-specific outputs).
      mem_writed(rmcs + 0x00, r_edi);
      mem_writed(rmcs + 0x04, r_esi);
      mem_writed(rmcs + 0x08, r_ebp);
      mem_writed(rmcs + 0x10, r_ebx);
      mem_writed(rmcs + 0x14, r_edx);
      mem_writed(rmcs + 0x18, r_ecx);
      mem_writed(rmcs + 0x1C, r_eax);
      mem_writew(rmcs + 0x20, r_flags);
      mem_writew(rmcs + 0x22, r_es);
      mem_writew(rmcs + 0x24, r_ds);
      mem_writew(rmcs + 0x26, r_fs);
      mem_writew(rmcs + 0x28, r_gs);

      if (dosemu::g_debug.stackwatch) {
        const uint32_t ss_lin = SegPhys(ss) + reg_esp;
        std::fprintf(stderr, "[simrm-exit]  INT %02x client SS:SP=%04x:%08x lin=%08x val=%04x\n",
            (unsigned)intnum, (unsigned)SegValue(ss), (unsigned)reg_esp,
            ss_lin, (unsigned)mem_readw(ss_lin));
      }
      if (dosemu::g_debug.simrm_trace || dosemu::g_debug.int4b_trace) {
        std::fprintf(stderr,
            "[simrm-exit]  INT %02x PM cs:eip=%04x:%08x cr0=%08x cpl=%u ss:esp=%04x:%08x\n",
            (unsigned)intnum,
            (unsigned)SegValue(cs), (unsigned)reg_eip,
            (unsigned)cpu.cr0, (unsigned)cpu.cpl,
            (unsigned)SegValue(ss), (unsigned)reg_esp);
        // Dump the IRETD frame at ring-0 SS:ESP -- this is what the
        // int31 callback's 66 CF will pop into outer EIP/CS/EFLAGS.
        const PhysPt stk_p = SegPhys(ss) + reg_esp;
        std::fprintf(stderr, "[simrm-exit]  INT %02x frame@%08x: EIP=%08x CS=%04x EFLAGS=%08x ESP=%08x SS=%04x\n",
            (unsigned)intnum, (unsigned)stk_p,
            (unsigned)mem_readd(stk_p),
            (unsigned)(mem_readd(stk_p + 4) & 0xFFFF),
            (unsigned)mem_readd(stk_p + 8),
            (unsigned)mem_readd(stk_p + 12),
            (unsigned)(mem_readd(stk_p + 16) & 0xFFFF));
      }
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0503: {  // Resize memory block
      // Input:  SI:DI = handle, BX:CX = new size in bytes.
      // Output: BX:CX = linear address of the (possibly-relocated) block.
      //         SI:DI = new handle.  In-place only (spec allows either).
      const uint32_t new_bytes = (static_cast<uint32_t>(reg_bx) << 16) | reg_cx;
      if (new_bytes == 0) { reg_ax = 0x8021; set_cf(true); return CBRET_NONE; }
      if (reg_si == 0) {
        if (reg_di < MCB_ARENA_START || reg_di >= MCB_ARENA_END) {
          reg_ax = 0x8023; set_cf(true); return CBRET_NONE;
        }
        const uint32_t new_paras32 = (new_bytes + 15u) >> 4;
        if (new_paras32 > 0xFFFFu) { reg_ax = 0x8012; set_cf(true); return CBRET_NONE; }
        uint16_t largest = 0;
        const uint16_t got = mcb_resize(reg_di,
            static_cast<uint16_t>(new_paras32), largest);
        if (got != new_paras32) { reg_ax = 0x8012; set_cf(true); return CBRET_NONE; }
        const uint32_t linear = static_cast<uint32_t>(reg_di) * 16u;
        reg_bx = (linear >> 16) & 0xFFFF;
        reg_cx = linear & 0xFFFF;
        set_cf(false);
        return CBRET_NONE;
      }
      // pm_arena block.
      const uint32_t base = (static_cast<uint32_t>(reg_si) << 16) | reg_di;
      if (!pm_resize(base, new_bytes)) {
        reg_ax = 0x8012; set_cf(true); return CBRET_NONE;
      }
      reg_bx = (base >> 16) & 0xFFFF;
      reg_cx = base & 0xFFFF;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0506: {  // DPMI 1.0: Get Page Attributes
      // Input:  SI:DI = handle, EBX = offset in block, CX = # pages,
      //         ES:EDX = dest buffer (one 16-bit attr per page).
      // Output: ES:EDX filled, CF=0.
      // We don't page, so every page of a live block reads back as
      // "type=1 committed, R/W, accessed, dirty" (0x39).  CWSDPMI
      // returns similar for committed pages.
      const uint32_t n = reg_cx;
      const PhysPt buf = SegPhys(es) + reg_edx;
      for (uint32_t i = 0; i < n; ++i)
        mem_writew(buf + i * 2, 0x0039);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0507: {  // DPMI 1.0: Set Page Attributes
      // Input:  SI:DI = handle, EBX = offset, CX = # pages,
      //         ES:EDX = src buffer.  Client wants to change page
      //         attributes (commit/uncommit/read-only).  We don't
      //         page, so there's nothing to do -- accept the call
      //         and succeed.  A client that expects uncommitted
      //         pages to fault on access won't get that semantic,
      //         but will at least not crash on the API call.
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0508:    // DPMI 1.0: Map Device in Memory Block
    case 0x0509: {  // DPMI 1.0: Map Conventional Memory in Memory Block
      // Input:  SI:DI = handle, EBX = offset, CX = # pages,
      //         EDX = physical or conventional address to map.
      // We don't remap; memory is linearly backed by dosbox's RAM.
      // Return CF=1 AX=0x8025 "invalid address" -- spec-legal.
      reg_ax = 0x8025;
      set_cf(true);
      return CBRET_NONE;
    }

    case 0x0E00: {  // DPMI 1.0: Get Coprocessor Status
      // Output: AX bits:
      //   0   MP bit (coprocessor present)
      //   1   client emulation enabled
      //   2   host emulation active
      //   3   client's initialize bit
      //   4-7 coprocessor type: 2=287, 3=387/later, 4=486DX+
      // dosbox-emulated FPU is present; we don't emulate in the host,
      // no client emulation, coprocessor type 387-class.
      reg_ax = 0x31;      // bits 0 + 4 + 5 = MP + type=3 (387)
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0E01: {  // DPMI 1.0: Set Coprocessor Emulation
      // Input:  BX bits 0..3 (MP / EM / client-emulation / initialize).
      // CWSDPMI mutates IDT[7] depending on whether the client
      // provides an emulator; we don't switch FPU delivery at runtime
      // so accept the call and succeed.
      set_cf(false);
      return CBRET_NONE;
    }

    default:
      reg_ax = 0x8001;
      set_cf(true);
      return CBRET_NONE;
  }
}

// Bitness-aware wrappers: dosbox callbacks can't pass parameters, so
// we encode the gate bitness by registering a distinct C function for
// each and having it toggle s_int_gate_bits32 around the real handler
// call.  set_cf reads s_int_gate_bits32 to pick the right flag offset.
Bitu dosemu_int21();
Bitu dosemu_int31();
Bitu dosemu_int21_bits32() {
  // Save-and-restore pattern: a nested child DPMI call can re-enter
  // these wrappers (via the int21 AH=4B -> child run -> child int31
  // -> dosemu_int31_bits32 path).  The inner wrapper resetting the
  // global to false on exit would flip the outer caller's state,
  // making its later set_cf() pick the wrong flags offset and
  // corrupt the ring-0 IRETD frame at PM_CB_STACK.
  const bool prev = s_int_gate_bits32;
  s_int_gate_bits32 = true;
  Bitu r = dosemu_int21();
  s_int_gate_bits32 = prev;
  return r;
}
Bitu dosemu_int31_bits32() {
  const bool prev = s_int_gate_bits32;
  s_int_gate_bits32 = true;
  Bitu r = dosemu_int31();
  s_int_gate_bits32 = prev;
  return r;
}

Bitu dosemu_int21() {
  if (dosemu::g_debug.trace) {
    // When the client is in 32-bit PM (s_int_gate_bits32) the high
    // halves of EBX/ECX/EDX may carry meaningful data (Watcom's
    // runtime, for instance, calls AH=FFh with partial 32-bit regs).
    // Print the full dword view in PM, low-word only in RM/16-bit.
    if (s_int_gate_bits32) {
      std::fprintf(stderr,
          "[trace] AH=%02x AL=%02x EBX=%08x ECX=%08x EDX=%08x "
          "ESI=%08x EDI=%08x\n",
          reg_ah, reg_al, reg_ebx, reg_ecx, reg_edx, reg_esi, reg_edi);
    } else {
      std::fprintf(stderr,
          "[trace] AH=%02x AL=%02x BX=%04x CX=%04x DX=%04x\n",
          reg_ah, reg_al, reg_bx, reg_cx, reg_dx);
    }
  }
  switch (reg_ah) {

    case 0x01: {  // Read char from stdin, echo to stdout, return in AL.
      uint8_t c;
      if (::read(STDIN_FILENO, &c, 1) <= 0) { reg_al = 0; return CBRET_NONE; }
      std::fputc(c, stdout);
      std::fflush(stdout);
      // DOS reports newlines as CR; the host sends LF.
      reg_al = (c == '\n') ? '\r' : c;
      return CBRET_NONE;
    }

    case 0x07:
    case 0x08: {  // Read char from stdin, no echo, return in AL.
      uint8_t c;
      if (::read(STDIN_FILENO, &c, 1) <= 0) { reg_al = 0; return CBRET_NONE; }
      reg_al = (c == '\n') ? '\r' : c;
      return CBRET_NONE;
    }

    case 0x0B: {  // Check stdin status.  AL=0xFF if data ready, 0 if not.
      fd_set rd;
      FD_ZERO(&rd); FD_SET(STDIN_FILENO, &rd);
      struct timeval zero = {0, 0};
      const int n = ::select(STDIN_FILENO + 1, &rd, nullptr, nullptr, &zero);
      reg_al = (n > 0 && FD_ISSET(STDIN_FILENO, &rd)) ? 0xFF : 0x00;
      return CBRET_NONE;
    }

    case 0x02: {  // Write character in DL to stdout
      std::fputc(reg_dl, stdout);
      std::fflush(stdout);
      return CBRET_NONE;
    }

    case 0x0A: {  // Buffered input.  DS:DX -> [max_len][actual_len][data...]
      const PhysPt buf = SegPhys(ds) + reg_dx;
      const uint8_t max = mem_readb(buf);
      if (max == 0) {
        mem_writeb(buf + 1, 0);
        return CBRET_NONE;
      }
      uint8_t count = 0;
      while (count < static_cast<uint8_t>(max - 1)) {
        uint8_t c;
        if (::read(STDIN_FILENO, &c, 1) <= 0) break;
        const uint8_t stored = (c == '\n') ? '\r' : c;
        mem_writeb(buf + 2 + count, stored);
        std::fputc(c, stdout);
        ++count;
        if (stored == '\r') break;
      }
      std::fflush(stdout);
      mem_writeb(buf + 1, count);
      return CBRET_NONE;
    }

    case 0x09: {  // Write $-terminated string at DS:DX to stdout
      const PhysPt base = SegPhys(ds);
      for (uint16_t off = reg_dx;; ++off) {
        const uint8_t c = mem_readb(base + off);
        if (c == '$') break;
        std::fputc(c, stdout);
      }
      std::fflush(stdout);
      return CBRET_NONE;
    }

    case 0x3C: {  // Create file; CX=attr, DS:DX=path.  Returns handle in AX.
      const std::string dos_path = read_dos_string(SegValue(ds), reg_dx);
      const Resolved    r        = resolve_path(dos_path);
      if (dosemu::g_debug.open_trace) {
        std::fprintf(stderr,
            "[create] CX=%04x path='%s' -> host='%s'\n",
            (unsigned)reg_cx, dos_path.c_str(), r.host_path.c_str());
      }
      int fd = ::open(r.host_path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
      if (fd < 0) {
        if (dosemu::g_debug.open_trace) {
          std::fprintf(stderr, "[create] -> errno=%d (%s)\n", errno, strerror(errno));
        }
        return_error(0x05);
        break;
      }
      int h = allocate_handle(fd, r.text_mode);
      if (h < 0) { ::close(fd); return_error(0x04); break; }
      reg_ax = static_cast<uint16_t>(h);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x3D: {  // Open file; AL=mode, DS:DX=path.  Returns handle in AX.
      std::string dos_path = read_dos_string(SegValue(ds), reg_dx);
      // DJGPP stub quirk on nested exec: the go32-v2 stub clears the
      // RM argv[0] scratch buffer at `[DS:0x764]` (writing 0x00 to
      // byte 0) after switching to PM -- but some PM-side code still
      // reads the path from that offset.  When we see an empty path
      // but bytes 1..N look like ":\..." (the rest of the path
      // survives, just the leading 'C' was zeroed), re-synthesize
      // the full argv[0] from our known child path.  Only kicks in
      // inside a nested process (s_process_stack non-empty) so
      // top-level flows are unaffected.
      if (dos_path.empty() && !s_process_stack.empty()
          && reg_dx != 0 && mem_readb(SegPhys(ds) + reg_dx) == 0
          && mem_readb(SegPhys(ds) + reg_dx + 1) == ':') {
        // Reconstruct from the :\... tail still in memory.  Prepend
        // the current drive letter.
        dos_path = s_current_drive;
        for (uint32_t i = 1; i < 128; ++i) {
          uint8_t b = mem_readb(SegPhys(ds) + reg_dx + i);
          if (b == 0) break;
          dos_path += static_cast<char>(b);
        }
        if (dosemu::g_debug.open_trace) {
          std::fprintf(stderr, "[open] reconstructed path from clobbered "
              "stub buffer: '%s'\n", dos_path.c_str());
        }
      }
      // DOS character-device names resolve by name, not by filesystem
      // path.  Programs probe for EMMXXXX0 (EMS) / XMSXXXX0 (XMS) / etc.
      // via AH=3D open and only want a valid handle that read/write/close
      // cleanly -- they talk to the driver via INT 67h / INT 2F, not
      // through this handle.  Strip drive and path, compare the base
      // name (without extension) to a small reserved set.
      {
        std::string base = dos_path;
        // drop drive letter "X:"
        if (base.size() >= 2 && base[1] == ':')
          base.erase(0, 2);
        // drop leading path components
        auto slash = base.find_last_of("\\/");
        if (slash != std::string::npos) base.erase(0, slash + 1);
        // drop extension (".EXT") for the comparison
        auto dot = base.find('.');
        std::string stem = (dot == std::string::npos) ? base : base.substr(0, dot);
        // case-fold
        for (auto &c : stem) c = static_cast<char>(::toupper(c));
        const bool is_char_dev =
            stem == "EMMXXXX0" ||
            stem == "XMSXXXX0" ||
            stem == "CON"      ||
            stem == "NUL"      ||
            stem == "AUX"      ||
            stem == "PRN"      ||
            stem == "CLOCK$";
        if (is_char_dev) {
          const int devfd = ::open("/dev/null", O_RDWR);
          if (devfd < 0) { return_error(0x02); break; }
          int h = allocate_handle(devfd, false);
          if (h < 0) { ::close(devfd); return_error(0x04); break; }
          reg_ax = static_cast<uint16_t>(h);
          set_cf(false);
          return CBRET_NONE;
        }
      }
      const Resolved r = resolve_path(dos_path);
      if (dosemu::g_debug.open_trace) {
        std::fprintf(stderr,
            "[open] AL=%02x DS:DX=%04x:%04x path='%s' -> host='%s' textmode=%d\n",
            (unsigned)reg_al, (unsigned)SegValue(ds), (unsigned)reg_dx,
            dos_path.c_str(), r.host_path.c_str(), (int)r.text_mode);
      }
      int flags = O_RDONLY;
      switch (reg_al & 0x07) {
        case 0: flags = O_RDONLY; break;
        case 1: flags = O_WRONLY; break;
        case 2: flags = O_RDWR;   break;
      }
      int fd = ::open(r.host_path.c_str(), flags);
      if (fd < 0) { return_error(0x02); break; }
      // DOS AH=3Dh refuses to open directories (returns error 05h
      // access denied).  POSIX open(2) on a dir succeeds and only
      // fails later on read, which confuses callers like GNU find
      // that probe with open+fstat to classify paths.
      struct stat st;
      if (::fstat(fd, &st) == 0 && S_ISDIR(st.st_mode)) {
        ::close(fd);
        return_error(0x05);
        break;
      }
      int h = allocate_handle(fd, r.text_mode);
      if (h < 0) { ::close(fd); return_error(0x04); break; }
      reg_ax = static_cast<uint16_t>(h);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x3E: {  // Close handle in BX.
      // Standard handles 0-4 (stdin/stdout/stderr/aux/prn) close
      // silently without actually freeing the host FD -- real DOS
      // dup's these from DOS internal tables, and a close returns OK
      // but doesn't invalidate the slot.  DJGPP grep (and others) do
      // an atexit close on stdout/stderr; returning EBADF here makes
      // them emit a spurious "write error" diagnostic.
      if (reg_bx <= 4) { set_cf(false); return CBRET_NONE; }
      const auto it = s_handles.find(reg_bx);
      if (it == s_handles.end()) { return_error(0x06); break; }
      ::close(it->second.fd);
      s_handles.erase(it);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x3F: {  // Read from handle BX, CX bytes, into DS:DX.
      // Addressing width tracks the caller's mode:
      //   - 32-bit PM client (direct INT 21 from flat PM): DS:EDX /
      //     ECX bytes (full 32-bit offset + count).
      //   - Sim-RM INT 21 (client set up RMCS with DX/CX): we're
      //     invoked in RM, so DX/CX are the canonical values and
      //     high halves of EDX/ECX may be stale.
      //   - 16-bit PM: same as RM for our purposes (offset fits in
      //     16 bits, count fits in 16 bits).
      // DJGPP's cpp.exe uses the first form with a 32-bit offset
      // whose low-16 bits are frequently zero; reg_dx alone silently
      // wrote to DS:0 (= start of cpp's code segment) and corrupted
      // the running program.
      const bool pm32 = cpu.pmode && s_int_gate_bits32;
      const uint32_t off32 = pm32 ? reg_edx : reg_dx;
      const uint32_t count = pm32 ? reg_ecx : reg_cx;
      int  fd        = -1;
      bool text_mode = false;
      int *pending   = nullptr;
      static int stdin_pending = -1;
      if (reg_bx == 0) {
        fd = STDIN_FILENO;
        // Stdin is treated as text iff the user asked for text mode
        // globally -- there's no filename to run extension heuristics on.
        text_mode = (s_default_mode == FileMode::Text);
        pending = &stdin_pending;
      } else {
        auto it = s_handles.find(reg_bx);
        if (it == s_handles.end()) { return_error(0x06); break; }
        fd = it->second.fd;
        // Real DOS does NOT do CR/LF translation on disk-file reads --
        // that's a C-runtime concern layered on top of AH=3F.  Only
        // device reads (stdin) do text-mode cooking at the DOS level.
        // Expanding \n -> \r\n here inflates the byte count the caller
        // sees, which breaks programs that seek relative to EOF
        // (e.g., DJGPP diff does lseek(-filesize, SEEK_CUR) and gets
        // EINVAL if the file looks bigger than it is on disk).
        text_mode = false;
        pending = &it->second.read_pending;
      }
      const PhysPt dst = SegPhys(ds) + off32;
      uint32_t out = 0;
      while (out < count) {
        if (text_mode && *pending >= 0) {
          mem_writeb(dst + out++, static_cast<uint8_t>(*pending));
          *pending = -1;
          continue;
        }
        uint8_t byte;
        ssize_t n = ::read(fd, &byte, 1);
        if (n < 0)  { return_error(0x05); set_cf(true); reg_ax = out; return CBRET_NONE; }
        if (n == 0) break;  // EOF
        if (text_mode && byte == 0x1A) break;    // DOS text-EOF marker
        if (text_mode && byte == '\n') {
          mem_writeb(dst + out++, '\r');
          if (out < count) mem_writeb(dst + out++, '\n');
          else             *pending = '\n';
        } else {
          mem_writeb(dst + out++, byte);
        }
      }
      if (pm32) reg_eax = out;
      else      reg_ax  = static_cast<uint16_t>(out);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x40: {  // Write to handle BX, CX bytes, from DS:DX.
      int  fd        = -1;
      bool text_mode = false;
      if      (reg_bx == 1) { fd = STDOUT_FILENO; }
      else if (reg_bx == 2) { fd = STDERR_FILENO; }
      else {
        auto it = s_handles.find(reg_bx);
        if (it == s_handles.end()) { return_error(0x06); break; }
        fd = it->second.fd;
        text_mode = it->second.text_mode;
      }
      // 32-bit PM: use full EDX/ECX.  RM/16-bit PM: DX/CX (high
      // halves may be stale).  Same mode-aware select as AH=3F.
      const bool pm32 = cpu.pmode && s_int_gate_bits32;
      const uint32_t count = pm32 ? reg_ecx : reg_cx;
      const uint32_t off32 = pm32 ? reg_edx : reg_dx;
      std::vector<uint8_t> buf(count);
      const PhysPt src = SegPhys(ds) + off32;
      for (uint32_t i = 0; i < count; ++i) buf[i] = mem_readb(src + i);
      if (dosemu::g_debug.write_trace) {
        std::fprintf(stderr,
            "[write] fd=%d count=%u ds=%04x(b=%08x) off=%08x bytes:",
            (int)reg_bx, (unsigned)count,
            (unsigned)SegValue(ds), (unsigned)SegPhys(ds),
            (unsigned)off32);
        for (uint32_t i = 0; i < count && i < 40; ++i)
          std::fprintf(stderr, " %02x", buf[i]);
        std::fprintf(stderr, "\n");
      }
      if (text_mode) {
        // DOS sends CR LF for newlines.  Host wants Unix-style LF.  Strip
        // CR bytes entirely -- lone CRs are rare and better dropped than
        // preserved (CRLF -> LF is the dominant case).
        size_t w = 0;
        for (size_t i = 0; i < buf.size(); ++i)
          if (buf[i] != '\r') buf[w++] = buf[i];
        buf.resize(w);
      }
      ssize_t n = ::write(fd, buf.data(), buf.size());
      if (n < 0) { return_error(0x05); break; }
      // Report the caller's full byte count as consumed even when text-mode
      // filtering compressed the output -- that's what DOS programs expect.
      if (pm32) reg_eax = count;
      else      reg_ax  = static_cast<uint16_t>(count);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x42: {  // Seek handle BX; CX:DX = offset, AL = whence (0,1,2).
      int host_fd;
      HostHandle *hi = nullptr;
      if (reg_bx <= 4) {
        // Standard handles: GNU cat does lseek(1, 0, SEEK_CUR) to
        // decide its output strategy.  On a pipe/tty host lseek
        // fails with ESPIPE; pretend success with pos=0 -- the
        // program already got char-device info from AH=44 so it
        // knows not to treat this as a real seek.
        host_fd = static_cast<int>(reg_bx);
      } else {
        auto it = s_handles.find(reg_bx);
        if (it == s_handles.end()) { return_error(0x06); break; }
        host_fd = it->second.fd;
        hi = &it->second;
      }
      int whence = SEEK_SET;
      if (reg_al == 1) whence = SEEK_CUR;
      if (reg_al == 2) whence = SEEK_END;
      // CX:DX is a 32-bit signed offset (high:low).  Cast through
      // int32_t to get correct sign extension to off_t.
      off_t off = static_cast<int32_t>(
          (static_cast<uint32_t>(reg_cx) << 16) | reg_dx);
      if (dosemu::g_debug.open_trace) {
        std::fprintf(stderr, "[seek] fd=%d whence=%d off=%lld (from CX:DX=%04x:%04x)\n",
            (int)reg_bx, whence, (long long)off,
            (unsigned)reg_cx, (unsigned)reg_dx);
      }
      off_t pos = ::lseek(host_fd, off, whence);
      if (pos < 0) {
        if (reg_bx <= 4) {
          pos = 0;  // non-seekable stdio: report position 0, success
        } else {
          if (dosemu::g_debug.open_trace) {
            std::fprintf(stderr, "[seek] -> errno=%d (%s)\n", errno, strerror(errno));
          }
          return_error(0x19);
          break;
        }
      }
      reg_ax = static_cast<uint16_t>(pos & 0xFFFF);
      reg_dx = static_cast<uint16_t>((pos >> 16) & 0xFFFF);
      if (hi) hi->read_pending = -1;  // invalidate text-mode read buffer
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x1A: {  // Set DTA to DS:DX
      s_dta_linear = SegPhys(ds) + reg_dx;
      return CBRET_NONE;
    }

    case 0x4E: {  // Find first: CX=attr mask, DS:DX=ASCIIZ path/pattern
      const std::string dos_path = read_dos_string(SegValue(ds), reg_dx);
      auto [dir_part, pat_part]  = split_dir_pattern(dos_path);

      const std::string host_dir = dir_part.empty()
          ? dos_to_host("") : dos_to_host(dir_part);
      FindState st;
      st.token = s_next_find_token++;
      if (!build_find_state(host_dir, upper(pat_part), st)) {
        return_error(0x03); break;                   // path not found
      }
      if (!scan_next(st)) { return_error(0x12); break; }
      const uint32_t tok = st.token;
      s_finds[tok] = std::move(st);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x4F: {  // Find next: continues the DTA's saved state
      // The search state key lives in the DTA itself (bytes 4..7
      // after our "DSFN" magic at 0..3), not at the DTA pointer.
      // Programs that swap DTAs between findfirst and findnext
      // (GNU find) rely on this.
      uint32_t tok = 0;
      if (mem_readb(s_dta_linear + 0) == 'D' &&
          mem_readb(s_dta_linear + 1) == 'S' &&
          mem_readb(s_dta_linear + 2) == 'F' &&
          mem_readb(s_dta_linear + 3) == 'N') {
        tok  = mem_readb(s_dta_linear + 4);
        tok |= mem_readb(s_dta_linear + 5) <<  8;
        tok |= mem_readb(s_dta_linear + 6) << 16;
        tok |= mem_readb(s_dta_linear + 7) << 24;
      }
      auto it = s_finds.find(tok);
      if (tok == 0 || it == s_finds.end()) {
        return_error(0x12);
        break;
      }
      if (!scan_next(it->second)) {
        s_finds.erase(it);
        return_error(0x12);
        break;
      }
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0E: {  // Set default drive.  DL = 0 for A, 1 for B, ...
      const char drive = 'A' + reg_dl;
      if (s_drives.find(drive) != s_drives.end()) s_current_drive = drive;
      reg_al = static_cast<uint8_t>(s_drives.size());   // number of drives
      return CBRET_NONE;
    }

    case 0x19: {  // Get default drive -> AL
      reg_al = static_cast<uint8_t>(s_current_drive - 'A');
      return CBRET_NONE;
    }

    case 0x25: {  // Set interrupt vector: AL=vector, DS:DX=handler far ptr
      const PhysPt ivt = reg_al * 4u;
      mem_writeb(ivt,     static_cast<uint8_t>(reg_dx & 0xFF));
      mem_writeb(ivt + 1, static_cast<uint8_t>((reg_dx >> 8) & 0xFF));
      mem_writeb(ivt + 2, static_cast<uint8_t>(SegValue(ds) & 0xFF));
      mem_writeb(ivt + 3, static_cast<uint8_t>((SegValue(ds) >> 8) & 0xFF));
      return CBRET_NONE;
    }

    case 0x29: {  // Parse filename at DS:SI into FCB at ES:DI.  Minimal
                  // implementation: report "no valid filename" and advance
                  // SI past whitespace so callers don't loop.
      while (true) {
        const uint8_t c = mem_readb(SegPhys(ds) + reg_si);
        if (c != ' ' && c != '\t') break;
        ++reg_si;
      }
      reg_al = 0xFF;  // bad-filename code; callers treat as "no more args"
      return CBRET_NONE;
    }

    case 0x33: {  // Get/set ctrl-break checking: AL=0 get, AL=1 set; DL
      static uint8_t ctrl_break_on = 0;
      if      (reg_al == 0) reg_dl = ctrl_break_on;
      else if (reg_al == 1) ctrl_break_on = reg_dl;
      // AL=5 (boot drive), AL=6 (DOS version) sub-functions left unhandled.
      return CBRET_NONE;
    }

    case 0x36: {  // Get disk free space: DL=drive (0=default, 1=A...)
                  // Returns: AX=sec/cluster, BX=free clusters, CX=bytes/sec,
                  // DX=total clusters.  Report a generous ~500MB.
      const char drive = (reg_dl == 0) ? s_current_drive
                                       : static_cast<char>('A' + reg_dl - 1);
      if (s_drives.find(drive) == s_drives.end()) {
        reg_ax = 0xFFFF;                // invalid drive
        return CBRET_NONE;
      }
      reg_ax = 32;      // sectors per cluster
      reg_bx = 32768;   // available clusters
      reg_cx = 512;     // bytes per sector
      reg_dx = 32768;   // total clusters  -> 32*32768*512 = 512 MiB
      return CBRET_NONE;
    }

    case 0x37: {  // Get/set switchar (the / vs - convention).  Keep it
                  // as '/' so DOS programs parse their own CLI sensibly.
      static uint8_t switchar = '/';
      if      (reg_al == 0) { reg_dl = switchar; reg_al = 0; }
      else if (reg_al == 1) { switchar = reg_dl; reg_al = 0; }
      return CBRET_NONE;
    }

    case 0x50: {  // Set current PSP to BX.  We only ever have one process,
                  // so just accept it silently -- some C runtimes call
                  // this to register their own PSP.
      return CBRET_NONE;
    }

    case 0x51:
    case 0x62: {  // Get current PSP -> BX
      reg_bx = PSP_SEG;
      return CBRET_NONE;
    }


    case 0x5D: {  // Network / file-locking sub-functions; report "not
                  // supported" so programs fall back to normal file ops.
      return_error(0x01);
      break;
    }

    case 0x60: {  // Qualify/canonicalize filename.
      // Input : DS:SI = source ASCIIZ filename
      //         ES:DI = 128-byte output buffer
      // Output: AX=0 success; ES:DI filled with canonicalized path
      //         (drive letter + absolute path with backslashes, upper
      //         case, ASCIIZ).  CF=1 + AX=0x02 on not-found.
      //
      // DJGPP's libc (_truename, realpath, glob) uses this heavily.
      // Returning "invalid function" (AL=01) as the default unknown
      // handler does made DJGPP libc leave internal state inconsistent
      // and later fd ops returned EBADF spuriously.  We implement a
      // minimal pass-through: resolve relative paths against cwd,
      // prepend the current drive, uppercase, write to ES:DI.
      const std::string src = read_dos_string(SegValue(ds), reg_si);
      // Split off drive letter if any.
      std::string path = src;
      char drive = s_current_drive;
      if (path.size() >= 2 && path[1] == ':') {
        drive = static_cast<char>(std::toupper(
                    static_cast<unsigned char>(path[0])));
        path.erase(0, 2);
      }
      // If not absolute (no leading slash/backslash), prepend cwd.
      if (path.empty() || (path[0] != '\\' && path[0] != '/')) {
        const std::string &cwd = s_drive_cwd[drive]; // "" if unset
        if (!cwd.empty() && cwd.front() != '\\' && cwd.front() != '/')
          path = "\\" + cwd + "\\" + path;
        else
          path = (cwd.empty() ? "\\" : cwd + "\\") + path;
      }
      // Normalize slashes + uppercase.
      for (auto &c : path) {
        if (c == '/') c = '\\';
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
      }
      std::string canon;
      canon += drive;
      canon += ':';
      canon += path;
      if (canon.size() > 127) canon.resize(127);   // leave room for NUL
      const PhysPt dst = SegPhys(es) + reg_di;
      for (size_t i = 0; i < canon.size(); ++i)
        mem_writeb(dst + i, static_cast<uint8_t>(canon[i]));
      mem_writeb(dst + canon.size(), 0);
      reg_ax = 0;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x65: {  // Get extended country information.
      // Input : AL=subfunction, BX=codepage (-1=current), DX=country
      //         (-1=current), ES:DI=buffer, CX=buffer size
      // Output: on success CX=info len, buffer filled.
      // Our minimal implementation reports US / codepage 437 /
      // "C"-locale-equivalent.  DJGPP libc uses this in setlocale
      // and various isalpha/toupper tables; FreeCOM uses AL=05
      // (filename-character table) to populate its is_fnchar() --
      // without which every character is treated as non-filename
      // and internal commands can't be parsed.
      const uint16_t cx_in = reg_cx;
      const PhysPt dst = SegPhys(es) + reg_di;
      uint8_t buf[64] = {0};
      size_t n = 0;
      if (reg_al == 0x05) {
        // Filename character table: buf[0]=id, buf[1..4]=FAR pointer
        // to the table.  FreeCOM reads the 10-byte header via that
        // pointer, then treats pointer+10 as the illegal-char list.
        // We park the table at a fixed offset (0x0060:0x0010 =
        // linear 0x610) just past the SFT stub block at 0x600.
        constexpr uint32_t nls_tab = 0x610;
        // Table layout (10-byte header + 13 illegal chars):
        //   [0..1] = reserved (size/pad)
        //   [2]    = reserved
        //   [3]    = inclFirst (lowest allowed char)
        //   [4]    = inclLast  (highest allowed char)
        //   [5]    = reserved
        //   [6]    = exclFirst (lead-byte exclude range start)
        //   [7]    = exclLast  (lead-byte exclude range end, empty)
        //   [8]    = reserved
        //   [9]    = illegalLen (count of bytes that follow)
        //   [10..] = illegal chars
        static const uint8_t tbl[10] = {
            0x17, 0x00,   // size hint
            0x00,
            0x21,         // inclFirst (first printable ASCII char, '!')
            0xFF,         // inclLast
            0x00,
            0x80,         // exclFirst
            0x80,         // exclLast (empty exclude range)
            0x00,
            13,           // illegalLen
        };
        static const char illegal[] = ".\"/\\[]:|<>+=;,";
        for (size_t i = 0; i < sizeof(tbl); ++i)
          mem_writeb(nls_tab + i, tbl[i]);
        for (size_t i = 0; i < 13; ++i)
          mem_writeb(nls_tab + 10 + i, static_cast<uint8_t>(illegal[i]));
        buf[0] = 0x05;
        buf[1] = 0x10;   // offset lo
        buf[2] = 0x00;   // offset hi
        buf[3] = 0x60;   // segment lo
        buf[4] = 0x00;   // segment hi
        n = 5;
      } else {
        // Default (subfunctions 1 etc.): 41-byte extended country info.
        buf[0] = 0x01;                                 // info id
        buf[1] = 41 & 0xFF;
        buf[2] = 0;
        buf[3] = 0xB5; buf[4] = 0x01;                  // codepage = 437
        buf[5] = 0x01; buf[6] = 0x00;                  // country = 1 (USA)
        buf[7] = 0;                                    // date format = US m/d/y
        buf[8] = '$'; buf[9] = 0;                      // currency
        buf[14] = ','; buf[16] = '.';                  // thousands/decimal
        buf[30] = ':'; buf[32] = '/';                  // time / date sep
        n = 41;
      }
      n = std::min<size_t>(cx_in, n);
      for (size_t i = 0; i < n; ++i)
        mem_writeb(dst + i, buf[i]);
      reg_cx = static_cast<uint16_t>(n);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x63: {  // Get lead-byte table (DBCS).  Report "no DBCS" via a
                  // NUL pointer.
      SegSet16(ds, 0);
      reg_si = 0;
      reg_al = 0;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x2A: {  // Get date: AL=dow (0=Sun), CX=year, DH=month, DL=day
      const time_t t = std::time(nullptr);
      struct tm lt;
      localtime_r(&t, &lt);
      reg_al = static_cast<uint8_t>(lt.tm_wday);
      reg_cx = static_cast<uint16_t>(lt.tm_year + 1900);
      reg_dh = static_cast<uint8_t>(lt.tm_mon + 1);
      reg_dl = static_cast<uint8_t>(lt.tm_mday);
      return CBRET_NONE;
    }

    case 0x2C: {  // Get time: CH=hour, CL=min, DH=sec, DL=hundredths
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      struct tm lt;
      localtime_r(&ts.tv_sec, &lt);
      reg_ch = static_cast<uint8_t>(lt.tm_hour);
      reg_cl = static_cast<uint8_t>(lt.tm_min);
      reg_dh = static_cast<uint8_t>(lt.tm_sec);
      reg_dl = static_cast<uint8_t>(ts.tv_nsec / 10000000);  // nanoseconds -> hundredths
      return CBRET_NONE;
    }

    case 0x56: {  // rename: DS:DX = old name, ES:DI = new name
      const std::string old_dos = read_dos_string(SegValue(ds), reg_dx);
      const std::string new_dos = read_dos_string(SegValue(es), reg_di);
      const std::string old_h   = dos_to_host(old_dos);
      const std::string new_h   = dos_to_host(new_dos);
      if (::rename(old_h.c_str(), new_h.c_str()) < 0) {
        return_error(errno == ENOENT ? 0x02 : 0x05);
        break;
      }
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x57: {  // Get/Set File Date/Time; AL=0 get, AL=1 set, BX=fd.
      // AL=0 returns CX=packed time, DX=packed date for the host fstat
      // of fd.  AL=1 takes CX/DX and applies via utimensat; we accept
      // it silently.  DJGPP's fstat() calls this as part of its
      // non-SFT fallback to fill in st_mtime.
      if (reg_al == 0x00) {
        // Standard handles (stdin/stdout/stderr) aren't in s_handles
        // but programs like GNU cat fstat() them to classify stdout
        // as regular-file vs char-device vs pipe.  Return "now" so
        // they don't spuriously fail with EBADF.
        struct stat st;
        int host_fd;
        if (reg_bx <= 4) {
          host_fd = static_cast<int>(reg_bx);
        } else {
          auto it = s_handles.find(reg_bx);
          if (it == s_handles.end()) { return_error(0x06); break; }
          host_fd = it->second.fd;
        }
        if (::fstat(host_fd, &st) < 0) { return_error(0x05); break; }
        struct tm *tm = ::localtime(&st.st_mtime);
        if (!tm) { return_error(0x05); break; }
        // DOS packed time: bits 15-11 hour, 10-5 min, 4-0 sec/2.
        reg_cx = static_cast<uint16_t>((tm->tm_hour << 11)
                                      | (tm->tm_min  << 5)
                                      | (tm->tm_sec  >> 1));
        // DOS packed date: bits 15-9 year-1980, 8-5 month, 4-0 day.
        const int yr = tm->tm_year + 1900 - 1980;
        reg_dx = static_cast<uint16_t>((std::max(0, std::min(yr, 127)) << 9)
                                      | ((tm->tm_mon + 1) << 5)
                                      |  tm->tm_mday);
        set_cf(false);
        return CBRET_NONE;
      }
      if (reg_al == 0x01) {
        // Accept the set silently (no-op); programs that care just
        // need CF=0.
        set_cf(false);
        return CBRET_NONE;
      }
      return_error(0x01);
      break;
    }

    case 0x30: {  // Get DOS version.  Report 6.22, no OEM info.
      reg_al = 6;
      reg_ah = 22;
      reg_bh = 0;
      reg_bl = 0;
      reg_cx = 0;
      return CBRET_NONE;
    }

    case 0x35: {  // Get interrupt vector: AL=vector -> ES:BX = handler
      const PhysPt ivt = reg_al * 4u;
      const uint16_t off = mem_readb(ivt)     | (mem_readb(ivt + 1) << 8);
      const uint16_t seg = mem_readb(ivt + 2) | (mem_readb(ivt + 3) << 8);
      reg_bx = off;
      SegSet16(es, seg);
      return CBRET_NONE;
    }

    case 0x39: {  // mkdir: DS:DX = path
      const std::string dos_path  = read_dos_string(SegValue(ds), reg_dx);
      const std::string host_path = dos_to_host(dos_path);
      if (::mkdir(host_path.c_str(), 0755) < 0) { return_error(0x03); break; }
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x3A: {  // rmdir: DS:DX = path
      const std::string dos_path  = read_dos_string(SegValue(ds), reg_dx);
      const std::string host_path = dos_to_host(dos_path);
      if (::rmdir(host_path.c_str()) < 0) { return_error(0x10); break; }  // current dir
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x41: {  // unlink: DS:DX = path
      const std::string dos_path  = read_dos_string(SegValue(ds), reg_dx);
      const std::string host_path = dos_to_host(dos_path);
      if (::unlink(host_path.c_str()) < 0) { return_error(0x02); break; }
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x3B: {  // Change directory: DS:DX = path
      const std::string dos_path  = read_dos_string(SegValue(ds), reg_dx);
      const std::string host_path = dos_to_host(dos_path);
      struct stat st;
      if (stat(host_path.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) {
        return_error(0x03);  // path not found
        break;
      }
      // Track cwd for subsequent AH=47 queries.  Strip drive letter from
      // DOS path so we store just the path-within-drive.
      std::string rel = dos_path;
      if (rel.size() >= 2 && rel[1] == ':') rel = rel.substr(2);
      std::replace(rel.begin(), rel.end(), '/', '\\');
      if (!rel.empty() && rel.front() == '\\') rel.erase(0, 1);
      s_drive_cwd[s_current_drive] = rel;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x38: {  // Get/set country info.  AL=0 DS:DX -> 32-byte buffer.
                  // Report US defaults; real programs use this for date,
                  // currency, and list-separator formatting.
      if (reg_al != 0) { return_error(0x01); break; }
      const PhysPt buf = SegPhys(ds) + reg_dx;
      for (int i = 0; i < 32; ++i) mem_writeb(buf + i, 0);
      mem_writew(buf + 0, 0);       // date format MM/DD/YY
      mem_writeb(buf + 2, '$');     // currency symbol "$"
      mem_writeb(buf + 7, ',');     // thousands separator
      mem_writeb(buf + 9, '.');     // decimal separator
      mem_writeb(buf + 11, '/');    // date separator
      mem_writeb(buf + 13, ':');    // time separator
      mem_writeb(buf + 15, 0);      // currency format: symbol prefix, no space
      mem_writeb(buf + 16, 2);      // currency decimal digits
      mem_writeb(buf + 17, 0);      // time format: 12-hour
      reg_bx = 1;                   // country code 1 = US
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x43: {  // Get/set file attributes.  DS:DX = path, AL: 0 get, 1 set.
      const std::string dos_path  = read_dos_string(SegValue(ds), reg_dx);
      const std::string host_path = dos_to_host(dos_path);
      if (reg_al == 0) {
        struct stat st;
        if (::stat(host_path.c_str(), &st) != 0) { return_error(0x02); break; }
        uint16_t attr = 0;
        if (S_ISDIR(st.st_mode))         attr |= 0x10;   // directory
        if (!(st.st_mode & S_IWUSR))     attr |= 0x01;   // read-only
        reg_cx = attr;
        set_cf(false);
        return CBRET_NONE;
      }
      if (reg_al == 1) {
        // Set attributes: honour read-only bit, ignore hidden/system/archive.
        struct stat st;
        if (::stat(host_path.c_str(), &st) == 0) {
          mode_t m = st.st_mode;
          if (reg_cx & 0x01) m &= ~(S_IWUSR | S_IWGRP | S_IWOTH);
          else               m |= S_IWUSR;
          ::chmod(host_path.c_str(), m);
        }
        set_cf(false);
        return CBRET_NONE;
      }
      return_error(0x01);
      break;
    }

    case 0x44: {  // IOCTL.
      if (dosemu::g_debug.open_trace) {
        std::fprintf(stderr, "[ioctl] AL=%02x BX=%04x CX=%04x DX=%04x\n",
            (unsigned)reg_al, (unsigned)reg_bx, (unsigned)reg_cx, (unsigned)reg_dx);
      }
      if (reg_al == 0x00) {
        // Get device info: DX = attribute bits.
        //   bit 0: is stdin         bit 6: "not at EOF" (more input)
        //   bit 1: is stdout        bit 7: character device (vs file)
        //   bit 2: is NUL device    bits 8-15: reserved
        // Bit 6 matters: FreeCOM's interactive-input decision is
        // `(attr & 0xc0) == 0x80` -- char device at EOF -> exit.
        // We leave bit 6 set so interactive shells don't early-bail.
        if (reg_bx == 0 || reg_bx == 1 || reg_bx == 2) {
          reg_dx = 0xC0 | (reg_bx & 0x07); // char device, not at EOF
        } else {
          auto it = s_handles.find(reg_bx);
          if (it == s_handles.end()) {
            // Unknown handle -> EBADF equivalent.  DJGPP's libc checks
            // CF after AH=44 AL=0 and translates 0x06 to EBADF.
            return_error(0x06);
            break;
          }
          reg_dx = 0; // regular file
        }
        set_cf(false);
        return CBRET_NONE;
      }
      return_error(0x01);  // invalid function
      break;
    }

    case 0x45: {  // Duplicate handle: BX = source handle -> AX = new handle
      // Standard handles dup to a real host fd via dup(2); otherwise
      // walk s_handles.  GNU flex uses this (via its stdio redirect)
      // to plumb its output file onto stdout before writing the
      // generated scanner.
      int host_fd;
      bool text_mode = false;
      if (reg_bx <= 4) {
        host_fd = static_cast<int>(reg_bx);
      } else {
        auto it = s_handles.find(reg_bx);
        if (it == s_handles.end()) { return_error(0x06); break; }
        host_fd = it->second.fd;
        text_mode = it->second.text_mode;
      }
      int new_fd = ::dup(host_fd);
      if (new_fd < 0) { return_error(0x04); break; }
      int h = allocate_handle(new_fd, text_mode);
      if (h < 0) { ::close(new_fd); return_error(0x04); break; }
      reg_ax = static_cast<uint16_t>(h);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x46: {  // Force duplicate: BX = source, CX = target; close CX if open
      // This is POSIX dup2.  Flex does dup2(output_fd, 1) to send
      // its generated scanner to stdout, then writes via AH=40 BX=1.
      // Implement by dup2()'ing on the host side and updating the
      // s_handles map so the guest's CX handle now points at the
      // new host fd.
      int src_fd;
      bool src_text = false;
      if (reg_bx <= 4) {
        src_fd = static_cast<int>(reg_bx);
      } else {
        auto it = s_handles.find(reg_bx);
        if (it == s_handles.end()) { return_error(0x06); break; }
        src_fd = it->second.fd;
        src_text = it->second.text_mode;
      }
      if (reg_cx <= 4) {
        // Force target onto the real std-handle fd.  After this,
        // AH=40 writes to BX=CX (e.g. 1) hit src_fd's host file.
        if (::dup2(src_fd, static_cast<int>(reg_cx)) < 0) {
          return_error(0x04); break;
        }
      } else {
        // Target is a regular DOS handle.  Close whatever was there,
        // then dup onto a fresh host fd and install it under CX.
        auto dst = s_handles.find(reg_cx);
        if (dst != s_handles.end()) {
          ::close(dst->second.fd);
          s_handles.erase(dst);
        }
        int new_fd = ::dup(src_fd);
        if (new_fd < 0) { return_error(0x04); break; }
        s_handles[reg_cx] = HostHandle{new_fd, src_text, -1};
      }
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x47: {  // Get current directory: DL = drive (0=current), DS:SI = 64-byte buf
      char drive = s_current_drive;
      if (reg_dl != 0) drive = 'A' + (reg_dl - 1);
      const std::string &cwd = s_drive_cwd[drive];   // "" if not set
      const PhysPt dst = SegPhys(ds) + reg_si;
      for (size_t i = 0; i < cwd.size() && i < 63; ++i)
        mem_writeb(dst + i, static_cast<uint8_t>(cwd[i]));
      mem_writeb(dst + std::min(cwd.size(), size_t{63}), 0);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x48: {  // Allocate BX paragraphs -> AX=segment, CF=1 if fail (BX=max)
      uint16_t largest = 0;
      const uint16_t seg = mcb_allocate(reg_bx, largest);
      if (seg == 0) {
        reg_ax = 0x08; reg_bx = largest; set_cf(true);
        return CBRET_NONE;
      }
      reg_ax = seg; set_cf(false);
      return CBRET_NONE;
    }

    case 0x49: {  // Free memory block at ES
      if (!mcb_free(SegValue(es))) { return_error(0x09); break; }  // bad MCB
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x4A: {  // Resize ES block to BX paragraphs
      // Real DOS programs start by shrinking their "own" block (the one
      // DOS implicitly allocated for the loaded .EXE/.COM) to free memory
      // for the heap.  That block isn't in our MCB chain -- the chain
      // only covers the arena above the program image.  For blocks our
      // chain doesn't know about, accept the resize as a no-op success
      // so programs can proceed to AH=48h.
      if (SegValue(es) < MCB_ARENA_START) {
        set_cf(false);
        return CBRET_NONE;
      }
      uint16_t largest = 0;
      const uint16_t got = mcb_resize(SegValue(es), reg_bx, largest);
      if (got == reg_bx) { set_cf(false); return CBRET_NONE; }
      reg_ax = 0x08; reg_bx = largest; set_cf(true);
      return CBRET_NONE;
    }

    case 0x4C: {  // Exit with code AL
      // Nested under AH=4Bh: record the exit code in the top of the
      // process stack and unwind the nested RunMachine so the parent's
      // AH=4Bh handler resumes and restores its state.  Top-level exit
      // halts the emulator.
      if (dosemu::g_debug.int4c_trace) {
        std::fprintf(stderr,
            "[4C] reg_al=%02x process_stack.size=%zu cs:eip=%04x:%08x\n",
            (unsigned)reg_al, s_process_stack.size(),
            (unsigned)SegValue(cs), (unsigned)reg_eip);
      }
      if (!s_process_stack.empty()) {
        s_process_stack.back().child_exit_code = reg_al;
        s_child_exit_pending = true;
        return CBRET_STOP;
      }
      s_exit_code = reg_al;
      shutdown_requested = true;
      return CBRET_STOP;
    }

    case 0x4B: {  // Load and Execute Program
      // AL=0: load + execute (full nested child run).
      // AL=1: load only, return initial regs in param block (debuggers).
      // AL=3: load overlay -- caller-specified load segment, no reloc.
      // AL=5: set execution state (DOS 5+).  Used by TSRs and memory
      //       managers to arrange a deferred-entry point; DOS's own
      //       IO.SYS is effectively the only caller in the wild.
      //       Return success without actually switching state -- no
      //       program we've met cares about the state actually being
      //       installed, only about AL=5 not returning "invalid
      //       function".
      if (reg_al == 5) {
        set_cf(false);
        reg_ax = 0;
        return CBRET_NONE;
      }
      if (reg_al != 0 && reg_al != 1 && reg_al != 3) {
        return_error(1);
        break;
      }
      const std::string dos_path = read_dos_string(SegValue(ds), reg_dx);
      const Resolved    r        = resolve_path(dos_path);
      if (dosemu::g_debug.int4b_trace) {
        std::fprintf(stderr, "[4B] dos='%s' -> host='%s' exists=%d\n",
            dos_path.c_str(), r.host_path.c_str(),
            ::access(r.host_path.c_str(), F_OK) == 0);
      }
      if (::access(r.host_path.c_str(), F_OK) != 0) {
        return_error(2);   // file not found
        break;
      }

      // AL=3 Load Overlay: caller provides load segment at
      // [ES:BX + 0], relocation factor at [ES:BX + 2].  We load the
      // raw image there (COM: bytes as-is; EXE: strip header + apply
      // reloc-factor to the relocation table entries) and return to
      // caller -- overlay entrypoints are caller's problem.
      if (reg_al == 3) {
        const PhysPt pb = SegPhys(es) + reg_bx;
        const uint16_t load_seg = mem_readw(pb + 0);
        const uint16_t reloc    = mem_readw(pb + 2);
        // Re-use load_program_at at the caller's load_seg.  The
        // loader's relocation logic uses load_seg+reloc for each
        // fixup target, but since we pass load_seg directly it
        // matches caller intent.  reloc is ignored for .COM.
        (void)reloc;
        InitialRegs discarded;
        if (!load_program_at(r.host_path, load_seg, discarded)) {
          return_error(2);
          break;
        }
        set_cf(false);
        return CBRET_NONE;
      }

      // Reserve 64KB for the child (0x1000 paragraphs), leaving room
      // for PSP (0x10 paras), code, data, and stack.  Real DOS hands
      // the child the largest free block (standard DOS behaviour --
      // a real "exec" hands the spawned program all remaining
      // conventional memory; the child then resizes down via AH=4A
      // to whatever it actually needs).  The old 64KB hardcode was
      // fine for our own tiny child fixtures but cripples real
      // DOS extenders like DOS/4GW that grow their allocation
      // during init.
      if (!s_mcb_initialised) mcb_init();
      uint16_t largest = 0;
      // Give the child enough conventional memory for the most
      // demanding single-program case we've seen (DOS/4GW's runtime
      // grows its initial block to 0x1236 paragraphs during init),
      // with a cap so nested AH=4B spawns still have room.  DOS
      // traditionally hands the child ALL remaining memory, but then
      // expects the child to AH=4A-shrink itself before spawning
      // grandchildren; the cap here lets less-well-behaved fixtures
      // nest without requiring an explicit shrink.
      //
      // 0x4000 paragraphs = 256KB -- fits DOS/4GW with headroom and
      // leaves ~384KB of the arena free for grandchildren.
      constexpr uint16_t CHILD_MAX_PARAS = 0x4000;
      const uint16_t env_reserve = (ENV_BYTES + 15u) / 16u;
      mcb_find_free(0xFFFF, largest);
      if (largest <= env_reserve + 0x20) { return_error(8); break; }
      uint16_t want = largest - (env_reserve + 0x20);
      if (want > CHILD_MAX_PARAS) want = CHILD_MAX_PARAS;
      const uint16_t child_psp = mcb_allocate(want, largest);
      if (child_psp == 0) {
        return_error(8);
        break;
      }
      // Zero the entire allocated region before loading.  Reused MCB
      // blocks retain whatever the previous owner wrote -- e.g. a
      // parent's freed scratch buffer still has its COFF-read data.
      // An MZ child that only covers the first few KB would leave
      // stale parent bytes visible at child-data offsets like 0x628,
      // which DJGPP's stub reads as a "was I previously in DPMI?"
      // flag and takes a PM-broken nested-exec branch on non-zero.
      {
        const PhysPt base = static_cast<PhysPt>(child_psp) * 16u;
        const size_t bytes = static_cast<size_t>(want) * 16u;
        for (size_t i = 0; i < bytes; ++i) mem_writeb(base + i, 0);
      }

      InitialRegs child_regs;
      if (!load_program_at(r.host_path, child_psp, child_regs)) {
        mcb_free(child_psp);
        return_error(2);
        break;
      }

      // AL=1 Load-without-execute: caller wants the image in memory
      // and the initial SS:SP / CS:IP returned in its parameter
      // block.  Param block layout for AL=1:
      //   +0E DWORD SS:SP
      //   +12 DWORD CS:IP
      // We also keep the child memory allocated -- caller will later
      // jump to the entry point, and eventual cleanup is caller's
      // responsibility (AH=49 free).  Env/PSP are set up as for AL=0.
      if (reg_al == 1) {
        const PhysPt psp = child_psp * 16u;
        for (int i = 0; i < 256; ++i) mem_writeb(psp + i, 0);
        mem_writeb(psp + 0x00, 0xCD);
        mem_writeb(psp + 0x01, 0x20);
        mem_writeb(psp + 0x02, 0x00);
        mem_writeb(psp + 0x03, 0xA0);
        mem_writeb(psp + 0x2C, ENV_SEG & 0xFF);
        mem_writeb(psp + 0x2D, (ENV_SEG >> 8) & 0xFF);
        mem_writeb(psp + 0x80, 0);
        mem_writeb(psp + 0x81, 0x0D);
        const PhysPt pb = SegPhys(es) + reg_bx;
        mem_writew(pb + 0x0E, child_regs.sp);
        mem_writew(pb + 0x10, child_regs.ss);
        mem_writew(pb + 0x12, child_regs.ip);
        mem_writew(pb + 0x14, child_regs.cs);
        set_cf(false);
        return CBRET_NONE;
      }

      // Copy the parent's environment into a fresh MCB so the child
      // can mutate its own copy without corrupting the parent's.  Env
      // blocks are typically <2KB; we give the child a full 2KB
      // (ENV_BYTES/16 paragraphs) for headroom.  Any failure rolls
      // back the code-block MCB before propagating the DOS error.
      const uint16_t env_paras = (ENV_BYTES + 15u) / 16u;
      const uint16_t child_env = mcb_allocate(env_paras, largest);
      if (child_env == 0) {
        mcb_free(child_psp);
        return_error(8);
        break;
      }
      // Source is whatever env-seg is currently active for the parent
      // -- top-level parent uses ENV_SEG; a grandchild inherits from
      // its parent's PSP+0x2C.  read_parent_env_seg(parent's PSP).
      uint16_t parent_env_seg = ENV_SEG;
      {
        const PhysPt parent_psp = SegValue(ds) * 16u;
        // Trust PSP+0x2C only when parent's DS actually points at a
        // PSP (first byte = 0xCD = INT 20h marker).
        if (mem_readb(parent_psp) == 0xCD) {
          parent_env_seg = mem_readw(parent_psp + 0x2C);
          if (parent_env_seg == 0) parent_env_seg = ENV_SEG;
        }
      }
      for (uint32_t i = 0; i < ENV_BYTES; ++i) {
        mem_writeb(child_env * 16u + i,
                   mem_readb(parent_env_seg * 16u + i));
      }

      // After the env-vars section (terminated by a double-NUL),
      // DOS stores a uint16 argc followed by argv[0] as ASCIIZ --
      // this is the CHILD's executable path, which DOS/4GW reads
      // to find the LE binary it's meant to load.  The copy above
      // preserved the PARENT's argv[0], which pointed DOS/4GW at
      // the wrong file (the parent rather than the child, giving
      // "not a DOS/16M executable 'C:\WCC386.EXE'").
      //
      // Scan for the double-NUL terminator then overwrite argv[0].
      {
        const PhysPt base = child_env * 16u;
        uint32_t i = 0;
        for (; i + 1 < ENV_BYTES; ++i) {
          if (mem_readb(base + i) == 0 && mem_readb(base + i + 1) == 0) {
            i += 2;   // past the terminator
            break;
          }
        }
        // argc = 1
        if (i + 2 <= ENV_BYTES) {
          mem_writeb(base + i++, 1);
          mem_writeb(base + i++, 0);
        }
        // argv[0] = uppercase full DOS path.  The DJGPP go32-v2 stub
        // and Watcom DOS/4GW both re-open argv[0] at startup to load
        // the COFF/LE payload, so stripping the directory here breaks
        // any child spawned via an absolute or relative-to-root path
        // (DJGPP spawnlp "C:\TESTS\DJ_WRITE.EXE" -> child looks for
        // "C:\DJ_WRITE.EXE" and fails).
        std::string argv0 = dos_path;
        // Normalise: ensure drive letter + backslashes + uppercase.
        for (auto &c : argv0) {
          if (c == '/') c = '\\';
          c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
        if (argv0.size() < 2 || argv0[1] != ':') {
          if (!argv0.empty() && argv0.front() == '\\') argv0.erase(0, 1);
          argv0 = std::string("C:\\") + argv0;
        }
        for (char c : argv0) {
          if (i >= ENV_BYTES - 1) break;
          mem_writeb(base + i++, static_cast<uint8_t>(c));
        }
        if (i < ENV_BYTES) mem_writeb(base + i, 0);
      }

      // Build child PSP: INT 20h marker, top-of-memory, env pointer,
      // empty command tail.
      {
        const PhysPt psp = child_psp * 16u;
        for (int i = 0; i < 256; ++i) mem_writeb(psp + i, 0);
        mem_writeb(psp + 0x00, 0xCD);
        mem_writeb(psp + 0x01, 0x20);
        mem_writeb(psp + 0x02, 0x00);
        mem_writeb(psp + 0x03, 0xA0);
        // PSP[0x18..0x2B] = 20-byte Job File Table (handle -> SFT).
        // Matches what top-level build_psp does; DJGPP's fstat walks
        // this via PSP[0x34] (far pointer) and needs at least the
        // std handles populated or it falls through to a fallback
        // path that broke nested-DJGPP exec.
        for (int i = 0x18; i <= 0x2B; i++) mem_writeb(psp + i, 0xFF);
        mem_writeb(psp + 0x18, 0x00); // stdin
        mem_writeb(psp + 0x19, 0x01); // stdout
        mem_writeb(psp + 0x1A, 0x02); // stderr
        mem_writeb(psp + 0x1B, 0x03); // aux
        mem_writeb(psp + 0x1C, 0x04); // prn
        mem_writeb(psp + 0x2C, child_env & 0xFF);
        mem_writeb(psp + 0x2D, (child_env >> 8) & 0xFF);
        // PSP[0x32] = max open handles (word).
        mem_writeb(psp + 0x32, 20);
        mem_writeb(psp + 0x33, 0);
        // PSP[0x34..0x37] = far pointer to JFT (child_psp:0x18).
        mem_writeb(psp + 0x34, 0x18);
        mem_writeb(psp + 0x35, 0x00);
        mem_writeb(psp + 0x36, child_psp & 0xFF);
        mem_writeb(psp + 0x37, (child_psp >> 8) & 0xFF);
        // Copy command tail from the caller's parameter block.  The
        // pblock's [+2..+5] is a far pointer to a DOS-formatted
        // command tail: [len][tail bytes][0x0D].  DOS/4GW reads this
        // from its PSP to discover what .EXE to load; an empty tail
        // made it fall back to a filename-from-env path that errored
        // with "not a DOS/16M executable".
        const PhysPt pb = SegPhys(es) + reg_bx;
        const uint16_t tail_off = mem_readw(pb + 2);
        const uint16_t tail_seg = mem_readw(pb + 4);
        const PhysPt tail_base = static_cast<PhysPt>(tail_seg) * 16u + tail_off;
        const uint8_t tail_len = mem_readb(tail_base);
        mem_writeb(psp + 0x80, tail_len);
        for (uint8_t i = 0; i < tail_len && i < 127; ++i)
          mem_writeb(psp + 0x81 + i, mem_readb(tail_base + 1 + i));
        mem_writeb(psp + 0x81 + tail_len, 0x0D);
      }

      // Snapshot parent CPU state before switching to child.  The
      // shim's IRET-return frame (parent's client CS:IP:FLAGS) stays
      // untouched on parent's stack; we restore these registers
      // so the shim's IRET finds them where it left them.
      ProcessState ps{};
      ps.cs = SegValue(cs); ps.ds = SegValue(ds);
      ps.ss = SegValue(ss); ps.es = SegValue(es);
      ps.fs = SegValue(fs); ps.gs = SegValue(gs);
      ps.eip = reg_eip; ps.esp = reg_esp; ps.ebp = reg_ebp;
      ps.eax = reg_eax; ps.ebx = reg_ebx;
      ps.ecx = reg_ecx; ps.edx = reg_edx;
      ps.esi = reg_esi; ps.edi = reg_edi;
      ps.eflags = reg_flags;
      ps.child_data_seg = child_psp;
      ps.child_env_seg  = child_env;
      s_process_stack.push_back(ps);

      if (dosemu::g_debug.int4b_trace) {
        std::fprintf(stderr,
            "[4B entry] cs:eip=%04x:%08x ss:esp=%04x:%08x ds=%04x es=%04x "
            "fs=%04x gs=%04x ebp=%08x efl=%08x cpl=%u cr0=%08x\n",
            (unsigned)ps.cs, ps.eip, (unsigned)ps.ss, ps.esp,
            (unsigned)ps.ds, (unsigned)ps.es, (unsigned)ps.fs,
            (unsigned)ps.gs, ps.ebp, ps.eflags,
            (unsigned)cpu.cpl, (unsigned)cpu.cr0);
      }

      // Publish child's PSP + env segment so dpmi_entry (if the
      // child goes protected) aliases LDT[4]/LDT[5] to the right
      // memory.  Saved-and-restored around the nested RunMachine.
      const uint16_t saved_psp_seg = s_current_psp_seg;
      const uint16_t saved_env_seg = s_current_env_seg;
      s_current_psp_seg = child_psp;
      s_current_env_seg = child_env;

      // Snapshot parent's LDT state.  The child's dpmi_entry wipes
      // the entire LDT before populating its starter set, which
      // destroys any DPMI allocations the parent made (LDT[6..N]).
      // On child exit we restore the snapshot so parent's post-
      // spawn code finds its DPMI selectors intact.
      std::vector<uint8_t> saved_ldt(LDT_BYTES);
      for (uint32_t i = 0; i < LDT_BYTES; ++i)
        saved_ldt[i] = mem_readb(LDT_BASE + i);
      uint8_t saved_ldt_in_use[sizeof(s_ldt_in_use)];
      std::memcpy(saved_ldt_in_use, s_ldt_in_use, sizeof(s_ldt_in_use));

      // Snapshot parent's PM exception handlers.  Child's libc will
      // install its own (via DPMI AX=0203), whose selector:offset
      // refers to child-specific memory.  After child exits that
      // memory is freed; if parent takes a PM fault, our dispatcher
      // would call the stale child handler and #GP on the invalid
      // target -- recursive-fault-loop in our dispatcher.
      ExcHandler saved_pm_exc[32];
      std::memcpy(saved_pm_exc, s_pm_exc, sizeof(s_pm_exc));

      // Snapshot the PM ring-0 callback stack (PM_CB_STACK).  If the
      // parent reached AH=4B via a ring-3 INT 31 call, the CPU has
      // already pushed the parent's IRETD frame (outer SS/ESP/EFLAGS/
      // CS/EIP) at the top of PM_CB_STACK.  The child's own ring
      // transitions during its run push onto this same stack, which
      // clobbers parent's frame -- the subsequent IRETD would then
      // pop garbage CS/EIP and fault in random stub code.  Snapshot
      // the full 4KB before the nested RunMachine and restore after.
      uint8_t saved_cb_stack[PM_CB_STACK_SIZE];
      for (uint32_t i = 0; i < PM_CB_STACK_SIZE; ++i)
        saved_cb_stack[i] = mem_readb(PM_CB_STACK_BASE + i);

      // Switch CPU to child entry state (a real-mode CS load; shim's
      // current execution will "resume" child flow via the main loop's
      // LOADIP reading these fresh values).
      SegSet16(cs, child_regs.cs);
      SegSet16(ds, child_psp);
      SegSet16(es, child_psp);
      SegSet16(ss, child_regs.ss);
      reg_eip = child_regs.ip;
      reg_esp = child_regs.sp;
      reg_eax = child_regs.ax;

      // Parent may be in PM (DJGPP parent under DPMI).  Child is
      // entered as real-mode code (the MZ/.COM entry point).  Flip
      // CR0.PE=0 so segment loads are interpreted as 16-bit real
      // segments instead of PM selectors; restore on child exit.
      const uint32_t saved_cr0_4b = cpu.cr0;
      if (saved_cr0_4b & 1) CPU_SET_CRX(0, saved_cr0_4b & ~1u);

      DOSBOX_RunMachine();

      // Always restore parent's CR0 -- the child may have flipped PE
      // (DJGPP child did PE=1 for its own DPMI entry, then exited with
      // CR0.PE still set, but our parent may have been in RM when it
      // called AH=4B (itself via simrm)).
      CPU_SET_CRX(0, saved_cr0_4b);

      // Restore parent's LDT.  Child's dpmi_entry wiped the whole
      // table and re-populated slots 1..5 with child's bases.  Slots
      // 6+ (which the parent may have allocated via INT 31 AX=0000)
      // were lost.  Parent code using those selectors would read
      // child-era memory; restore the full 2KB snapshot we took.
      for (uint32_t i = 0; i < LDT_BYTES; ++i)
        mem_writeb(LDT_BASE + i, saved_ldt[i]);
      std::memcpy(s_ldt_in_use, saved_ldt_in_use, sizeof(s_ldt_in_use));
      // Restore parent's PM exception handlers (child's were in
      // child-memory that's now freed).
      std::memcpy(s_pm_exc, saved_pm_exc, sizeof(s_pm_exc));
      // Restore parent's PM_CB_STACK contents.  Parent's ring-3->0
      // INT 31 frame lives near the top of this stack; child's
      // ring transitions overwrote it.
      for (uint32_t i = 0; i < PM_CB_STACK_SIZE; ++i)
        mem_writeb(PM_CB_STACK_BASE + i, saved_cb_stack[i]);

      // Child exited via AH=4Ch; restore parent state.
      s_child_exit_pending = false;
      s_current_psp_seg = saved_psp_seg;
      s_current_env_seg = saved_env_seg;
      const ProcessState restored = s_process_stack.back();
      s_process_stack.pop_back();
      // In PM, parent's segment selectors are LDT/GDT indices and
      // their bases must come from the descriptor tables -- not from
      // `val << 4` (which SegSet16 does).  Also, the child's
      // dpmi_entry overwrote the starter-set LDT slots (1..5) with
      // ITS base addresses, so we must rewrite parent's bases first.
      if (saved_cr0_4b & 1) {
        // Parent was in PM.  The LDT snapshot/restore above already
        // put parent's descriptors back; use the PARENT's saved
        // selectors (not hardcoded starter-set values -- DJGPP's libc
        // may have switched CS/DS/ES to LDT[6..8] for its 32-bit
        // flat-model runtime).  CPU_SetSegGeneral re-reads the
        // descriptor we just restored so Segs.phys is accurate.
        CPU_SetSegGeneral(ds, restored.ds);
        CPU_SetSegGeneral(ss, restored.ss);
        CPU_SetSegGeneral(es, restored.es);
        CPU_SetSegGeneral(fs, restored.fs);
        CPU_SetSegGeneral(gs, restored.gs);
        // CS can't be loaded via MOV -- refresh cached base from the
        // descriptor directly.  cpu.gdt.GetDescriptor handles both
        // GDT and LDT selectors (routes via the TI bit).
        Descriptor d;
        cpu.gdt.GetDescriptor(restored.cs, d);
        Segs.val[cs]  = restored.cs;
        Segs.phys[cs] = d.GetBase();
      } else {
        SegSet16(cs, restored.cs);
        SegSet16(ds, restored.ds);
        SegSet16(ss, restored.ss);
        SegSet16(es, restored.es);
        SegSet16(fs, restored.fs);
        SegSet16(gs, restored.gs);
      }
      reg_eip = restored.eip;
      reg_esp = restored.esp;
      reg_ebp = restored.ebp;
      reg_eax = restored.eax; reg_ebx = restored.ebx;
      reg_ecx = restored.ecx; reg_edx = restored.edx;
      reg_esi = restored.esi; reg_edi = restored.edi;
      reg_flags = restored.eflags;

      if (dosemu::g_debug.int4b_trace) {
        Descriptor dcs;
        cpu.gdt.GetDescriptor(Segs.val[cs], dcs);
        std::fprintf(stderr,
            "[4B exit]  cs:eip=%04x:%08x ss:esp=%04x:%08x ds=%04x es=%04x "
            "fs=%04x gs=%04x efl=%08x cpl=%u cr0=%08x cs.base=%08x\n",
            (unsigned)Segs.val[cs], reg_eip, (unsigned)Segs.val[ss],
            reg_esp, (unsigned)Segs.val[ds], (unsigned)Segs.val[es],
            (unsigned)Segs.val[fs], (unsigned)Segs.val[gs], reg_flags,
            (unsigned)cpu.cpl, (unsigned)cpu.cr0,
            (unsigned)dcs.GetBase());
      }

      mcb_free(child_psp);
      if (restored.child_env_seg) mcb_free(restored.child_env_seg);

      // Expose the exit code via AH=4Dh and return it in AL for
      // callers that prefer the simpler "read AL after 4B" idiom.
      s_last_child_exit = restored.child_exit_code;
      reg_al = restored.child_exit_code;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x4D: {  // Get Child Return Code
      // Output: AL = exit code, AH = termination type (0=normal,
      //         1=Ctrl-C, 2=critical error, 3=TSR).  We always report
      //         0 for AH since our AH=4C path doesn't distinguish.
      reg_ax = s_last_child_exit;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x6C: {  // Extended open/create.  BX=mode, CX=attr, DX=action,
                  // DS:SI=path.  Returns AX=handle, CX=action-taken
                  // (1=opened, 2=created, 3=truncated).
      const std::string dos_path = read_dos_string(SegValue(ds), reg_si);
      const Resolved    r        = resolve_path(dos_path);
      const bool exists = (::access(r.host_path.c_str(), F_OK) == 0);
      const uint16_t action = reg_dx;
      const bool open_existing = (action & 0x01) != 0;
      const bool truncate_existing = (action & 0x02) != 0;
      const bool create_new    = (action & 0x10) != 0;

      if (exists && !open_existing && !truncate_existing) {
        return_error(0x50);  // file already exists
        break;
      }
      if (!exists && !create_new) {
        return_error(0x02);  // file not found
        break;
      }

      int flags = O_RDONLY;
      switch (reg_bx & 0x07) {
        case 0: flags = O_RDONLY; break;
        case 1: flags = O_WRONLY; break;
        case 2: flags = O_RDWR;   break;
      }
      uint16_t action_taken;
      if (!exists) { flags |= O_CREAT; action_taken = 2; }
      else if (truncate_existing) { flags |= O_TRUNC; action_taken = 3; }
      else { action_taken = 1; }

      int fd = ::open(r.host_path.c_str(), flags, 0644);
      if (fd < 0) { return_error(0x05); break; }
      int h = allocate_handle(fd, r.text_mode);
      if (h < 0) { ::close(fd); return_error(0x04); break; }
      reg_ax = static_cast<uint16_t>(h);
      reg_cx = action_taken;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x66: {  // Get/Set global code page
      // Watcom's runtime probes this during startup.  Return CF=0 with
      // BX=DX=437 (US code page) so the runtime's locale init proceeds.
      reg_bx = 437;
      reg_dx = 437;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x68: {  // Commit file (DOS 4+, fsync equivalent)
      // Caller wants changes flushed to disk.  Our host-side writes are
      // already through write(2); fsync would be nicer but unnecessary.
      // Report success.
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x52: {  // Get List-of-Lists pointer (ES:BX -> SYSVARS)
      // Real DOS has a complex "list of lists" internal structure.  We
      // don't have one, but we DO need its first SFT-chain-pointer
      // slot at offset 4 to contain 0xFFFF -- DJGPP's fstat() walks
      // the SFT chain to look up fd → file info, and treats 0xFFFF as
      // "end of chain" (return -2, fall back to non-SFT path).  If we
      // return a pointer whose +4 word is garbage, fstat fails with
      // EBADF on our file handles.
      //
      // Maintain a 16-byte static block at linear 0x0600 with the
      // sentinel in place.  Point ES:BX there.
      static bool inited = false;
      if (!inited) {
        for (int i = 0; i < 16; i++) mem_writeb(0x600 + i, 0);
        mem_writeb(0x604, 0xFF);  // SFT chain head offset lo
        mem_writeb(0x605, 0xFF);  // SFT chain head offset hi
        inited = true;
      }
      SegSet16(es, 0x0060);
      reg_bx = 0;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0xFF: {
      // Undocumented DOS/4GW API (Rational Systems / Tenberry).
      // Watcom's C runtime probes for DOS/4GW's presence via
      // AH=FFh sub-functions when starting a 32-bit LE binary.
      // Without this, wcc386.exe, wd.exe, vi.exe and friends fall
      // through to a no-extender fallback path that GP-faults.
      //
      // The key probe is:
      //   AH=FFh DH=00h DL=78h -> if DOS/4G present:
      //     EAX = 0x4734FFFF   (high word 0x4734 = byte-swapped "4G")
      //
      // (Reference: Ralf Brown's Interrupt List, INT 21 AH=FFh DH=00h.)
      //
      // Claiming DOS/4G presence is a promise we can't fully keep --
      // real DOS/4G provides additional API entry points that clients
      // may call after detection.  But advertising presence here
      // gets the Watcom runtime past its hard-coded "abort if no
      // extender detected" step, which is the current blocker.
      // Further DOS/4G-specific entry points can be stubbed as
      // clients hit them.
      if (reg_al == 0x00 && reg_dh == 0x00 && reg_dl == 0x78) {
        reg_eax = 0x4734FFFFu;
        set_cf(false);
        return CBRET_NONE;
      }
      // Fall through to the soft-fail path for other AH=FFh sub-fns.
      return_error(0x01);
      break;
    }

    case 0x58: {  // Get/Set Memory Allocation Strategy / UMB link state.
      // DJGPP startup fires AH=58 AL=00 (get strategy) as part of
      // its libc init.  Returning error makes it fall back silently,
      // but reporting "first fit" (0) is more truthful.  Set (AL=01)
      // and UMB ops (AL=02/03) are accepted silently.
      switch (reg_al) {
        case 0x00: reg_ax = 0x0000; set_cf(false); return CBRET_NONE; // first-fit
        case 0x01: set_cf(false);   return CBRET_NONE;                // set strategy (no-op)
        case 0x02: reg_al = 0;      set_cf(false); return CBRET_NONE; // UMBs unlinked
        case 0x03: set_cf(false);   return CBRET_NONE;                // set UMB link (no-op)
      }
      return_error(0x01);
      break;
    }

    case 0x5E: {  // Network / machine-name services.
      // GNU find queries the NetBIOS machine name via AH=5E AL=00.
      // Reply with a fixed "DOSEMU" identifier so the "unimplemented"
      // log noise goes away; the specific name is cosmetic for our
      // purposes and no real network is involved.
      if (reg_al == 0x00) {
        const PhysPt buf = SegPhys(ds) + reg_dx;
        static const char kName[17] = "DOSEMU          ";
        for (int i = 0; i < 16; ++i) mem_writeb(buf + i, kName[i]);
        reg_cx = 0;    // CL=0: no network
        reg_ch = 1;    // CH=1: name is valid
        set_cf(false);
        return CBRET_NONE;
      }
      // Other AL sub-functions (printer redirect, get NetBIOS vars)
      // aren't implemented.  Return error; no real callers hit them.
      return_error(0x01);
      break;
    }

    case 0x71: {
      // AH=71 is the DOS LFN (Long File Name) API.  For most
      // sub-functions we signal "LFN not supported" so DJGPP falls
      // back to the SFN API we handle (AH=3C, 3D, 4E, etc.).
      //
      // Exception: AL=60h or A0h (truename, long-form canonicalize)
      // has no SFN equivalent -- GNU flex with `-o out.c` calls
      // AL=A0 on its output file path and bails if we return "LFN
      // not supported" instead of giving it a canonicalized path.
      // Delegate to the AH=60 logic (builds a "C:\...\" path).
      if (reg_al == 0x60 || reg_al == 0xA0) {
        const std::string src = read_dos_string(SegValue(ds), reg_si);
        std::string path = src;
        char drive = s_current_drive;
        if (path.size() >= 2 && path[1] == ':') {
          drive = static_cast<char>(std::toupper(
                      static_cast<unsigned char>(path[0])));
          path.erase(0, 2);
        }
        if (path.empty() || (path[0] != '\\' && path[0] != '/')) {
          const std::string &cwd = s_drive_cwd[drive];
          if (!cwd.empty() && cwd.front() != '\\' && cwd.front() != '/')
            path = "\\" + cwd + "\\" + path;
          else
            path = (cwd.empty() ? "\\" : cwd + "\\") + path;
        }
        for (auto &c : path) {
          if (c == '/') c = '\\';
          c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
        std::string canon;
        canon += drive;
        canon += ':';
        canon += path;
        // LFN buffer is 260 bytes; leave room for NUL.
        if (canon.size() > 259) canon.resize(259);
        const PhysPt dst = SegPhys(es) + reg_di;
        for (size_t i = 0; i < canon.size(); ++i)
          mem_writeb(dst + i, static_cast<uint8_t>(canon[i]));
        mem_writeb(dst + canon.size(), 0);
        reg_ax = 0;
        set_cf(false);
        return CBRET_NONE;
      }
      // Other LFN sub-functions: for findfirst/findnext/findclose
      // (AL=4E/4F/A1) specifically, return a real DOS error so
      // libc callers like DJGPP's newlib stat() map it to ENOENT
      // instead of EINVAL.  AX=0x7100 (LFN not supported) is the
      // conservative answer but make 4.4's stat() sees that as
      // EINVAL and fails before trying the SFN fallback.
      if (reg_al == 0x4E || reg_al == 0x4F || reg_al == 0xA1) {
        return_error(0x02);  // file not found -> ENOENT
        return CBRET_NONE;
      }
      // For other sub-functions: signal not-supported so callers
      // fall back to SFN.
      reg_ax = 0x7100;
      set_cf(true);
      return CBRET_NONE;
    }

    default:
      // Unknown sub-function.  Log once per distinct AH to stderr for
      // debugging, then return the DOS "invalid function" error so the
      // program continues -- killing on unimplemented AH values prevents
      // real tools from ever reaching the parts we *do* handle.
      static std::set<uint8_t> warned;
      if (warned.insert(reg_ah).second) {
        std::fprintf(stderr,
                     "dosemu: unimplemented INT 21h AH=%02Xh "
                     "(AL=%02Xh BX=%04Xh CX=%04Xh DX=%04Xh) -- returning "
                     "invalid-function, program continues\n",
                     reg_ah, reg_al, reg_bx, reg_cx, reg_dx);
      }
      return_error(0x01);
      break;
  }
  return CBRET_NONE;
}

// --- Program load ----------------------------------------------------------

// Read a file entirely into a host buffer; empty on failure.
std::vector<uint8_t> read_file(const std::string &path) {
  std::vector<uint8_t> out;
  std::FILE *f = std::fopen(path.c_str(), "rb");
  if (!f) {
    std::fprintf(stderr, "dosemu: cannot open %s: %s\n",
                 path.c_str(), std::strerror(errno));
    return out;
  }
  std::fseek(f, 0, SEEK_END);
  long size = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  if (size < 0) { std::fclose(f); return out; }
  out.resize(static_cast<size_t>(size));
  if (std::fread(out.data(), 1, size, f) != static_cast<size_t>(size)) {
    std::fprintf(stderr, "dosemu: read error on %s\n", path.c_str());
    out.clear();
  }
  std::fclose(f);
  return out;
}

// Load a .COM image at PSP_SEG:0100.  .COM conventions: CS=DS=ES=SS=PSP_SEG,
// IP=100h, SP=FFFEh.
bool load_com_at(const std::string &path, uint16_t psp_seg, InitialRegs &out) {
  const auto bytes = read_file(path);
  if (bytes.empty()) return false;
  if (bytes.size() > MAX_COM_SIZE) {
    std::fprintf(stderr, "dosemu: %s too large for .COM (%zu bytes)\n",
                 path.c_str(), bytes.size());
    return false;
  }
  const PhysPt load_addr = psp_seg * 16u + COM_ENTRY_OFFSET;
  for (size_t i = 0; i < bytes.size(); ++i)
    mem_writeb(load_addr + i, bytes[i]);
  out = {psp_seg, COM_ENTRY_OFFSET, psp_seg, 0xFFFE, 0};
  return true;
}

// Helper: read a little-endian 16-bit word from a byte buffer.
uint16_t rdw(const std::vector<uint8_t> &b, size_t off) {
  return static_cast<uint16_t>(b[off]) |
         (static_cast<uint16_t>(b[off + 1]) << 8);
}
// Helper: read a little-endian 32-bit dword.
uint32_t rdd(const std::vector<uint8_t> &b, size_t off) {
  return static_cast<uint32_t>(b[off]) |
         (static_cast<uint32_t>(b[off + 1]) << 8)  |
         (static_cast<uint32_t>(b[off + 2]) << 16) |
         (static_cast<uint32_t>(b[off + 3]) << 24);
}

// LE (Linear Executable) binary format -- Watcom DOS4G/W's output.
// MZ stub at the start, "new exe" header at file[0x3C] pointing to a
// "LE" or "LX" signature.  For now we parse and report the structure;
// full execution requires installing PM descriptors for each object,
// setting CS:EIP and SS:ESP to the entry point, and jumping into the
// image.  That's substantial follow-up work.  This scaffolding lets
// dosemu recognize LE binaries and fail with a clear, parseable error
// rather than the generic "header mismatch" the MZ path would give.
// Per-object runtime record populated by load_le_at.  Enough metadata
// for a future-session "enter PM at entry point" routine to install a
// code/data descriptor per object and translate the LE's linear view
// into our PM-mapped memory.
struct LeObject {
  uint32_t virt_base;       // client-visible linear base from the LE header
  uint32_t virt_size;
  uint32_t flags;           // copy of the object table's flags word
  uint32_t first_page;      // 1-based index of the object's first page
  uint32_t page_count;
  uint32_t host_base;       // host linear address where we placed the object
  uint16_t host_seg;        // for MCB-backed objects, the paragraph seg;
                            // 0 if pm_arena-backed (look at host_base instead)
  uint16_t ldt_sel;         // LDT selector installed by le_install_descriptors
                            // (0 = not yet installed)
  bool     is_code;
  bool     is_big;          // 0x2000 "BIG" bit => 32-bit
};

bool load_le_inspect(const std::string &path,
                     const std::vector<uint8_t> &f,
                     size_t le_off) {
  if (f.size() < le_off + 0xB0) return false;
  const char sig[3] = { static_cast<char>(f[le_off]),
                        static_cast<char>(f[le_off + 1]), 0 };
  if (std::strcmp(sig, "LE") != 0 && std::strcmp(sig, "LX") != 0) return false;

  const uint16_t cpu_type     = rdw(f, le_off + 0x08);
  const uint32_t num_pages    = rdd(f, le_off + 0x14);
  const uint32_t entry_obj    = rdd(f, le_off + 0x18);  // 1-based
  const uint32_t entry_eip    = rdd(f, le_off + 0x1C);
  const uint32_t stack_obj    = rdd(f, le_off + 0x20);
  const uint32_t stack_esp    = rdd(f, le_off + 0x24);
  const uint32_t page_size    = rdd(f, le_off + 0x28);
  const uint32_t obj_tbl_off  = rdd(f, le_off + 0x40);
  const uint32_t num_objects  = rdd(f, le_off + 0x44);
  const uint32_t page_tbl_off = rdd(f, le_off + 0x48);
  const uint32_t data_pages   = rdd(f, le_off + 0x80);

  std::fprintf(stderr,
               "dosemu: %s is %s (CPU=%u, %u pages of %u bytes, %u objects, "
               "entry obj#%u+0x%x, stack obj#%u+0x%x)\n",
               path.c_str(), sig, cpu_type, num_pages, page_size,
               num_objects, entry_obj, entry_eip, stack_obj, stack_esp);

  // Dump per-object metadata.  Useful for future-session loader work.
  for (uint32_t i = 0; i < num_objects && i < 16; ++i) {
    const size_t entry = le_off + obj_tbl_off + i * 24;
    if (entry + 24 > f.size()) break;
    const uint32_t virt_size    = rdd(f, entry + 0x00);
    const uint32_t reloc_base   = rdd(f, entry + 0x04);
    const uint32_t flags        = rdd(f, entry + 0x08);
    const uint32_t page_idx     = rdd(f, entry + 0x0C);
    const uint32_t page_count   = rdd(f, entry + 0x10);
    std::fprintf(stderr,
                 "  obj %u: base=0x%08x size=0x%x flags=0x%04x pages=%u..%u (%s%s%s%s)\n",
                 i + 1, reloc_base, virt_size, flags,
                 page_idx, page_idx + page_count - 1,
                 (flags & 0x0004) ? "X" : "-",
                 (flags & 0x0002) ? "W" : "-",
                 (flags & 0x0001) ? "R" : "-",
                 (flags & 0x2000) ? "32" : "16");
  }
  (void)data_pages; (void)page_tbl_off;
  return true;
}

// Copy the pages of each LE object into host memory allocated from
// the MCB chain.  Populates `objects` with one entry per object,
// including the host seg where the data ended up.  Caller is
// responsible for freeing the host segments on failure or on
// "client exit" (a future follow-up).
//
// Page types we handle:
//   0 Legal         -- copy bytes from data_pages_off + (page-1)*4KB
//   1 Iterated      -- compressed RLE; logs a one-shot warning and
//                      leaves the page zeroed (loader continues, but a
//                      binary with iterated pages won't execute
//                      correctly until this is expanded).  Uncommon.
//   2 Invalid       -- leave zeros
//   3 Zero-filled   -- leave zeros (we allocated zero-filled memory)
//   4 Range-of-zeros (LX) -- leave zeros
//
// Last-page-size field on the LE header clips the very last page's
// copy to the exact byte count.  No fixups are applied here; that's
// a follow-up.  Returns true on success.
bool le_load_objects(const std::vector<uint8_t> &f, size_t le_off,
                     std::vector<LeObject> &objects) {
  const uint32_t num_objects  = rdd(f, le_off + 0x44);
  const uint32_t obj_tbl_off  = rdd(f, le_off + 0x40);
  const uint32_t page_tbl_off = rdd(f, le_off + 0x48);
  const uint32_t page_size    = rdd(f, le_off + 0x28);
  const uint32_t last_page_sz = rdd(f, le_off + 0x2C);
  const uint32_t total_pages  = rdd(f, le_off + 0x14);
  const uint32_t data_pages   = rdd(f, le_off + 0x80);
  objects.reserve(num_objects);
  for (uint32_t i = 0; i < num_objects; ++i) {
    const size_t e = le_off + obj_tbl_off + i * 24;
    if (e + 24 > f.size()) return false;
    LeObject o{};
    o.virt_size = rdd(f, e + 0x00);
    o.virt_base = rdd(f, e + 0x04);
    o.flags     = rdd(f, e + 0x08);
    o.is_code   = (o.flags & 0x0004) != 0;
    // LE object flag bit 13 (0x2000) = BIG -- default operand size is
    // 32 bits (descriptor D=1).  Bit 14 (0x4000) is "conforming" for
    // code segments, NOT the BIG bit.  Spec: OS/2 2.x LE format,
    // object table flag layout.  We were reading the wrong bit, which
    // made real 32-bit LE binaries (e.g. Watcom's wd.exe with obj
    // flags 0x2045) look 16-bit and their 20-bit entry_eip (0x6f104)
    // unreachable through a 16-bit IP.
    o.is_big    = (o.flags & 0x2000) != 0;
    const uint32_t page_idx    = rdd(f, e + 0x0C);   // 1-based
    const uint32_t page_count  = rdd(f, e + 0x10);
    o.first_page  = page_idx;
    o.page_count  = page_count;
    // Allocate host memory sized to cover virt_size.  Small objects
    // that fit in a paragraph-count fit the MCB arena (keeps them
    // below 1MB, handy for future mixed 16-bit/32-bit object use).
    // Anything bigger goes to pm_arena above 1MB.
    const uint32_t paras = (o.virt_size + 15u) / 16u;
    if (paras == 0) {
      std::fprintf(stderr, "dosemu: LE obj %u paras=0\n", i + 1);
      return false;
    }
    o.host_seg  = 0;
    o.host_base = 0;
    if (paras <= 0xFFFFu) {
      uint16_t largest = 0;
      const uint16_t seg = mcb_allocate(
          static_cast<uint16_t>(paras), largest);
      if (seg != 0) {
        o.host_seg  = seg;
        o.host_base = static_cast<uint32_t>(seg) * 16u;
      }
    }
    if (o.host_base == 0) {
      const uint32_t base = pm_alloc(o.virt_size);
      if (base == 0) {
        std::fprintf(stderr, "dosemu: LE obj %u: pm_alloc %u bytes failed\n",
                     i + 1, o.virt_size);
        return false;
      }
      o.host_base = base;
    }
    // Zero-fill.  mem_writeb is slow for large blocks; we accept that
    // here because LE objects tend to have meaningful initialization
    // data (BSS is uncommon) so in practice the copy loop below
    // overwrites most bytes anyway.  Keep the explicit zero so
    // uninitialized regions start cleanly rather than holding the
    // previous allocation's bytes.
    for (uint32_t j = 0; j < o.virt_size; ++j)
      mem_writeb(o.host_base + j, 0);
    // Copy pages.
    for (uint32_t p = 0; p < page_count; ++p) {
      const uint32_t pt_entry_idx = page_idx - 1 + p;
      if (pt_entry_idx >= total_pages) continue;
      const size_t pt_entry = le_off + page_tbl_off + pt_entry_idx * 4;
      if (pt_entry + 4 > f.size()) continue;
      // LE page entry: 3-byte page number (big-endian) + 1-byte type.
      const uint32_t pnum_be = (static_cast<uint32_t>(f[pt_entry]) << 16)
                             | (static_cast<uint32_t>(f[pt_entry + 1]) << 8)
                             |  static_cast<uint32_t>(f[pt_entry + 2]);
      const uint8_t  ptype   = f[pt_entry + 3];
      if (ptype != 0) {
        // Non-"legal" page types leave the allocated memory at its
        // pre-zeroed state.  Warn once per unique ptype so the user
        // knows when a binary hits an untested code path.  Known:
        //   1 Iterated     -- RLE-compressed; uncommon but exists
        //   2 Invalid      -- fine to leave zero
        //   3 Zero-filled  -- fine to leave zero
        //   4 LX Range-of-zeros -- fine to leave zero
        static uint8_t seen_types = 0;
        const uint8_t bit = 1u << (ptype & 7);
        if (!(seen_types & bit)) {
          seen_types |= bit;
          const char *name =
              ptype == 1 ? "Iterated (RLE-compressed, zero-fill stub)"
            : ptype == 2 ? "Invalid (zero-fill OK)"
            : ptype == 3 ? "Zero-filled (zero-fill OK)"
            : ptype == 4 ? "Range-of-zeros (zero-fill OK)"
            :              "unknown";
          std::fprintf(stderr,
              "dosemu: LE obj %u page %u: non-legal page type 0x%x -- %s\n",
              i + 1, p, ptype, name);
        }
        continue;
      }
      if (pnum_be == 0) continue;
      const uint32_t file_pg_off = data_pages + (pnum_be - 1) * page_size;
      uint32_t copy_bytes = page_size;
      if (pnum_be == total_pages && last_page_sz != 0)
        copy_bytes = last_page_sz;
      const uint32_t dst_off = p * page_size;
      if (dst_off + copy_bytes > o.virt_size)
        copy_bytes = o.virt_size - dst_off;
      for (uint32_t j = 0; j < copy_bytes; ++j) {
        if (file_pg_off + j >= f.size()) break;
        mem_writeb(o.host_base + dst_off + j, f[file_pg_off + j]);
      }
    }
    objects.push_back(o);
  }
  return true;
}

// Walk the fixup-page-table + fixup-record-table and patch source
// offsets inside each object's host memory so that internal references
// land on the actual (post-load) location of their target object.  The
// LE file expresses every reference as (source_type, target_obj_num,
// target_offset) -- our job is to turn that into a concrete host
// linear address objects[tgt].host_base + target_offset.
//
// Supported source types (DPMI 0.9 LE section 7):
//   0x02 16-bit selector           - stub: leaves zero (no PM sel yet)
//   0x05 16-bit offset             - low 16 bits of target linear
//   0x06 16:32 pointer             - selector stub + 32-bit offset
//   0x07 32-bit offset             - full 32-bit target linear
//   0x08 32-bit self-relative      - target_linear - (src_addr + 4)
// Target reference types supported:
//   0b00 internal reference (everything we need for no-imports LE)
// Unsupported (logged + skipped): imported-by-ordinal, by-name, and
// entry-table targets.  Those require imports the mini-loader doesn't
// plan to grow.
bool le_apply_fixups(const std::vector<uint8_t> &f, size_t le_off,
                     const std::vector<LeObject> &objects) {
  const uint32_t num_pages      = rdd(f, le_off + 0x14);
  const uint32_t page_size      = rdd(f, le_off + 0x28);
  const uint32_t fixup_page_tbl = rdd(f, le_off + 0x68);
  const uint32_t fixup_rec_tbl  = rdd(f, le_off + 0x6C);
  if (fixup_page_tbl == 0 || fixup_rec_tbl == 0 || num_pages == 0)
    return true;  // nothing to do

  auto find_obj_for_page = [&](uint32_t page_1based) -> const LeObject * {
    for (const auto &o : objects)
      if (page_1based >= o.first_page
          && page_1based < o.first_page + o.page_count)
        return &o;
    return nullptr;
  };

  uint32_t total_fixups = 0;
  for (uint32_t pg = 0; pg < num_pages; ++pg) {
    const size_t pt_entry = le_off + fixup_page_tbl + pg * 4;
    const size_t pt_next  = le_off + fixup_page_tbl + (pg + 1) * 4;
    if (pt_next + 4 > f.size()) return false;
    const uint32_t rec_start = rdd(f, pt_entry);
    const uint32_t rec_end   = rdd(f, pt_next);
    if (rec_end < rec_start) return false;
    if (rec_start == rec_end) continue;  // no fixups on this page

    const uint32_t page_1based = pg + 1;
    const LeObject *src_obj = find_obj_for_page(page_1based);
    if (!src_obj) {
      std::fprintf(stderr, "dosemu: LE fixup: page %u has no owning object\n",
                   page_1based);
      continue;
    }
    const uint32_t page_off_in_obj =
        (page_1based - src_obj->first_page) * page_size;

    size_t p = le_off + fixup_rec_tbl + rec_start;
    const size_t p_end = le_off + fixup_rec_tbl + rec_end;
    while (p + 5 <= p_end && p + 5 <= f.size()) {
      const uint8_t  src_type   = f[p + 0];
      const uint8_t  tgt_flags  = f[p + 1];
      const int16_t  src_off_s  = static_cast<int16_t>(rdw(f, p + 2));
      p += 4;
      const uint8_t  ref_type   = tgt_flags & 0x03;
      const bool     f_16obj    = (tgt_flags & 0x40) != 0;
      const bool     f_32toff   = (tgt_flags & 0x10) != 0;

      if (ref_type != 0x00) {
        // Skip imports / entry-table targets.  Parse just enough to
        // advance past them; for our minimal fixture they never occur.
        std::fprintf(stderr, "dosemu: LE fixup: ref_type %u (imports) "
                     "not supported, skipping record\n", ref_type);
        return false;
      }

      if (p >= f.size()) return false;
      uint32_t tgt_obj;
      if (f_16obj) { tgt_obj = rdw(f, p); p += 2; }
      else         { tgt_obj = f[p];      p += 1; }

      uint32_t tgt_off = 0;
      // type 2 (16-bit selector) has no target offset in the record.
      if ((src_type & 0x0F) != 0x02) {
        if (f_32toff) { if (p + 4 > f.size()) return false;
                        tgt_off = rdd(f, p); p += 4; }
        else          { if (p + 2 > f.size()) return false;
                        tgt_off = rdw(f, p); p += 2; }
      }

      if (tgt_obj == 0 || tgt_obj > objects.size()) {
        std::fprintf(stderr, "dosemu: LE fixup: bad target obj %u\n", tgt_obj);
        continue;
      }
      const LeObject &to = objects[tgt_obj - 1];
      const uint32_t target_linear = to.host_base + tgt_off;

      // Compute the source host address.
      const int32_t src_off_in_page = static_cast<int32_t>(src_off_s);
      const int32_t src_off_in_obj  = static_cast<int32_t>(page_off_in_obj)
                                    + src_off_in_page;
      if (src_off_in_obj < 0
          || static_cast<uint32_t>(src_off_in_obj) >= src_obj->virt_size) {
        std::fprintf(stderr, "dosemu: LE fixup: src off 0x%x outside obj\n",
                     src_off_in_obj);
        continue;
      }
      const uint32_t src_addr = src_obj->host_base
                              + static_cast<uint32_t>(src_off_in_obj);

      auto patch8  = [&](uint32_t a, uint32_t v) { mem_writeb(a, v & 0xFF); };
      auto patch16 = [&](uint32_t a, uint32_t v) {
        patch8(a, v); patch8(a + 1, v >> 8);
      };
      auto patch32 = [&](uint32_t a, uint32_t v) {
        patch16(a, v); patch16(a + 2, v >> 16);
      };

      const uint8_t s = src_type & 0x0F;
      switch (s) {
        case 0x05:  // 16-bit offset
          patch16(src_addr, target_linear);
          break;
        case 0x07:  // 32-bit offset
          patch32(src_addr, target_linear);
          break;
        case 0x08: {  // 32-bit self-relative
          const uint32_t rel = target_linear - (src_addr + 4);
          patch32(src_addr, rel);
          break;
        }
        case 0x02:  // 16-bit selector: the target object's LDT selector
          patch16(src_addr, to.ldt_sel);
          break;
        case 0x06:  // 16:32 far pointer: 32-bit offset + 16-bit selector.
          // Layout on disk (little-endian): 4 bytes offset, then 2
          // bytes selector.  Store offset (relative to target object)
          // in the first dword, selector in the trailing word.
          patch32(src_addr, tgt_off);
          patch16(src_addr + 4, to.ldt_sel);
          break;
        case 0x03:  // 16:16 far pointer: 16-bit offset + 16-bit selector.
          patch16(src_addr, tgt_off);
          patch16(src_addr + 2, to.ldt_sel);
          break;
        case 0x00:  // byte fixup
          patch8(src_addr, target_linear);
          break;
        default:
          std::fprintf(stderr, "dosemu: LE fixup: unhandled src_type 0x%02x\n",
                       src_type);
          break;
      }
      // Per-fixup output: always print the first 16 (plenty for
      // LE_MIN.EXE and for spotting early fixup-walker regressions);
      // suppress the rest unless DOSEMU_TRACE is set.  Real LE
      // binaries (wd.exe, vi.exe) have 13000+ fixups each.
      if (total_fixups < 16 || dosemu::g_debug.trace) {
        std::fprintf(stderr,
            "dosemu: LE fixup: page %u off 0x%04x type 0x%02x -> "
            "obj%u+0x%x = 0x%08x (at host 0x%05x)\n",
            page_1based, src_off_s & 0xFFFF, src_type,
            tgt_obj, tgt_off, target_linear, src_addr);
      } else if (total_fixups == 16) {
        std::fprintf(stderr,
            "dosemu: LE fixup: (further entries suppressed; rerun "
            "with DOSEMU_TRACE=1 for the full list)\n");
      }
      ++total_fixups;
    }
  }
  std::fprintf(stderr, "dosemu: LE fixups applied: %u\n", total_fixups);
  return true;
}

// Install one LDT descriptor per LE object.  Base = host_base,
// limit = virt_size - 1, access byte derived from object flags:
//   code object (flag 0x0004) => 0x9A (present, DPL=0, code, readable)
//   data object              => 0x92 (present, DPL=0, data, writable)
// D-bit from BIG flag (0x2000) sets default operand size:
//   16-bit object (BIG=0) => descriptor D=0
//   32-bit object (BIG=1) => descriptor D=1
// Writes the resulting selector back into o.ldt_sel.  Returns true
// on success.  Caller is responsible for freeing the descriptors (via
// the existing LDT bitmap machinery) if it tears the load down.
bool le_install_descriptors(std::vector<LeObject> &objects) {
  // Reuse the DPMI PM infrastructure: the LDT lives at LDT_BASE, the
  // in-use bitmap tracks free slots, and write_ldt_descriptor writes
  // the 8-byte descriptor with the right base/limit/access layout.
  // This does *not* enter PM -- clients loading an LE that are going
  // to execute in PM must do that separately.  The descriptors we
  // install are valid for an already-running DPMI-style PM session.
  // LDT_BASE is above 1MB so A20 must be on before we write; default
  // DOS state has A20 off and writes would wrap to low memory.
  MEM_A20_Enable(true);
  const uint16_t need = static_cast<uint16_t>(objects.size());
  const uint16_t start = ldt_find_run(need);
  if (start == 0) {
    std::fprintf(stderr, "dosemu: LE: no LDT run of %u descriptors\n", need);
    return false;
  }
  for (uint16_t i = 0; i < need; ++i) {
    auto &o = objects[i];
    const uint16_t idx = start + i;
    const uint8_t access = o.is_code ? 0x9A : 0x92;
    const uint32_t limit = (o.virt_size > 0) ? (o.virt_size - 1) : 0;
    write_ldt_descriptor(idx, o.host_base, limit, access, o.is_big);
    ldt_set(idx, true);
    o.ldt_sel = static_cast<uint16_t>((idx << 3) | 0x04 | 0x00);  // TI=1, RPL=0
    std::fprintf(stderr,
        "dosemu: LE obj %u: LDT slot %u sel=0x%04x base=0x%08x "
        "limit=0x%x access=0x%02x D=%u\n",
        i + 1, idx, o.ldt_sel, o.host_base, limit, access, o.is_big ? 1 : 0);
  }
  return true;
}

// LE launch prep: called after le_install_descriptors has populated
// LDT slots for each object.  Seeds GDT/IDT/LDTR machinery in RM so
// that dosemu_startup's is_pm branch can finish the mode switch with
// just CR0.PE=1 + CPU_LLDT + segment loads + CPU_JMP.  We don't flip
// CR0 here: subsequent code in load_exe_at and the return up to
// dosemu_startup would run in PM with RM CS, which corrupts things.
void le_launch_pm_prep(bool bits32) {
  // Gate bitness must match the entry object's BIG flag.  A 32-bit
  // gate pushes a 12-byte frame and IRETD pops 12 bytes; a 16-bit
  // gate pushes 6 bytes and IRET pops 6 bytes.  Mixing the two
  // results in garbage on the stack and an immediate #GP cascade.
  // GDT[1..4] are unused by LE clients (they use LDT selectors for
  // everything) but pm_setup_gdt_and_idt writes valid descriptors
  // covering the current RM segments anyway -- harmless.
  pm_setup_gdt_and_idt(bits32, SegValue(cs), SegValue(ds),
                       SegValue(ss), SegValue(es));
  // Do NOT zero the LDT -- le_install_descriptors has already filled
  // our slots and flipping those to zero would lose them.

  // Install per-vector PM exception handlers for 0x00..0x1F.
  // Without these, a client #GP (or any other early fault) sends the
  // CPU to an uninstalled IDT gate and dosbox aborts with "Gate
  // Selector points to illegal descriptor".  Gate bitness MUST match
  // the entry obj's default size: 32-bit client -> 32-bit gates
  // (CB_IRETD pops 12 bytes) or 16-bit client -> 16-bit gates
  // (CB_IRET pops 6 bytes).  Mixing produces corrupt frames.
  const uint16_t cb_off = bits32 ? s_le_exc_cb32_off : s_le_exc_cb16_off;
  if (cb_off) {
    for (int v = 0; v < 0x20; ++v)
      write_idt_gate(v, PM_CB_SEL, cb_off, bits32);
  }
}

// Load an MZ .EXE at the given PSP segment (image goes at psp_seg + 0x10).
bool load_exe_at(const std::string &path, uint16_t psp_seg, InitialRegs &out) {
  const uint16_t load_seg = psp_seg + 0x10;
  const auto f = read_file(path);
  if (f.size() < 0x1C) {
    std::fprintf(stderr, "dosemu: %s too small to be an MZ .EXE\n", path.c_str());
    return false;
  }
  if (!((f[0] == 'M' && f[1] == 'Z') || (f[0] == 'Z' && f[1] == 'M'))) {
    std::fprintf(stderr, "dosemu: %s has no MZ signature\n", path.c_str());
    return false;
  }

  // Before handling as a pure MZ, check for an LE/LX "new exe" header
  // pointed at by MZ+0x3C.  If present, report the structure so the
  // failure is actionable rather than silent.  Actually executing the
  // LE image is a future-session item -- requires installing PM
  // descriptors for each object, applying fixups, and entering PM at
  // the entry point with the right CS:EIP/SS:ESP.
  // Auto-detect DOS-extender-bound MZ binaries: if the MZ stub
  // contains a signature of a bundled extender (DOS/4G, DOS/16M,
  // PMODE/W, ...), treat the whole file as a plain MZ and let the
  // stub's own code (the extender) handle the LE payload.  Only go
  // down our direct-LE-dispatch path for files where the stub is
  // clearly not doing anything meaningful (bare LE binaries like
  // LE_MIN.EXE, OS/2 executables running on DOS, etc.).
  bool extender_detected = false;
  if (f.size() >= 0x40) {
    const uint32_t lfanew = rdd(f, 0x3C);
    if (lfanew > 0x40 && lfanew <= f.size()) {
      // Scan the MZ stub for well-known extender signatures.
      const uint8_t *stub = f.data();
      const size_t slen = std::min<size_t>(lfanew, 4096);
      auto has = [&](const char *s) -> bool {
        const size_t n = std::strlen(s);
        for (size_t i = 0; i + n <= slen; ++i)
          if (std::memcmp(stub + i, s, n) == 0) return true;
        return false;
      };
      if (has("DOS/4G") || has("DOS/16M") || has("PMODE/W")) {
        extender_detected = true;
      }
    }
  }
  if (extender_detected) {
    s_extender_bound = true;
    // Fall through to the MZ loader below.
  }
  if (!extender_detected
      && f.size() >= 0x40
      && !dosemu::g_debug.le_as_mz) {
    const uint32_t le_off = rdd(f, 0x3C);
    if (le_off != 0 && le_off + 2 <= f.size()
        && ((f[le_off] == 'L' && f[le_off + 1] == 'E') ||
            (f[le_off] == 'L' && f[le_off + 1] == 'X'))) {
      load_le_inspect(path, f, le_off);
      std::vector<LeObject> objects;
      if (!s_mcb_initialised) mcb_init();
      if (!le_load_objects(f, le_off, objects)) {
        std::fprintf(stderr,
            "dosemu: %s is LE/LX but le_load_objects failed\n",
            path.c_str());
        for (auto &o : objects) {
          if (o.host_seg) mcb_free(o.host_seg);
          else if (o.host_base) pm_free(o.host_base);
        }
        return false;
      }
      std::fprintf(stderr, "dosemu: LE objects loaded to host segments:\n");
      for (size_t i = 0; i < objects.size(); ++i) {
        std::fprintf(stderr,
                     "  obj %zu: host=0x%08x size=0x%x %s %s\n",
                     i + 1, objects[i].host_base, objects[i].virt_size,
                     objects[i].is_code ? "CODE" : "DATA",
                     objects[i].is_big ? "32-bit" : "16-bit");
      }
      // Descriptors first: le_apply_fixups needs each object's
      // ldt_sel populated for selector-bearing fixup types
      // (0x02/0x03/0x06) to resolve.
      if (!le_install_descriptors(objects)) {
        std::fprintf(stderr, "dosemu: LE descriptor install failed\n");
        return false;
      }
      le_apply_fixups(f, le_off, objects);

      // Extract entry/stack selectors from the populated LDT.
      const uint32_t entry_obj_1 = rdd(f, le_off + 0x18);
      const uint32_t entry_eip   = rdd(f, le_off + 0x1C);
      const uint32_t stack_obj_1 = rdd(f, le_off + 0x20);
      const uint32_t stack_esp   = rdd(f, le_off + 0x24);
      const uint32_t auto_obj_1  = rdd(f, le_off + 0x94);
      if (entry_obj_1 == 0 || entry_obj_1 > objects.size()
          || stack_obj_1 == 0 || stack_obj_1 > objects.size()) {
        std::fprintf(stderr, "dosemu: LE entry_obj=%u stack_obj=%u out of range\n",
                     entry_obj_1, stack_obj_1);
        return false;
      }
      const LeObject &eo = objects[entry_obj_1 - 1];
      const LeObject &so = objects[stack_obj_1 - 1];
      const uint16_t ds_sel = (auto_obj_1 && auto_obj_1 <= objects.size())
          ? objects[auto_obj_1 - 1].ldt_sel : so.ldt_sel;

      std::fprintf(stderr,
          "dosemu: LE entry CS=%04x:EIP=%08x SS=%04x:ESP=%08x DS=%04x\n",
          eo.ldt_sel, entry_eip, so.ldt_sel, stack_esp, ds_sel);

      // Stage GDT/IDT so dosemu_startup just has to flip CR0 and jump.
      // Gate bitness matches entry object's BIG flag -- mismatched
      // bitness produces corrupt frames on exceptions or INTs.
      le_launch_pm_prep(eo.is_big);

      out.is_pm    = true;
      out.is_32bit = eo.is_big;
      out.cs       = eo.ldt_sel;
      out.ss       = so.ldt_sel;
      out.pm_ds    = ds_sel;
      out.pm_eip   = entry_eip;
      out.pm_esp   = stack_esp;
      // Fill the RM-side fields with something sane; they're not
      // consumed on the is_pm path but gcc will warn about unused-init.
      out.ip = 0;
      out.sp = 0;
      out.ax = 0;
      // Objects' descriptor slots + host memory stay live for the
      // duration of the LE program.  They will leak when the program
      // exits (acceptable for now -- process exit reclaims everything).
      return true;
    }
  }

  const uint16_t bytes_in_last_page = rdw(f, 0x02);
  const uint16_t pages_total        = rdw(f, 0x04);
  const uint16_t reloc_count        = rdw(f, 0x06);
  const uint16_t header_paragraphs  = rdw(f, 0x08);
  const uint16_t init_ss            = rdw(f, 0x0E);
  const uint16_t init_sp            = rdw(f, 0x10);
  const uint16_t init_ip            = rdw(f, 0x14);
  const uint16_t init_cs            = rdw(f, 0x16);
  const uint16_t reloc_offset       = rdw(f, 0x18);

  const size_t header_size_bytes = static_cast<size_t>(header_paragraphs) * 16;
  size_t total_image_bytes = static_cast<size_t>(pages_total) * 512;
  if (bytes_in_last_page) total_image_bytes -= (512 - bytes_in_last_page);
  if (total_image_bytes < header_size_bytes || total_image_bytes > f.size()) {
    std::fprintf(stderr, "dosemu: %s header describes %zu bytes, file has %zu\n",
                 path.c_str(), total_image_bytes, f.size());
    return false;
  }

  const size_t image_bytes = total_image_bytes - header_size_bytes;
  const PhysPt load_addr   = load_seg * 16;
  for (size_t i = 0; i < image_bytes; ++i)
    mem_writeb(load_addr + i, f[header_size_bytes + i]);

  // Apply relocations: each entry is (offset, segment); the word at
  // (load_seg + segment):offset needs load_seg added.
  for (uint16_t i = 0; i < reloc_count; ++i) {
    const size_t entry = reloc_offset + i * 4u;
    if (entry + 3 >= f.size()) {
      std::fprintf(stderr, "dosemu: %s reloc %u out of bounds\n", path.c_str(), i);
      return false;
    }
    const uint16_t r_off = rdw(f, entry);
    const uint16_t r_seg = rdw(f, entry + 2);
    const PhysPt  target = (load_seg + r_seg) * 16 + r_off;
    const uint16_t val   = mem_readb(target) | (mem_readb(target + 1) << 8);
    const uint16_t fixed = static_cast<uint16_t>(val + load_seg);
    mem_writeb(target,     static_cast<uint8_t>(fixed & 0xFF));
    mem_writeb(target + 1, static_cast<uint8_t>((fixed >> 8) & 0xFF));
  }

  out = {
    static_cast<uint16_t>(load_seg + init_cs),
    init_ip,
    static_cast<uint16_t>(load_seg + init_ss),
    init_sp,
    0,
  };
  return true;
}

// Dispatch .COM vs .EXE by extension (case-insensitive).  load_program
// loads at the top-level PSP; load_program_at loads a child at a
// caller-chosen PSP segment (AH=4Bh).
bool iends_with(const std::string &path, const char *suffix) {
  size_t n = std::strlen(suffix);
  if (path.size() < n) return false;
  for (size_t i = 0; i < n; ++i) {
    char a = std::tolower(static_cast<unsigned char>(path[path.size() - n + i]));
    char b = std::tolower(static_cast<unsigned char>(suffix[i]));
    if (a != b) return false;
  }
  return true;
}
bool load_program_at(const std::string &path, uint16_t psp_seg,
                     InitialRegs &out) {
  // Prefer content sniffing over extension: FreeCOM ships as
  // COMMAND.COM but is actually MZ format (> 64KB).  DOS loaders
  // check the magic bytes, not the extension.
  std::FILE *f = std::fopen(path.c_str(), "rb");
  if (f) {
    uint8_t magic[2] = {0, 0};
    std::fread(magic, 1, 2, f);
    std::fclose(f);
    if (magic[0] == 'M' && magic[1] == 'Z')
      return load_exe_at(path, psp_seg, out);
  }
  if (iends_with(path, ".exe")) return load_exe_at(path, psp_seg, out);
  return load_com_at(path, psp_seg, out);
}
bool load_program(const std::string &path, InitialRegs &out) {
  return load_program_at(path, PSP_SEG, out);
}

// Fill the env block at ENV_SEG: sequence of "KEY=value\0" strings, an
// extra NUL to terminate the list, uint16 count (1), then the program
// path as ASCIIZ.  Returns false if the block would overflow ENV_BYTES.
bool build_env_block(const std::string &program_path) {
  const PhysPt base = ENV_SEG * 16u;
  size_t off = 0;

  auto put_string = [&](const std::string &s) {
    for (char c : s) {
      if (off >= ENV_BYTES - 1) return false;
      mem_writeb(base + off++, static_cast<uint8_t>(c));
    }
    mem_writeb(base + off++, 0);
    return true;
  };

  // Minimal DOS env: COMSPEC + PATH + a few host env vars that are
  // commonly expected (HOME, USER, TMPDIR).
  if (!put_string("COMSPEC=C:\\COMMAND.COM")) return false;
  if (!put_string("PATH=C:\\"))               return false;
  for (const char *key : {"HOME", "USER", "TMPDIR", "LANG", "TZ", "DJGPP"}) {
    const char *val = std::getenv(key);
    if (val && *val) {
      std::string entry = std::string(key) + "=" + val;
      if (entry.size() > 200) continue;   // keep things sane
      if (!put_string(entry)) return false;
    }
  }
  // End-of-list marker (extra NUL).
  if (off >= ENV_BYTES) return false;
  mem_writeb(base + off++, 0);
  // argc = 1 (the program itself).
  if (off + 2 > ENV_BYTES) return false;
  mem_writeb(base + off++, 1);
  mem_writeb(base + off++, 0);
  // argv[0] as ASCIIZ.  Use the DOS-style uppercase of the full path
  // relative to the host's cwd (which our C: drive is mounted at).
  // The go32-v2 stub opens this file to load the COFF payload -- just
  // the basename wouldn't find programs in subdirectories.
  std::string argv0 = "C:\\";
  std::string rel = program_path;
  // Strip a leading "./" (relative) or "/" (absolute) -- we're joining
  // onto "C:\" which already has the root backslash.
  if (rel.size() >= 2 && rel[0] == '.' && (rel[1] == '/' || rel[1] == '\\'))
    rel.erase(0, 2);
  else if (!rel.empty() && (rel[0] == '/' || rel[0] == '\\'))
    rel.erase(0, 1);
  for (auto &c : rel) {
    if (c == '/') c = '\\';
    else c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  argv0 += rel;
  return put_string(argv0);
}

// Build a minimal PSP at PSP_SEG:0000.  Populates INT 20h, command tail at
// offset 80h, and the env-segment pointer at offset 2Ch.
void build_psp(const std::string &program_path,
               const std::vector<std::string> &args) {
  const PhysPt psp = PSP_SEG * 16;
  for (int i = 0; i < 256; ++i) mem_writeb(psp + i, 0);

  // INT 20h at offset 0x00 (CD 20).
  mem_writeb(psp + 0x00, 0xCD);
  mem_writeb(psp + 0x01, 0x20);

  // Top-of-memory segment at offset 02h.  DOS programs read this to know
  // how much RAM they have; setting it to the top of conventional memory
  // (0xA000) reports 640K - PSP_SEG = ~639K as the arena size.
  mem_writeb(psp + 0x02, 0x00);
  mem_writeb(psp + 0x03, 0xA0);

  // PSP[0x18..0x2B] = 20-byte Job File Table (handle -> SFT index).
  // Initialize to 0xFF (= "closed") to be honest, then overwrite the
  // standard handles 0..4 with identity indices.  DJGPP's fstat walks
  // this via PSP[0x34] (far pointer).
  for (int i = 0x18; i <= 0x2B; i++) mem_writeb(psp + i, 0xFF);
  mem_writeb(psp + 0x18, 0x00); // stdin
  mem_writeb(psp + 0x19, 0x01); // stdout
  mem_writeb(psp + 0x1A, 0x02); // stderr
  mem_writeb(psp + 0x1B, 0x03); // aux
  mem_writeb(psp + 0x1C, 0x04); // prn

  // PSP[0x32] = maximum number of open handles (word).
  mem_writeb(psp + 0x32, 20);
  mem_writeb(psp + 0x33, 0);

  // PSP[0x34..0x37] = far pointer to the JFT (above).  Points at PSP:0x18.
  mem_writeb(psp + 0x34, 0x18);     // offset lo
  mem_writeb(psp + 0x35, 0x00);     // offset hi
  mem_writeb(psp + 0x36, PSP_SEG & 0xFF);
  mem_writeb(psp + 0x37, (PSP_SEG >> 8) & 0xFF);

  // Environment segment at offset 2Ch.
  build_env_block(program_path);
  mem_writeb(psp + 0x2C, ENV_SEG & 0xFF);
  mem_writeb(psp + 0x2D, (ENV_SEG >> 8) & 0xFF);

  // Command tail: " arg1 arg2 ..." + 0x0D.  Arguments containing
  // whitespace are quoted so DOS programs that split the tail on
  // spaces (e.g. DJGPP's crt0 argv parser) reconstruct the same
  // argv we got from the host shell.  Any existing double-quote in
  // the arg is backslash-escaped -- DJGPP's parser honors this.
  std::string tail;
  for (const auto &a : args) {
    tail += ' ';
    const bool needs_quote = a.find_first_of(" \t") != std::string::npos;
    if (needs_quote) tail += '"';
    for (char c : a) {
      if (c == '"' || c == '\\') tail += '\\';
      tail += c;
    }
    if (needs_quote) tail += '"';
  }
  if (tail.size() > 126) tail.resize(126);
  mem_writeb(psp + 0x80, static_cast<uint8_t>(tail.size()));
  for (size_t i = 0; i < tail.size(); ++i)
    mem_writeb(psp + 0x81 + i, static_cast<uint8_t>(tail[i]));
  mem_writeb(psp + 0x81 + tail.size(), 0x0D);
}

void dosemu_startup() {
  // Override dosbox's INT 21h callback.  The CALLBACK_HandlerObject is a
  // local: its destructor runs when this function returns, which is still
  // inside control->StartUp() -- dosbox's callback tables are alive and the
  // Uninstall path works cleanly.  A global-lifetime object would destruct
  // after control.reset() and crash on a freed callback table.
  CALLBACK_HandlerObject int21_cb;
  int21_cb.Install(&dosemu_int21, CB_INT21, "dosemu Int 21");
  int21_cb.Set_RealVec(0x21);
  // Capture the callback's RM address so the DPMI entry can route PM
  // INT 21h through a GDT/IDT gate targeting the same native-call bytes.
  {
    const RealPt rp = int21_cb.Get_RealPointer();
    s_int21_cb_seg = static_cast<uint16_t>((rp >> 16) & 0xFFFF);
    s_int21_cb_off = static_cast<uint16_t>(rp & 0xFFFF);
  }

  // "Stop" callback for AX=0302: a no-op native handler whose only
  // purpose is to be an IRET target the mode-switch dispatcher pushes
  // on the RM stack before jumping to the client's procedure.  When
  // the procedure IRETs, the CPU lands at this callback's stub (FE 38
  // ... CF) whose native dispatch returns CBRET_STOP -- the same
  // "unwind the nested RunMachine" signal CALLBACK_RunRealInt/Far
  // uses internally.
  CALLBACK_HandlerObject rm_stop_cb;
  rm_stop_cb.Install(&dosemu_rm_stop, CB_IRET, "dosemu RM stop (AX=0302)");
  s_rm_stop_ptr = rm_stop_cb.Get_RealPointer();

  CALLBACK_HandlerObject noop_retf_cb;
  noop_retf_cb.Install(&dosemu_noop_retf, CB_RETF,
                       "dosemu no-op RETF (AX=0305/0306)");
  s_noop_retf_ptr = noop_retf_cb.Get_RealPointer();

  // 32 RM callback slots for AX=0303.  Each is a CB_RETF callback
  // whose FE 38 dispatches to its dedicated rmcb_N trampoline; the
  // trampoline calls do_rm_callback(N) which handles the mode switch.
  // Local array: destructs on dosemu_startup's return, same lifetime
  // pattern as the other callbacks above.
  CALLBACK_HandlerObject rmcb_objs[RM_CALLBACK_COUNT];
  for (int i = 0; i < RM_CALLBACK_COUNT; ++i) {
    char name[32];
    std::snprintf(name, sizeof(name), "dosemu RM callback %d", i);
    rmcb_objs[i].Install(s_rmcb_table[i], CB_RETF, name);
    s_rm_callbacks[i].rm_addr = rmcb_objs[i].Get_RealPointer();
    s_rm_callbacks[i].allocated = false;
  }

  // Second callback: same native handler, but the dosbox-emitted stub
  // ends in IRETD (`66 CF`) so it can correctly unwind the 12-byte
  // frame a 32-bit IDT gate pushes.  Used only by the 32-bit DPMI PM
  // path; real-mode INT 21h still goes through CB_INT21 above.  The
  // stub runs in the 16-bit CB_SEG selector (0x28): `66 CF` in 16-bit
  // CS is IRETD, same bytes in a 32-bit CS would be plain IRET, so we
  // deliberately keep the selector 16-bit and let the 66 prefix force
  // the 32-bit pop.
  CALLBACK_HandlerObject int21_cb32;
  int21_cb32.Install(&dosemu_int21_bits32, CB_IRETD, "dosemu Int 21 (32-bit PM)");
  {
    const RealPt rp = int21_cb32.Get_RealPointer();
    s_int21_cb32_seg = static_cast<uint16_t>((rp >> 16) & 0xFFFF);
    s_int21_cb32_off = static_cast<uint16_t>(rp & 0xFFFF);
  }

  // INT 31h callbacks: real-mode IVT entry + two PM entry points (16-bit
  // IRET and 32-bit IRETD), wired into the PM IDT by dosemu_dpmi_entry.
  CALLBACK_HandlerObject int31_cb;
  int31_cb.Install(&dosemu_int31, CB_IRET, "dosemu Int 31 (DPMI)");
  int31_cb.Set_RealVec(0x31);
  {
    const RealPt rp = int31_cb.Get_RealPointer();
    s_int31_cb_off = static_cast<uint16_t>(rp & 0xFFFF);
  }
  CALLBACK_HandlerObject int31_cb32;
  int31_cb32.Install(&dosemu_int31_bits32, CB_IRETD, "dosemu Int 31 (32-bit PM)");
  {
    const RealPt rp = int31_cb32.Get_RealPointer();
    s_int31_cb32_off = static_cast<uint16_t>(rp & 0xFFFF);
  }

  // LE exception handlers: one 16-bit + one 32-bit callback shared
  // by all 32 vectors.  Used by the LE loader's direct-PM-entry path
  // (le_launch_pm_prep) to catch faults before the client has a
  // chance to install real handlers.
  CALLBACK_HandlerObject le_exc_cb32, le_exc_cb16;
  le_exc_cb32.Install(&dosemu_le_exc_any32, CB_IRETD,
                      "dosemu LE exception (32-bit)");
  s_le_exc_cb32_off = static_cast<uint16_t>(
      le_exc_cb32.Get_RealPointer() & 0xFFFF);
  le_exc_cb16.Install(&dosemu_le_exc_any16, CB_IRET,
                      "dosemu LE exception (16-bit)");
  s_le_exc_cb16_off = static_cast<uint16_t>(
      le_exc_cb16.Get_RealPointer() & 0xFFFF);

  // Per-vector PM exception trampolines (32-bit) + user_exception_return.
  // Used by the DPMI path: AX=0203 records the user's sel:off; the
  // IDT gate (installed at DPMI entry) points at the trampoline, which
  // builds the CWSDPMI-style 32-byte exception frame before jumping to
  // the user handler.  When the user handler LRETs, it lands in the
  // user_exception_return trampoline which restores outer SS:ESP and
  // IRETDs back to the faulting instruction.  Required by DJGPP and
  // anything else that was coded against CWSDPMI's frame layout.
  static CALLBACK_HandlerObject pm_exc_cb_objs[32];
  for (int v = 0; v < 32; ++v) {
    char name[48];
    std::snprintf(name, sizeof(name), "dosemu PM exc vec %d", v);
    pm_exc_cb_objs[v].Install(s_pm_exc_tramps[v], CB_IRETD, name);
    s_pm_exc_cb32_off[v] = static_cast<uint16_t>(
        pm_exc_cb_objs[v].Get_RealPointer() & 0xFFFF);
  }
  static CALLBACK_HandlerObject pm_exc_ret_cb;
  pm_exc_ret_cb.Install(&dosemu_pm_exc_ret, CB_IRETD,
                        "dosemu PM exc return trampoline");
  s_pm_exc_ret_off = static_cast<uint16_t>(
      pm_exc_ret_cb.Get_RealPointer() & 0xFFFF);

  // INT 2Fh handler for DPMI detection (stage 2).  Reports DPMI present
  // with a real-mode entry point that currently fails the mode switch.
  CALLBACK_HandlerObject dpmi_entry_cb;
  dpmi_entry_cb.Install(&dosemu_dpmi_entry, CB_RETF,
                        "dosemu DPMI entry (stage 2 stub)");
  const RealPt entry_addr = dpmi_entry_cb.Get_RealPointer();
  s_dpmi_entry_seg = static_cast<uint16_t>((entry_addr >> 16) & 0xFFFF);
  s_dpmi_entry_off = static_cast<uint16_t>(entry_addr & 0xFFFF);
  CALLBACK_HandlerObject int2f_cb;
  int2f_cb.Install(&dosemu_int2f, CB_IRET, "dosemu Int 2F (DPMI detect)");
  int2f_cb.Set_RealVec(0x2F);

  // XMS driver entry point -- returned to clients via INT 2F/4310h.
  // Uses CB_RETF (far-call entry, ends with RETF) since XMS clients
  // far-call the driver, not INT it.
  CALLBACK_HandlerObject xms_cb;
  xms_cb.Install(&dosemu_xms_driver, CB_RETF, "dosemu XMS driver");
  {
    const RealPt rp = xms_cb.Get_RealPointer();
    s_xms_driver_seg = (rp >> 16) & 0xFFFF;
    s_xms_driver_off = rp & 0xFFFF;
  }

  // INT 16h keyboard -- plumbed to host stdin so interactive DOS programs
  // (deltree, debug, etc.) can read a keypress via the BIOS path.
  s_int16_peek = -1;
  CALLBACK_HandlerObject int16_cb;
  int16_cb.Install(&dosemu_int16, CB_IRET, "dosemu Int 16 (BIOS kbd)");
  int16_cb.Set_RealVec(0x16);

  build_psp(s_program, s_args);

  InitialRegs ir;
  if (!load_program(s_program, ir)) {
    s_exit_code = 1;
    shutdown_requested = true;
    return;
  }

  if (ir.is_pm) {
    // LE entry path.  load_exe_at has already:
    //   - allocated per-object host memory + applied fixups
    //   - installed one LDT descriptor per object
    //   - run pm_setup_gdt_and_idt to populate GDT/IDT in memory and
    //     issue CPU_LGDT/CPU_LIDT (still in RM)
    // We finish the switch here: flip CR0.PE, activate the LDT, load
    // the data/stack selectors, then CPU_JMP into the LE entry.
    CPU_SET_CRX(0, 0x00000001);   // PE=1
    CPU_LLDT(PM_LDT_SEL);
    CPU_SetSegGeneral(ds, ir.pm_ds);
    CPU_SetSegGeneral(es, ir.pm_ds);
    CPU_SetSegGeneral(ss, ir.ss);
    reg_esp = ir.pm_esp;
    std::fprintf(stderr,
        "dosemu: LE entering PM: CS=%04x:EIP=%08x SS=%04x:ESP=%08x "
        "DS=%04x ES=%04x D=%u\n",
        ir.cs, ir.pm_eip, ir.ss, ir.pm_esp, ir.pm_ds, ir.pm_ds,
        ir.is_32bit ? 1 : 0);
    CPU_JMP(ir.is_32bit, ir.cs, ir.pm_eip, 0);
    DOSBOX_RunMachine();
    return;
  }

  // DS and ES always point at the PSP at program entry; CS/IP/SS/SP come
  // from the loader (differ between .COM and .EXE).
  SegSet16(cs, ir.cs);
  SegSet16(ds, PSP_SEG);
  SegSet16(es, PSP_SEG);
  SegSet16(ss, ir.ss);
  reg_eip = ir.ip;
  reg_sp  = ir.sp;
  reg_ax  = ir.ax;

  DOSBOX_RunMachine();
}

} // namespace

int run_program(const dosemu::Config &cfg) {
  s_program = cfg.program;
  s_args    = cfg.args;
  s_exit_code = 0;
  s_handles.clear();
  s_drives.clear();
  s_drive_cwd.clear();
  s_mcb_initialised = false;           // re-init chain on first allocation
  s_file_mappings = cfg.file_mappings;
  s_dta_linear    = PSP_SEG * 16 + 0x80;
  for (auto &kv : s_finds) close_find_state(kv.second);
  s_finds.clear();

  // Populate drives from .cfg; fall back to C: = process CWD when empty.
  for (const auto &d : cfg.drives) s_drives[d.letter] = d.host_path;
  if (s_drives.empty()) {
    char cwd[4096];
    if (getcwd(cwd, sizeof(cwd))) s_drives['C'] = cwd;
    else                          s_drives['C'] = ".";
  }
  s_current_drive = s_drives.begin()->first;

  // Default mode.  Auto resolves per-file by extension; Text/Binary are
  // unconditional.
  s_default_mode = cfg.default_mode;

  loguru::g_stderr_verbosity = (cfg.verbose >= 2) ? loguru::Verbosity_INFO
                              : (cfg.verbose >= 1) ? loguru::Verbosity_WARNING
                                                   : loguru::Verbosity_ERROR;

  static const char *dummy_argv[] = {"dosemu", nullptr};
  auto cmdline = std::make_unique<CommandLine>(1, dummy_argv);
  // ::Config is dosbox's Config (in the global namespace); without the
  // leading :: the nested dosemu::Config shadows it.
  control      = std::make_unique<::Config>(cmdline.get());

  if (cfg.headless) {
    // On macOS, SDL's "offscreen" driver is EGL-based and cannot init without
    // a real GL loader -- it fails to create any window even in texture mode.
    // "dummy" creates no window at all, which is what we want anyway.
#if defined(__APPLE__)
    setenv("SDL_VIDEODRIVER", "dummy", 1);
#else
    setenv("SDL_VIDEODRIVER", "offscreen", 1);
#endif
    setenv("SDL_AUDIODRIVER", "dummy", 1);
  }

  try {
    InitConfigDir();

    // Replicate upstream sdl_main's pre-DOSBOX_Init setup so that the [sdl]
    // section is registered.  Without this, dosbox's GFX code crashes in
    // initialize_vsync_settings when the video timer fires.
    messages_add_command_line();
    DOS_Locale_AddMessages();
    RENDER_AddMessages();
    messages_add_sdl();
    config_add_sdl();

    DOSBOX_Init();
    control->ParseConfigFiles(GetConfigDir());

    // Headless-friendly overrides.  SDL's offscreen driver cannot grab the
    // mouse, and nothing displays anyway; disable mouse capture and the
    // DOS mouse driver, mute the mixer, and set sbtype=none so init
    // doesn't trip on audio devices that don't exist in this sandbox.
    if (cfg.headless) {
      if (auto *s = control->GetSection("mouse")) {
        s->HandleInputline("mouse_capture=nomouse");
        s->HandleInputline("dos_mouse_driver=false");
      }
      if (auto *s = control->GetSection("mixer"))   s->HandleInputline("nosound=true");
      if (auto *s = control->GetSection("sblaster")) s->HandleInputline("sbtype=none");
      // Avoid the OpenGL probe in sdlmain.cpp:set_output -- it fails under
      // SDL's offscreen driver on macOS, and we render to nothing anyway.
      if (auto *s = control->GetSection("sdl"))     s->HandleInputline("output=texture");
    }
    // Keep the interpreter core for tracing.  dosbox auto-switches
    // to the dynamic JIT when a program enters PM; that's fine for
    // speed but bypasses core_normal where DOSEMU_CPU_TRACE is
    // instrumented.  Force core=normal when the trace flag is set
    // and warn so the user knows their cycle budget just shrank.
    if (dosemu::g_debug.cpu_trace) {
      if (auto *s = control->GetSection("cpu")) s->HandleInputline("core=normal");
      std::fprintf(stderr,
          "dosemu: DOSEMU_CPU_TRACE is set -- forcing core=normal "
          "(the dynamic JIT cores bypass the trace hooks); "
          "execution will be measurably slower than normal.\n");
    }

    if (SDL_Init(SDL_INIT_AUDIO | SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
      std::fprintf(stderr, "dosemu: SDL_Init failed: %s\n", SDL_GetError());
      return -1;
    }

    control->ParseEnv();
    control->Init();
    control->SetStartUp(&dosemu_startup);
    control->StartUp();
  } catch (const std::exception &e) {
    std::fprintf(stderr, "dosemu: bring-up threw: %s\n", e.what());
    return -1;
  } catch (char *msg) {
    std::fprintf(stderr, "dosemu: bring-up failed: %s\n", msg);
    return -1;
  }

  return s_exit_code;
}

} // namespace dosemu::bridge

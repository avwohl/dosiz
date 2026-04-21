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

// .EXE image goes one paragraph past the PSP (PSP is 256 bytes = 16 paras).
constexpr uint16_t EXE_LOAD_SEG     = PSP_SEG + 0x10;

// Environment block sits in its own segment below the PSP.  Holds strings
// "VAR=value\0" separated by NUL, terminated by an extra NUL, then a
// little-endian uint16 count followed by argv[0] as ASCIIZ.
constexpr uint16_t ENV_SEG          = 0x0050;  // physical 0x500, start of DOS data area
constexpr uint32_t ENV_BYTES        = 0x0800;  // reserve 2KB -- fits average env

// Initial register snapshot a loader returns to dosemu_startup.
struct InitialRegs {
  uint16_t cs, ip, ss, sp, ax;
};

// --- Run-time state, reset by run_com() ------------------------------------

std::string                 s_program;
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
};
std::map<uint32_t, FindState> s_finds;

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

  // Fall back to a case-insensitive scan of the parent directory on the
  // last segment only.  Intermediate dirs must exist exact-case today;
  // extending to every segment is straightforward if a real tool needs it.
  const size_t slash = literal.find_last_of('/');
  const std::string parent = (slash == std::string::npos) ? "."
                                                          : literal.substr(0, slash);
  const std::string target_u = upper(
      (slash == std::string::npos) ? literal : literal.substr(slash + 1));
  DIR *d = ::opendir(parent.c_str());
  if (!d) return literal;
  std::string found;
  while (struct dirent *ent = ::readdir(d)) {
    if (upper(ent->d_name) == target_u) { found = ent->d_name; break; }
  }
  ::closedir(d);
  if (!found.empty()) return parent + "/" + found;
  return literal;        // caller will get ENOENT from the open() call
}

// Simple case-insensitive glob with * and ? — enough for DOS patterns.
bool glob_match(const std::string &name_u, const std::string &pat_u) {
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
void dta_write_entry(const std::string &host_full, const std::string &mangled_name) {
  struct stat st{};
  ::stat(host_full.c_str(), &st);

  uint16_t dostime = 0, dosdate = 0;
  dos_time_from_unix(st.st_mtime, dosdate, dostime);

  // Zero reserved area (0x00..0x14).
  for (int i = 0; i < 0x15; ++i) mem_writeb(s_dta_linear + i, 0);
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
  dta_write_entry(st.host_dir + "/" + e.host_name, e.mangled_8_3);
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

void set_cf(bool val) {
  // Flip bit 0 (CF) on the FLAGS word the CPU will pop on IRET.  The
  // frame layout depends on how we were entered:
  //   real mode / 16-bit gate:  [SS:SP+4] = 16-bit FLAGS
  //   32-bit interrupt gate:    [SS:SP+8] = low word of 32-bit EFLAGS
  // `cpu.code.big` tracks the current CS descriptor's D flag, which
  // matches the gate bitness we installed.  `cpu.stack.big` selects
  // SP vs ESP as the frame pointer.  `SegPhys(ss)` returns the PM
  // descriptor base (or seg*16 in real mode) so this works in every
  // mode without conditioning on cpu.pmode.  dosbox's CALLBACK_SCF
  // assumes a real-mode SS and hits the wrong memory in PM -- we
  // hit that when DPMI stage 4 introduced INT 31h handlers that
  // actually depend on CF propagating back to the client.
  const PhysPt ss_base    = SegPhys(ss);
  const uint32_t sp_val   = cpu.stack.big ? reg_esp : reg_sp;
  const unsigned flags_off = cpu.code.big ? 8u : 4u;
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
constexpr uint16_t GDT_SEG    = 0x1800;       // physical 0x18000
constexpr uint16_t GDT_LIMIT  = 0x3F;         // 8 entries * 8 bytes - 1
constexpr uint16_t PM_CS_SEL   = 0x08;
constexpr uint16_t PM_DS_SEL   = 0x10;
constexpr uint16_t PM_SS_SEL   = 0x18;
constexpr uint16_t PM_ES_SEL   = 0x20;
constexpr uint16_t PM_CB_SEL   = 0x28;        // selector for dosbox's CB_SEG
constexpr uint16_t PM_LDT_SEL  = 0x30;        // GDT[6]: LDT descriptor itself
constexpr uint16_t PM_SHIM_SEL = 0x38;        // GDT[7]: 32-bit reflection shim area

// 32-bit PM reflection shims.  Each slot is an 8-byte 16-bit code
// sequence that re-invokes the dosbox native callback for a real-mode
// IVT vector and then does IRETD instead of plain IRET, so a 32-bit
// interrupt gate's 12-byte EIP/CS/EFLAGS frame is correctly unwound.
// Shim layout (6 meaningful bytes, padded to 8):
//   FE 38 cb_lo cb_hi 66 CF 90 90
// The `FE 38 LL HH` is dosbox's native-callback opcode with the cb_num
// copied from the existing RM stub the real-mode IVT points at.
constexpr uint16_t PM_SHIM_SEG        = 0x1C00;    // physical 0x1C000
constexpr uint16_t PM_SHIM_SLOT_BYTES = 8;
constexpr uint16_t PM_SHIM_TOTAL      = 256 * PM_SHIM_SLOT_BYTES;

constexpr uint16_t IDT_SEG    = 0x1A00;       // physical 0x1A000
constexpr uint16_t IDT_LIMIT  = 0x7FF;        // 256 entries * 8 bytes - 1

// LDT for DPMI client-allocated descriptors (INT 31h AX=0000/0001/0002).
// 256 entries * 8 bytes = 2KB, placed between the IDT and the MCB arena.
// Index 0 in the LDT is reserved null, so client selectors start at 0x000C
// (idx=1, TI=1, RPL=0 -- we run the client at ring 0 to keep the host
// simple; DPMI spec nominally wants RPL=3 but nothing we care about
// distinguishes the two in dosbox's PM core).
constexpr uint16_t LDT_SEG    = 0x1B00;       // physical 0x1B000
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

void write_gdt_descriptor(int idx, uint32_t base, uint32_t limit,
                          uint8_t access, bool bits32 = false) {
  const PhysPt p = GDT_SEG * 16u + idx * 8u;
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
  const PhysPt p = IDT_SEG * 16u + idx * 8u;
  mem_writeb(p + 0, off & 0xFF);
  mem_writeb(p + 1, (off >> 8) & 0xFF);
  mem_writeb(p + 2, sel & 0xFF);
  mem_writeb(p + 3, (sel >> 8) & 0xFF);
  mem_writeb(p + 4, 0);
  mem_writeb(p + 5, bits32 ? 0x8E : 0x86);      // present, DPL=0, int gate
  mem_writeb(p + 6, (off >> 16) & 0xFF);        // 32-bit offset high
  mem_writeb(p + 7, (off >> 24) & 0xFF);
}

Bitu dosemu_dpmi_entry() {
  // Called via FAR CALL from real-mode DPMI clients after AX=1687h.
  // Stack layout at entry:  [SP] = client IP, [SP+2] = client CS
  // (the stub's CB is 'retf', so we must leave those words there -- but we
  // overwrite CS with our PM code selector so retf takes us into PM).
  //
  // This is DPMI stage 3 -- mode switch only.  Stages 4-7 (LDT management,
  // INT 21h reflection from PM, linear memory allocation) are not yet
  // implemented, so most real DPMI clients (DJGPP, DOS4GW) will still fail
  // at the first interrupt after the switch.  The switch itself is
  // verifiable; what breaks after reveals the next piece to build.

  const uint16_t client_cs = mem_readb(SegValue(ss) * 16u + reg_sp + 2)
                           | (mem_readb(SegValue(ss) * 16u + reg_sp + 3) << 8);
  // Client IP is the word below (unused here -- we don't relocate it).

  // Build GDT: code, data, stack, es segments all cover the corresponding
  // 64K real-mode arena.  Access byte 0x9A = present, DPL=0, code,
  // readable; 0x92 = present, DPL=0, data, writable.
  // AX on entry: 0 = 16-bit PM, 1 = 32-bit PM.  Everything else about the
  // switch is identical except for the D flag on the code descriptor and
  // the IDT gate type.
  const bool bits32 = (reg_ax == 1);

  write_gdt_descriptor(0, 0,              0,      0);                 // null
  write_gdt_descriptor(1, client_cs * 16, 0xFFFF, 0x9A, bits32);      // code
  write_gdt_descriptor(2, SegValue(ds) * 16, 0xFFFF, 0x92);           // data
  write_gdt_descriptor(3, SegValue(ss) * 16, 0xFFFF, 0x92);           // stack
  write_gdt_descriptor(4, SegValue(es) * 16, 0xFFFF, 0x92);           // es
  // Selector 0x28: code segment covering dosbox's callback area
  // (CB_SEG=0xF000 -> base 0xF0000).  Always 16-bit: dosbox's callback
  // stub emits 16-bit instructions (FB/CF/CB), so a 32-bit client's
  // INT 21h hits a 16-bit compatibility sub-segment.
  write_gdt_descriptor(5, 0xF0000, 0xFFFF, 0x9A);                    // cb
  // GDT[6] = LDT descriptor.  Access byte 0x82 = present, DPL=0, system,
  // LDT type; granularity byte has D=0 (system descriptors ignore D but
  // keep the nibble clean).
  write_gdt_descriptor(6, LDT_SEG * 16u, LDT_BYTES - 1, 0x82);
  // GDT[7] = 32-bit reflection-shim code segment (always 16-bit CS --
  // the shim bytes use 16-bit encodings with a `66` prefix to force
  // IRETD).  Access byte 0x9A = present, DPL=0, code, readable.
  write_gdt_descriptor(7, PM_SHIM_SEG * 16u, PM_SHIM_TOTAL - 1, 0x9A);

  // Zero the LDT so every unallocated slot reads as a not-present
  // descriptor (access byte 0), then reset the in-use bitmap to match.
  for (uint32_t off = 0; off < LDT_BYTES; ++off)
    mem_writeb(LDT_SEG * 16u + off, 0);
  for (auto &b : s_ldt_in_use) b = 0;
  s_ldt_in_use[0] |= 0x01;   // slot 0 reserved (null)
  for (auto &c : s_seg2desc_cache) c = 0;

  CPU_LGDT(GDT_LIMIT, GDT_SEG * 16u);

  // Build IDT: zero all 256 entries, install INT 21h at the requested
  // bitness.  CLI'd by the fixture/client -- other vectors would fault.
  //
  // The stub offset differs by bitness: the 16-bit stub ends in plain
  // IRET (matches a 16-bit gate's 6-byte stack frame), the 32-bit stub
  // ends in IRETD (matches a 32-bit gate's 12-byte frame).  Pointing a
  // 32-bit gate at the 16-bit stub worked for the INT itself but faulted
  // on the way out -- IRET popped CS from the high half of EIP (=0, the
  // null descriptor).  Hence s_int21_cb32_off for 32-bit clients.
  const uint16_t int21_cb_off = bits32 ? s_int21_cb32_off : s_int21_cb_off;
  const uint16_t int31_cb_off = bits32 ? s_int31_cb32_off : s_int31_cb_off;
  for (int i = 0; i < 256; ++i) write_idt_gate(i, 0, 0, bits32);

  // PM→RM reflection for un-installed vectors.
  //
  // 16-bit path: point the PM IDT gate directly at the RM stub in
  // CB_SEG.  Its `CF` (16-bit IRET) correctly unwinds a 16-bit gate's
  // 6-byte frame.
  //
  // 32-bit path: the same `CF` can't unwind a 32-bit gate's 12-byte
  // frame, so we build a per-vector shim in PM_SHIM_SEG that re-does
  // the native callback and ends in `66 CF` (IRETD).  The shim's CS
  // (PM_SHIM_SEL) is 16-bit; the `66` prefix forces the trailing IRET
  // to pop 32 bits.  cb_num comes from mining the `FE 38 LL HH` bytes
  // inside the existing RM stub -- works because every dosbox callback
  // type (CB_IRET, CB_INT21, CB_IRET_STI, etc.) includes that same
  // 4-byte native-call invocation somewhere in its first handful of
  // bytes, possibly preceded by a 1-byte `FB` (STI).
  //
  // INT 21h/31h keep their dedicated gates below, so we skip them.
  // (CB_SEG is a dosbox macro = 0xF000.)
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
    // 32-bit: scan the first 5 bytes of the stub for `FE 38` and copy
    // the 2-byte cb_num that follows into our shim.
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
    const PhysPt shim = PM_SHIM_SEG * 16u + slot_off;
    mem_writeb(shim + 0, 0xFE);
    mem_writeb(shim + 1, 0x38);
    mem_writeb(shim + 2, cb_lo);
    mem_writeb(shim + 3, cb_hi);
    mem_writeb(shim + 4, 0x66);
    mem_writeb(shim + 5, 0xCF);
    mem_writeb(shim + 6, 0x90);
    mem_writeb(shim + 7, 0x90);
    write_idt_gate(v, PM_SHIM_SEL, slot_off, true);
  }

  if (int21_cb_off)
    write_idt_gate(0x21, PM_CB_SEL, int21_cb_off, bits32);
  if (int31_cb_off)
    write_idt_gate(0x31, PM_CB_SEL, int31_cb_off, bits32);
  CPU_LIDT(IDT_LIMIT, IDT_SEG * 16u);

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

Bitu dosemu_int2f() {
  if (reg_ax == 0x1687) {
    // DPMI detection response:
    //   AX = 0    (DPMI host present)
    //   BX = 1    (flags: bit 0 = 32-bit DPMI available)
    //   CL = 3    (CPU type: 386)
    //   DH:DL = 0:90h  (DPMI 0.90 -- what DJGPP expects)
    //   SI = 1    (paragraphs of private data required by host)
    //   ES:DI = real-mode entry-point for the switch.
    reg_ax = 0;
    reg_bx = 1;
    reg_cl = 3;
    reg_dh = 0;
    reg_dl = 0x5A;      // 0x5A = 90
    reg_si = 1;
    SegSet16(es, s_dpmi_entry_seg);
    reg_di = s_dpmi_entry_off;
    return CBRET_NONE;
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
RealPt s_rm_stop_ptr = 0;

// INT 31h (DPMI) — stage 4 subset.
//
//   AX=0400  Get DPMI version.
//   AX=0006  Get segment base address (BX=selector -> CX:DX=base).
//   AX=0007  Set segment base address (BX=selector, CX:DX=base).
//
// Everything else still returns CF=1 / AX=8001h ("unsupported DPMI
// function").  AX=0006/0007 read and write the base field of a GDT
// descriptor in the GDT we installed at GDT_SEG; selectors outside
// GDT_LIMIT return AX=8022h (invalid selector).  LDT management (LDT
// descriptor allocation) is still stage 4 proper and not wired up.
// Descriptor helpers shared by AX=0000/0001/0002 and AX=0006/0007.
// `selector_table_base` returns LDT_SEG*16 if TI=1, GDT_SEG*16 otherwise.
// `selector_is_valid` rejects selectors whose index is out of range for
// the chosen table (GDT: 8 entries, LDT: 256 entries), or slot 0 for
// LDT (reserved null).
inline PhysPt selector_table_base(uint16_t sel) {
  return (sel & 0x4) ? LDT_SEG * 16u : GDT_SEG * 16u;
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
                          uint8_t access, bool bits32 = false) {
  const PhysPt p = LDT_SEG * 16u + idx * 8u;
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
  switch (reg_ax) {

    case 0x0400: {  // Get DPMI version
      // Advertise DPMI 0.90 (matches the INT 2Fh/1687h detection response,
      // which reports DH:DL = 0:5Ah = "DPMI 0.90").
      reg_ax = 0x005A;         // major 0, minor 0x5A (= 90 decimal)
      reg_bx = 0x0002;         // flags: bit 1 = reflect real-mode INTs
      reg_cl = 3;              // CPU type: 386
      reg_dh = 0x08;           // master PIC base interrupt
      reg_dl = 0x70;           // slave PIC base interrupt
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
      for (uint16_t i = 0; i < count; ++i) {
        ldt_set(start + i, true);
        // Install a placeholder data descriptor: base 0, limit 0, access
        // 0x92 (present, DPL=0, data r/w).  The client is expected to
        // call AX=0007 (set base) + a size service before using it, so
        // a base-0/limit-0 slot is valid-but-empty.
        write_ldt_descriptor(start + i, 0, 0, 0x92);
      }
      reg_ax = (start << 3) | 0x04;      // TI=1, RPL=0
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0001: {  // Free LDT descriptor
      // Input: BX = selector.  Rejects GDT selectors (TI=0) and slot 0.
      if (!(reg_bx & 0x4) || !selector_is_valid(reg_bx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      const uint16_t idx = reg_bx >> 3;
      if (!ldt_bit(idx)) {
        reg_ax = 0x8022; set_cf(true); return CBRET_NONE;
      }
      ldt_set(idx, false);
      write_ldt_descriptor(idx, 0, 0, 0);   // mark not-present
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
        reg_ax = (s_seg2desc_cache[seg] << 3) | 0x04;
        set_cf(false);
        return CBRET_NONE;
      }
      const uint16_t idx = ldt_find_run(1);
      if (idx == 0) {
        reg_ax = 0x8011; set_cf(true); return CBRET_NONE;
      }
      ldt_set(idx, true);
      write_ldt_descriptor(idx, seg * 16u, 0xFFFF, 0x92);
      s_seg2desc_cache[seg] = idx;
      reg_ax = (idx << 3) | 0x04;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0003: {  // Get selector increment
      reg_ax = 8;
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
      write_ldt_descriptor(ldt_idx, data_seg * 16u, limit_bytes, 0x92);
      // Record the alias so AX=0002 on the same RM seg returns the
      // same selector (DPMI identity semantics).
      s_seg2desc_cache[data_seg] = ldt_idx;
      reg_ax = data_seg;
      reg_dx = (ldt_idx << 3) | 0x04;   // TI=1 (LDT), RPL=0
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
      const PhysPt p = LDT_SEG * 16u + idx * 8u;
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

    case 0x0204: {  // Get real-mode interrupt vector
      // Input:  BL = vector number
      // Output: CX:DX = real-mode seg:off from the IVT (physical 0..3FF)
      const PhysPt v = static_cast<uint32_t>(reg_bl) * 4u;
      reg_dx = mem_readw(v);
      reg_cx = mem_readw(v + 2);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0205: {  // Set real-mode interrupt vector
      // Input:  BL = vector, CX:DX = new seg:off.  Writes the real-mode
      // IVT directly.  A real DPMI host would track per-client hooks and
      // restore the IVT on client exit; our single-client model writes
      // through and lets the client own the vector.
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

    case 0x0501: {  // Allocate memory block (stage 6 minimal)
      // Input:  BX:CX = size in bytes (BX high, CX low)
      // Output: BX:CX = linear address of block
      //         SI:DI = handle (opaque; must round-trip to 0x0502/0x0503)
      // Error:  CF=1, AX=8012h (out of memory)
      //
      // We delegate to the real-mode MCB allocator: every byte we hand a
      // DPMI client comes from the same 0x2000-0xA000 paragraph arena DOS
      // memory management uses.  That caps allocations at ~600KB total --
      // fine for small DPMI clients, nowhere near enough for a real
      // 32-bit app.  Returning a linear address <1MB is legal per the
      // DPMI spec; real hosts typically allocate above 1MB but the spec
      // only requires that the address be valid for the client to
      // read/write.
      const uint32_t bytes = (static_cast<uint32_t>(reg_bx) << 16) | reg_cx;
      if (bytes == 0) {
        reg_ax = 0x8021;       // invalid value
        set_cf(true);
        return CBRET_NONE;
      }
      const uint32_t paras32 = (bytes + 15u) >> 4;
      if (paras32 > 0xFFFFu) {
        reg_ax = 0x8012;
        set_cf(true);
        return CBRET_NONE;
      }
      uint16_t largest = 0;
      const uint16_t data_seg = mcb_allocate(
          static_cast<uint16_t>(paras32), largest);
      if (data_seg == 0) {
        reg_ax = 0x8012;       // physical memory unavailable
        set_cf(true);
        return CBRET_NONE;
      }
      const uint32_t linear = static_cast<uint32_t>(data_seg) * 16u;
      reg_bx = (linear >> 16) & 0xFFFF;
      reg_cx = linear & 0xFFFF;
      // Handle = data_seg (16 bits, stashed in DI; SI=0).  0x0502/0x0503
      // recover the MCB by reading (handle - 1).
      reg_si = 0;
      reg_di = data_seg;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0502: {  // Free memory block
      // Input:  SI:DI = handle returned by 0x0501.
      // SI must be zero (we never hand out handles with SI != 0) and
      // DI must be a valid MCB data segment in our arena.
      if (reg_si != 0 || reg_di < MCB_ARENA_START || reg_di >= MCB_ARENA_END) {
        reg_ax = 0x8023;       // invalid handle
        set_cf(true);
        return CBRET_NONE;
      }
      if (!mcb_free(reg_di)) {
        reg_ax = 0x8023;
        set_cf(true);
        return CBRET_NONE;
      }
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

      // Snapshot PM state (segments + full register file + stack).
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
        // Scratch stack in the DOS data area.  The DPMI spec lets the
        // host provide one when the client leaves SS:SP zero.
        SegSet16(ss, 0x0050);
        reg_esp = 0x0F00;
      }

      CALLBACK_RunRealInt(intnum);

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

      set_cf(false);
      return CBRET_NONE;
    }

    case 0x0503: {  // Resize memory block
      // Input:  SI:DI = handle, BX:CX = new size in bytes.
      // Output: BX:CX = linear address of the (possibly-relocated) block.
      //         SI:DI = new handle.  We only resize in place, so handle
      //         and base are unchanged on success -- the spec allows a
      //         host to relocate on grow; we just fail instead.
      if (reg_si != 0 || reg_di < MCB_ARENA_START || reg_di >= MCB_ARENA_END) {
        reg_ax = 0x8023; set_cf(true); return CBRET_NONE;
      }
      const uint32_t new_bytes = (static_cast<uint32_t>(reg_bx) << 16) | reg_cx;
      if (new_bytes == 0) {
        reg_ax = 0x8021; set_cf(true); return CBRET_NONE;
      }
      const uint32_t new_paras32 = (new_bytes + 15u) >> 4;
      if (new_paras32 > 0xFFFFu) {
        reg_ax = 0x8012; set_cf(true); return CBRET_NONE;
      }
      uint16_t largest = 0;
      const uint16_t got = mcb_resize(reg_di,
          static_cast<uint16_t>(new_paras32), largest);
      if (got != new_paras32) {
        reg_ax = 0x8012;       // couldn't satisfy the new size
        set_cf(true);
        return CBRET_NONE;
      }
      const uint32_t linear = static_cast<uint32_t>(reg_di) * 16u;
      reg_bx = (linear >> 16) & 0xFFFF;
      reg_cx = linear & 0xFFFF;
      // SI:DI unchanged (in-place resize = same handle).
      set_cf(false);
      return CBRET_NONE;
    }

    default:
      reg_ax = 0x8001;
      set_cf(true);
      return CBRET_NONE;
  }
}

Bitu dosemu_int21() {
  if (std::getenv("DOSEMU_TRACE")) {
    std::fprintf(stderr, "[trace] AH=%02x AL=%02x BX=%04x CX=%04x DX=%04x\n",
                 reg_ah, reg_al, reg_bx, reg_cx, reg_dx);
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
      int fd = ::open(r.host_path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
      if (fd < 0) { return_error(0x05); break; }
      int h = allocate_handle(fd, r.text_mode);
      if (h < 0) { ::close(fd); return_error(0x04); break; }
      reg_ax = static_cast<uint16_t>(h);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x3D: {  // Open file; AL=mode, DS:DX=path.  Returns handle in AX.
      const std::string dos_path = read_dos_string(SegValue(ds), reg_dx);
      const Resolved    r        = resolve_path(dos_path);
      int flags = O_RDONLY;
      switch (reg_al & 0x07) {
        case 0: flags = O_RDONLY; break;
        case 1: flags = O_WRONLY; break;
        case 2: flags = O_RDWR;   break;
      }
      int fd = ::open(r.host_path.c_str(), flags);
      if (fd < 0) { return_error(0x02); break; }
      int h = allocate_handle(fd, r.text_mode);
      if (h < 0) { ::close(fd); return_error(0x04); break; }
      reg_ax = static_cast<uint16_t>(h);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x3E: {  // Close handle in BX.
      const auto it = s_handles.find(reg_bx);
      if (it == s_handles.end()) { return_error(0x06); break; }
      ::close(it->second.fd);
      s_handles.erase(it);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x3F: {  // Read from handle BX, CX bytes, into DS:DX.
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
        text_mode = it->second.text_mode;
        pending = &it->second.read_pending;
      }
      const PhysPt dst = SegPhys(ds) + reg_dx;
      uint16_t out = 0;
      while (out < reg_cx) {
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
          if (out < reg_cx) mem_writeb(dst + out++, '\n');
          else              *pending = '\n';
        } else {
          mem_writeb(dst + out++, byte);
        }
      }
      reg_ax = out;
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
      std::vector<uint8_t> buf(reg_cx);
      const PhysPt src = SegPhys(ds) + reg_dx;
      for (uint16_t i = 0; i < reg_cx; ++i) buf[i] = mem_readb(src + i);
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
      reg_ax = reg_cx;
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x42: {  // Seek handle BX; CX:DX = offset, AL = whence (0,1,2).
      auto it = s_handles.find(reg_bx);
      if (it == s_handles.end()) { return_error(0x06); break; }
      int whence = SEEK_SET;
      if (reg_al == 1) whence = SEEK_CUR;
      if (reg_al == 2) whence = SEEK_END;
      off_t off = (static_cast<int32_t>(reg_cx) << 16) | reg_dx;
      off_t pos = ::lseek(it->second.fd, off, whence);
      if (pos < 0) { return_error(0x19); break; }
      reg_ax = static_cast<uint16_t>(pos & 0xFFFF);
      reg_dx = static_cast<uint16_t>((pos >> 16) & 0xFFFF);
      it->second.read_pending = -1;  // invalidate text-mode read buffer
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

      // Evict any prior state on this DTA.
      s_finds.erase(s_dta_linear);

      const std::string host_dir = dir_part.empty()
          ? dos_to_host("") : dos_to_host(dir_part);
      FindState st;
      if (!build_find_state(host_dir, upper(pat_part), st)) {
        return_error(0x03); break;                   // path not found
      }
      if (!scan_next(st)) { return_error(0x12); break; }
      s_finds[s_dta_linear] = std::move(st);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x4F: {  // Find next: continues the DTA's saved state
      auto it = s_finds.find(s_dta_linear);
      if (it == s_finds.end()) { return_error(0x12); break; }
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

    case 0x4B: {  // Load-and-execute.  Real implementation needs a full
                  // child-process + CPU-state save/restore; stub returns
                  // "file not found" so well-behaved callers just give up.
      return_error(0x02);
      break;
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

    case 0x44: {  // IOCTL.  Only AL=0 (get device info) minimally supported.
      if (reg_al == 0x00) {
        // Report "file" (no device flag) for our handles.  Real DOS sets
        // bit 7 for character devices (stdin/stdout/con) -- many isatty
        // checks live here.
        if (reg_bx == 0 || reg_bx == 1 || reg_bx == 2) {
          reg_dx = 0x80 | (reg_bx & 0x07); // char device, handle-as-dev-number
        } else {
          reg_dx = 0; // regular file
        }
        set_cf(false);
        return CBRET_NONE;
      }
      return_error(0x01);  // invalid function
      break;
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
        reg_ax = 0x08;                   // insufficient memory
        reg_bx = largest;                // largest block available
        set_cf(true);
        return CBRET_NONE;
      }
      reg_ax = seg;
      set_cf(false);
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
      reg_ax = 0x08;
      reg_bx = largest;
      set_cf(true);
      return CBRET_NONE;
    }

    case 0x4C: {  // Exit with code AL
      s_exit_code = reg_al;
      shutdown_requested = true;
      return CBRET_STOP;
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
      return_error(0x01);  // invalid function
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
bool load_com(const std::string &path, InitialRegs &out) {
  const auto bytes = read_file(path);
  if (bytes.empty()) return false;
  if (bytes.size() > MAX_COM_SIZE) {
    std::fprintf(stderr, "dosemu: %s too large for .COM (%zu bytes)\n",
                 path.c_str(), bytes.size());
    return false;
  }
  const PhysPt load_addr = PSP_SEG * 16 + COM_ENTRY_OFFSET;
  for (size_t i = 0; i < bytes.size(); ++i)
    mem_writeb(load_addr + i, bytes[i]);
  out = {PSP_SEG, COM_ENTRY_OFFSET, PSP_SEG, 0xFFFE, 0};
  return true;
}

// Helper: read a little-endian 16-bit word from a byte buffer.
uint16_t rdw(const std::vector<uint8_t> &b, size_t off) {
  return static_cast<uint16_t>(b[off]) |
         (static_cast<uint16_t>(b[off + 1]) << 8);
}

// Load an MZ .EXE.  Parses the 32+ byte header, strips it, places the image
// at EXE_LOAD_SEG:0, applies relocations by adding EXE_LOAD_SEG to each
// fix-up target, and returns the initial register state.
bool load_exe(const std::string &path, InitialRegs &out) {
  const auto f = read_file(path);
  if (f.size() < 0x1C) {
    std::fprintf(stderr, "dosemu: %s too small to be an MZ .EXE\n", path.c_str());
    return false;
  }
  if (!((f[0] == 'M' && f[1] == 'Z') || (f[0] == 'Z' && f[1] == 'M'))) {
    std::fprintf(stderr, "dosemu: %s has no MZ signature\n", path.c_str());
    return false;
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
  const PhysPt load_addr   = EXE_LOAD_SEG * 16;
  for (size_t i = 0; i < image_bytes; ++i)
    mem_writeb(load_addr + i, f[header_size_bytes + i]);

  // Apply relocations: each entry is (offset, segment); the word at
  // (EXE_LOAD_SEG + segment):offset needs EXE_LOAD_SEG added.
  for (uint16_t i = 0; i < reloc_count; ++i) {
    const size_t entry = reloc_offset + i * 4u;
    if (entry + 3 >= f.size()) {
      std::fprintf(stderr, "dosemu: %s reloc %u out of bounds\n", path.c_str(), i);
      return false;
    }
    const uint16_t r_off = rdw(f, entry);
    const uint16_t r_seg = rdw(f, entry + 2);
    const PhysPt  target = (EXE_LOAD_SEG + r_seg) * 16 + r_off;
    const uint16_t val   = mem_readb(target) | (mem_readb(target + 1) << 8);
    const uint16_t fixed = static_cast<uint16_t>(val + EXE_LOAD_SEG);
    mem_writeb(target,     static_cast<uint8_t>(fixed & 0xFF));
    mem_writeb(target + 1, static_cast<uint8_t>((fixed >> 8) & 0xFF));
  }

  out = {
    static_cast<uint16_t>(EXE_LOAD_SEG + init_cs),
    init_ip,
    static_cast<uint16_t>(EXE_LOAD_SEG + init_ss),
    init_sp,
    0,
  };
  return true;
}

// Dispatch .COM vs .EXE by extension (case-insensitive).
bool load_program(const std::string &path, InitialRegs &out) {
  auto iends_with = [&](const char *suffix) {
    size_t n = std::strlen(suffix);
    if (path.size() < n) return false;
    for (size_t i = 0; i < n; ++i) {
      char a = std::tolower(static_cast<unsigned char>(path[path.size() - n + i]));
      char b = std::tolower(static_cast<unsigned char>(suffix[i]));
      if (a != b) return false;
    }
    return true;
  };
  if (iends_with(".exe")) return load_exe(path, out);
  return load_com(path, out);
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
  for (const char *key : {"HOME", "USER", "TMPDIR", "LANG"}) {
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
  // argv[0] as ASCIIZ.  Use the DOS-style uppercase of the last path segment.
  std::string argv0 = "C:\\";
  const size_t slash = program_path.find_last_of("/\\");
  std::string basename = (slash == std::string::npos) ? program_path
                                                      : program_path.substr(slash + 1);
  for (auto &c : basename) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  argv0 += basename;
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

  // Environment segment at offset 2Ch.
  build_env_block(program_path);
  mem_writeb(psp + 0x2C, ENV_SEG & 0xFF);
  mem_writeb(psp + 0x2D, (ENV_SEG >> 8) & 0xFF);

  // Command tail: " arg1 arg2 ..." + 0x0D
  std::string tail;
  for (const auto &a : args) {
    tail += ' ';
    tail += a;
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

  // Second callback: same native handler, but the dosbox-emitted stub
  // ends in IRETD (`66 CF`) so it can correctly unwind the 12-byte
  // frame a 32-bit IDT gate pushes.  Used only by the 32-bit DPMI PM
  // path; real-mode INT 21h still goes through CB_INT21 above.  The
  // stub runs in the 16-bit CB_SEG selector (0x28): `66 CF` in 16-bit
  // CS is IRETD, same bytes in a 32-bit CS would be plain IRET, so we
  // deliberately keep the selector 16-bit and let the 66 prefix force
  // the 32-bit pop.
  CALLBACK_HandlerObject int21_cb32;
  int21_cb32.Install(&dosemu_int21, CB_IRETD, "dosemu Int 21 (32-bit PM)");
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
  int31_cb32.Install(&dosemu_int31, CB_IRETD, "dosemu Int 31 (32-bit PM)");
  {
    const RealPt rp = int31_cb32.Get_RealPointer();
    s_int31_cb32_off = static_cast<uint16_t>(rp & 0xFFFF);
  }

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
    setenv("SDL_VIDEODRIVER", "offscreen", 1);
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

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

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <ctime>
#include <map>
#include <memory>
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

// Initial register snapshot a loader returns to dosemu_startup.
struct InitialRegs {
  uint16_t cs, ip, ss, sp, ax;
};

// --- Run-time state, reset by run_com() ------------------------------------

std::string                 s_program;
std::vector<std::string>    s_args;
int                         s_exit_code = 0;

// Bump allocator for AH=48/49/4A.  Host paragraphs above this segment are
// assumed free; allocations are carved in order and never actually freed.
// Good enough for C runtimes that malloc once at startup.
uint16_t                    s_mem_highwater = 0x2000;
constexpr uint16_t          MEM_CEILING     = 0x9FFF; // top of conventional mem

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

// Default translation mode for newly-opened files.  When true, AH=40h
// writes strip CR bytes and AH=3Fh reads expand LF to CR LF -- gives the
// DOS program classic CRLF semantics while the host file stays Unix-LF.
bool                        s_default_text_mode = false;

// Per-file / per-pattern mappings from cfg.file_mappings.
std::vector<FileMapping>    s_file_mappings;

// Disk Transfer Area.  DOS programs set this via AH=1Ah and read results of
// findfirst/findnext from it.  Default is PSP:0080h (128-byte area).
uint32_t                    s_dta_linear = PSP_SEG * 16 + 0x80;

// State for AH=4Eh/4Fh findfirst/findnext.  Keyed by DTA linear address so
// a program can juggle multiple DTAs.  DIR* stays open across findnext
// calls and closes when the scan runs out or the state is evicted.
struct FindState {
  DIR*        dir;
  std::string host_dir;
  std::string pattern_u;  // upper-cased glob (e.g. "*.TST")
};
std::map<uint32_t, FindState> s_finds;

void close_find_state(FindState &st) {
  if (st.dir) ::closedir(st.dir);
  st.dir = nullptr;
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

// Translate a DOS path (drive-letter + backslashes) to a host path.
// For this iteration, all drives resolve under a single base directory.
std::string dos_to_host(const std::string &dos_path) {
  std::string path = dos_path;
  char drive = s_current_drive;

  if (path.size() >= 2 && path[1] == ':') {
    drive = static_cast<char>(std::toupper(static_cast<unsigned char>(path[0])));
    path  = path.substr(2);
  }

  std::replace(path.begin(), path.end(), '\\', '/');

  auto it = s_drives.find(drive);
  if (it == s_drives.end()) {
    // Unknown drive -> treat path as relative to CWD.
    return path;
  }
  if (path.empty()) return it->second;
  if (path.front() == '/') return it->second + path;
  return it->second + "/" + path;
}

std::string upper(const std::string &s) {
  std::string u = s;
  for (auto &c : u) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return u;
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
// the drive mount.  Falls back to dos_to_host() + s_default_text_mode.
struct Resolved {
  std::string host_path;
  bool        text_mode;
};

// Mangle a host filename into DOS 8.3 form.  Long basenames become
// FIRST6~1, long extensions are truncated.  Case-sensitive collisions
// aren't tracked -- a directory with "LongFoo1" and "LongFoo2" will show
// both as "LONGFO~1" to DOS.  Acceptable for initial findfirst support;
// collision-aware numbering comes later if real tools need it.
std::string mangle_8_3(const std::string &name) {
  std::string base, ext;
  const size_t dot = name.find_last_of('.');
  if (dot == std::string::npos) { base = name; ext = ""; }
  else { base = name.substr(0, dot); ext = name.substr(dot + 1); }

  auto up = [](std::string &s) {
    for (auto &c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  };
  up(base);
  up(ext);

  if (base.size() > 8) base = base.substr(0, 6) + "~1";
  if (ext.size()  > 3) ext  = ext.substr(0, 3);
  if (ext.empty())  return base;
  return base + "." + ext;
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

// Scan the current FindState's DIR until a matching entry is found.
// Fills the DTA and returns true on success.  On exhaustion, closes the
// DIR and returns false.
bool scan_next(FindState &st) {
  while (struct dirent *ent = ::readdir(st.dir)) {
    const std::string name = ent->d_name;
    if (name == "." || name == "..") continue;
    const std::string mangled = mangle_8_3(name);
    if (!glob_match(mangled, st.pattern_u)) continue;
    dta_write_entry(st.host_dir + "/" + name, mangled);
    return true;
  }
  close_find_state(st);
  return false;
}

Resolved resolve_path(const std::string &dos_path) {
  const std::string base_u = upper(basename_dos(dos_path));

  auto mode_to_text = [](const FileMapping &m, bool default_text) {
    if (m.mode == FileMode::Text)   return true;
    if (m.mode == FileMode::Binary) return false;
    return default_text;
  };

  // Pass 1: exact (non-wildcard) match.
  for (const auto &m : s_file_mappings) {
    if (!has_wildcard(m.pattern) && upper(m.pattern) == base_u) {
      return {m.host_path, mode_to_text(m, s_default_text_mode)};
    }
  }
  // Pass 2: wildcard -- first match wins, mode-only override.
  bool text_mode = s_default_text_mode;
  for (const auto &m : s_file_mappings) {
    if (has_wildcard(m.pattern) && glob_match(base_u, upper(m.pattern))) {
      text_mode = mode_to_text(m, s_default_text_mode);
      break;
    }
  }
  return {dos_to_host(dos_path), text_mode};
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
  // Build the FLAGS word dosbox will pop on IRET by flipping bit 0 on the
  // stacked copy at SS:SP+4 (IP:CS:FLAGS).  Uses the dosbox helper.
  CALLBACK_SCF(val);
}

void return_error(uint16_t dos_err) {
  reg_ax = dos_err;
  set_cf(true);
}

Bitu dosemu_int21() {
  switch (reg_ah) {

    case 0x02: {  // Write character in DL to stdout
      std::fputc(reg_dl, stdout);
      std::fflush(stdout);
      return CBRET_NONE;
    }

    case 0x09: {  // Write $-terminated string at DS:DX to stdout
      const PhysPt base = SegValue(ds) * 16;
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
        text_mode = s_default_text_mode;
        pending = &stdin_pending;
      } else {
        auto it = s_handles.find(reg_bx);
        if (it == s_handles.end()) { return_error(0x06); break; }
        fd = it->second.fd;
        text_mode = it->second.text_mode;
        pending = &it->second.read_pending;
      }
      const PhysPt dst = SegValue(ds) * 16 + reg_dx;
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
      const PhysPt src = SegValue(ds) * 16 + reg_dx;
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
      s_dta_linear = SegValue(ds) * 16u + reg_dx;
      return CBRET_NONE;
    }

    case 0x4E: {  // Find first: CX=attr mask, DS:DX=ASCIIZ path/pattern
      const std::string dos_path = read_dos_string(SegValue(ds), reg_dx);
      auto [dir_part, pat_part]  = split_dir_pattern(dos_path);

      // Evict any prior state on this DTA so we don't leak DIR*.
      auto it = s_finds.find(s_dta_linear);
      if (it != s_finds.end()) { close_find_state(it->second); s_finds.erase(it); }

      const std::string host_dir = dir_part.empty()
          ? dos_to_host("")
          : dos_to_host(dir_part);
      DIR *dir = ::opendir(host_dir.c_str());
      if (!dir) { return_error(0x03); break; }      // path not found

      FindState st;
      st.dir       = dir;
      st.host_dir  = host_dir;
      st.pattern_u = upper(pat_part);
      if (!scan_next(st)) { return_error(0x12); break; }  // no more files
      s_finds[s_dta_linear] = st;
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
      const PhysPt dst = SegValue(ds) * 16 + reg_si;
      for (size_t i = 0; i < cwd.size() && i < 63; ++i)
        mem_writeb(dst + i, static_cast<uint8_t>(cwd[i]));
      mem_writeb(dst + std::min(cwd.size(), size_t{63}), 0);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x48: {  // Allocate BX paragraphs -> AX = segment, or CF=1 w/ BX = max
      if (reg_bx == 0xFFFF || reg_bx > (MEM_CEILING - s_mem_highwater)) {
        reg_bx = (s_mem_highwater > MEM_CEILING) ? 0
                 : (MEM_CEILING - s_mem_highwater);
        return_error(0x08);  // insufficient memory
        break;
      }
      reg_ax = s_mem_highwater;
      s_mem_highwater = static_cast<uint16_t>(s_mem_highwater + reg_bx);
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x49: {  // Free memory block at ES.  Bump allocator never frees.
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x4A: {  // Resize memory block at ES to BX paragraphs.  Stub: succeed
                  // if shrinking or if the block is the last one we handed out.
      set_cf(false);
      return CBRET_NONE;
    }

    case 0x4C: {  // Exit with code AL
      s_exit_code = reg_al;
      shutdown_requested = true;
      return CBRET_STOP;
    }

    default:
      std::fprintf(stderr, "dosemu: unimplemented INT 21h AH=%02Xh "
                           "(AL=%02Xh BX=%04Xh CX=%04Xh DX=%04Xh)\n",
                   reg_ah, reg_al, reg_bx, reg_cx, reg_dx);
      s_exit_code = 1;
      shutdown_requested = true;
      return CBRET_STOP;
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

// Build a minimal PSP at PSP_SEG:0000.  Only the command tail at offset 80h
// is populated meaningfully; everything else is zeroed.  Real programs
// querying the PSP will find a legal-looking but mostly empty structure.
void build_psp(const std::vector<std::string> &args) {
  const PhysPt psp = PSP_SEG * 16;
  for (int i = 0; i < 256; ++i) mem_writeb(psp + i, 0);

  // INT 20h at offset 0x00 (CD 20) — programs sometimes jump here.
  mem_writeb(psp + 0x00, 0xCD);
  mem_writeb(psp + 0x01, 0x20);

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

  build_psp(s_args);

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
  s_mem_highwater = 0x2000;
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

  // Default mode: if the user asked for Text (or Auto with eol_convert on),
  // newly-opened handles translate CR LF <-> LF host-side.
  s_default_text_mode =
      (cfg.default_mode == FileMode::Text) ||
      (cfg.default_mode == FileMode::Auto && cfg.default_eol_convert);
  // Auto-default with eol_convert=true is historical cpmemu behaviour but
  // can surprise users running binary tools.  Enable only when the user
  // asked for text explicitly; Auto falls back to binary.
  if (cfg.default_mode == FileMode::Auto) s_default_text_mode = false;

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

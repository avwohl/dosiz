//
// config.cc — .cfg parser and env var expander for dosiz.
//
// The parser is a straight port of cpmemu's load_config_file() with a few
// DOS-specific keys added (machine, cputype, memsize, drive_C, ...).
//

#include "config.h"
#include "debug_settings.hpp"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <sys/stat.h>

namespace dosiz {

namespace {

std::string trim(std::string s) {
  size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

FileMode parse_mode(const std::string &v) {
  if (v == "text")   return FileMode::Text;
  if (v == "binary") return FileMode::Binary;
  return FileMode::Auto;
}

bool parse_bool(const std::string &v) {
  return v == "true" || v == "1" || v == "yes" || v == "on";
}

} // namespace

std::string expand_env_vars(const std::string &s) {
  std::string out;
  out.reserve(s.size());
  for (size_t i = 0; i < s.size(); ) {
    if (s[i] != '$') { out += s[i++]; continue; }
    ++i;  // past '$'
    std::string name;
    if (i < s.size() && s[i] == '{') {
      ++i;
      while (i < s.size() && s[i] != '}') name += s[i++];
      if (i < s.size()) ++i;  // past '}'
    } else {
      while (i < s.size() && (std::isalnum(static_cast<unsigned char>(s[i])) || s[i] == '_'))
        name += s[i++];
    }
    if (const char *v = std::getenv(name.c_str())) out += v;
  }
  return out;
}

bool load_config_file(const std::string &path, Config &cfg) {
  std::ifstream in(path);
  if (!in.is_open()) {
    std::fprintf(stderr, "dosiz: cannot open config file: %s\n", path.c_str());
    return false;
  }

  std::string line;
  int lineno = 0;
  while (std::getline(in, line)) {
    ++lineno;

    if (size_t h = line.find('#'); h != std::string::npos)
      line = line.substr(0, h);
    line = trim(line);
    if (line.empty()) continue;

    size_t eq = line.find('=');
    if (eq == std::string::npos) {
      std::fprintf(stderr, "dosiz: %s:%d: missing '='\n", path.c_str(), lineno);
      continue;
    }

    std::string key = trim(line.substr(0, eq));
    std::string val = expand_env_vars(trim(line.substr(eq + 1)));

    if      (key == "program")       cfg.program = val;
    else if (key == "cd" || key == "chdir") cfg.chdir_path = val;
    else if (key == "default_mode")  cfg.default_mode = parse_mode(val);
    else if (key == "eol_convert")   cfg.default_eol_convert = parse_bool(val);
    else if (key == "verbose")       cfg.verbose = std::atoi(val.c_str());
    else if (key == "printer")       cfg.printer_path = val;
    else if (key == "aux_input")     cfg.aux_in_path = val;
    else if (key == "aux_output")    cfg.aux_out_path = val;
    else if (key == "machine")       cfg.machine = val;
    else if (key == "cputype")       cfg.cputype = val;
    else if (key == "core")          cfg.cpu_core = val;
    else if (key == "memsize")       cfg.memsize_mb = std::atoi(val.c_str());
    else if (key == "headless")      cfg.headless = parse_bool(val);
    else if (key.rfind("drive_", 0) == 0 && key.size() == 7) {
      DriveMount dm;
      dm.letter    = static_cast<char>(std::toupper(static_cast<unsigned char>(key[6])));
      dm.host_path = val;
      cfg.drives.push_back(dm);
    } else if (key == "args") {
      // Split on whitespace into argv-like list. Quoting not yet supported.
      std::string tok;
      for (char c : val) {
        if (std::isspace(static_cast<unsigned char>(c))) {
          if (!tok.empty()) { cfg.args.push_back(tok); tok.clear(); }
        } else {
          tok += c;
        }
      }
      if (!tok.empty()) cfg.args.push_back(tok);
    } else {
      // Treat as file mapping.  Accepted value forms:
      //   PATH              -> host path, default mode
      //   PATH text|binary  -> host path + explicit mode
      //   text|binary|auto  -> mode-only override (no path; used with
      //                        wildcards like "*.TXT = text")
      FileMapping m;
      m.pattern     = key;
      m.mode        = cfg.default_mode;
      m.eol_convert = cfg.default_eol_convert;

      if (val == "text" || val == "binary" || val == "auto") {
        m.mode = parse_mode(val);
        if (m.mode == FileMode::Binary) m.eol_convert = false;
        // host_path stays empty -> wildcard mode-only override.
      } else {
        size_t sp = val.find_last_of(' ');
        if (sp != std::string::npos) {
          std::string tail = val.substr(sp + 1);
          if (tail == "text" || tail == "binary" || tail == "auto") {
            m.mode = parse_mode(tail);
            if (m.mode == FileMode::Binary) m.eol_convert = false;
            val = trim(val.substr(0, sp));
          }
        }
        m.host_path = val;
      }
      cfg.file_mappings.push_back(m);
    }
  }
  return true;
}

namespace {

std::string upper(const std::string &s) {
  std::string u = s;
  for (auto &c : u) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return u;
}

bool path_is_file(const std::string &p) {
  struct stat st;
  return stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

bool has_path_sep(const std::string &s) {
  return s.find_first_of("/\\") != std::string::npos;
}

bool has_com_or_exe(const std::string &s) {
  if (s.size() < 4) return false;
  std::string tail = upper(s.substr(s.size() - 4));
  return tail == ".COM" || tail == ".EXE";
}

// Case-insensitive scan of `dir` for a file named NAME.{COM,EXE}.  Prefers
// .COM over .EXE (DOS convention).  Returns empty on miss.
std::string find_in_dir(const std::string &dir, const std::string &name) {
  const std::string want_u = upper(name);
  DIR *d = opendir(dir.empty() ? "." : dir.c_str());
  if (!d) return "";
  std::string best_com, best_exe;
  while (struct dirent *e = readdir(d)) {
    const std::string entry = e->d_name;
    if (entry.size() != want_u.size() + 4) continue;
    const std::string entry_u = upper(entry);
    if (entry_u.compare(0, want_u.size(), want_u) != 0) continue;
    const std::string ext = entry_u.substr(want_u.size());
    if      (ext == ".COM" && best_com.empty()) best_com = entry;
    else if (ext == ".EXE" && best_exe.empty()) best_exe = entry;
  }
  closedir(d);
  const std::string hit = !best_com.empty() ? best_com : best_exe;
  if (hit.empty()) return "";
  return dir.empty() ? hit : (dir + "/" + hit);
}

// Same but for an exact filename (name already carries the extension).
std::string find_exact_in_dir(const std::string &dir, const std::string &name) {
  const std::string want_u = upper(name);
  DIR *d = opendir(dir.empty() ? "." : dir.c_str());
  if (!d) return "";
  std::string hit;
  while (struct dirent *e = readdir(d)) {
    if (upper(e->d_name) == want_u) { hit = e->d_name; break; }
  }
  closedir(d);
  if (hit.empty()) return "";
  return dir.empty() ? hit : (dir + "/" + hit);
}

} // namespace

std::string resolve_program_path(const std::string &name) {
  // If the caller already supplied a directory component, don't search --
  // just verify the file exists.
  if (has_path_sep(name)) {
    return path_is_file(name) ? name : std::string{};
  }

  std::vector<std::string> dirs = {""};      // empty = CWD
  if (const char *env = dosiz::g_debug.path) {
    std::string p = env;
    size_t start = 0;
    while (start <= p.size()) {
      size_t sep = p.find(':', start);
      if (sep == std::string::npos) sep = p.size();
      if (sep > start) dirs.push_back(p.substr(start, sep - start));
      if (sep == p.size()) break;
      start = sep + 1;
    }
  }

  for (const auto &dir : dirs) {
    std::string hit = has_com_or_exe(name) ? find_exact_in_dir(dir, name)
                                           : find_in_dir(dir, name);
    if (!hit.empty()) return hit;
  }
  return "";
}

std::string sidecar_cfg(const std::string &program_path) {
  const size_t dot = program_path.find_last_of('.');
  if (dot == std::string::npos) return "";
  const std::string candidate = program_path.substr(0, dot) + ".cfg";
  if (path_is_file(candidate)) return candidate;
  // Also accept .CFG for DOS-ish naming.
  const std::string upper_candidate = program_path.substr(0, dot) + ".CFG";
  if (path_is_file(upper_candidate)) return upper_candidate;
  return "";
}

} // namespace dosiz

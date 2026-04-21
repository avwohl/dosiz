//
// config.cc — .cfg parser and env var expander for dosemu.
//
// The parser is a straight port of cpmemu's load_config_file() with a few
// DOS-specific keys added (machine, cputype, memsize, drive_C, ...).
//

#include "config.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

namespace dosemu {

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
    std::fprintf(stderr, "dosemu: cannot open config file: %s\n", path.c_str());
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
      std::fprintf(stderr, "dosemu: %s:%d: missing '='\n", path.c_str(), lineno);
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

} // namespace dosemu

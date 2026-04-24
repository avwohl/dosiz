//
// config.h — dosiz runtime configuration, loaded from CLI and/or a .cfg file.
//
// The .cfg format mirrors cpmemu/examples/example.cfg: a flat "key = value"
// text file with #-comments, environment variable expansion ($VAR or ${VAR}),
// and a small set of recognised keys.
//

#ifndef DOSIZ_CONFIG_H
#define DOSIZ_CONFIG_H

#include <map>
#include <string>
#include <vector>

namespace dosiz {

enum class FileMode { Auto, Text, Binary };

struct FileMapping {
  std::string pattern;   // e.g. "*.BAS" or "MYFILE.TXT"
  std::string host_path; // Linux path (may contain $VAR references, pre-expanded)
  FileMode    mode = FileMode::Auto;
  bool        eol_convert = true;
};

struct DriveMount {
  char        letter = 'C';      // DOS drive letter
  std::string host_path;         // Linux directory to mount
  FileMode    mode = FileMode::Auto;
};

struct Config {
  // Program to run and its arguments.
  std::string program;                  // e.g. "HELLO.EXE" or "C:\\TC\\TCC.EXE"
  std::vector<std::string> args;

  // Change to this Linux directory before launching dosbox. The same directory
  // becomes the default mount for C: unless drives[] overrides.
  std::string chdir_path;

  // Drive → host path mappings. If empty, CWD is mounted as C: by default.
  std::vector<DriveMount> drives;

  // File mode / EOL defaults. Per-file overrides live in file_mappings.
  FileMode default_mode = FileMode::Auto;
  bool     default_eol_convert = true;
  std::vector<FileMapping> file_mappings;

  // Device redirection.
  std::string printer_path;     // File for PRN/LPT1
  std::string aux_in_path;      // File read as AUX input
  std::string aux_out_path;     // File for AUX output

  // Machine defaults. Passed through to dosbox.conf [dosbox]/[cpu].
  std::string machine = "svga_s3";          // dosbox machine= value
  std::string cputype = "auto";             // dosbox cputype=
  std::string cpu_core = "auto";            // dosbox core=
  int         memsize_mb = 16;              // dosbox memsize=

  // dosiz runtime behaviour.
  bool headless = true;           // Suppress SDL window when true
  int  verbose  = 0;              // 0=quiet, 1=normal, 2=trace
};

// Load a .cfg file into cfg. Returns true on success. Errors are printed to
// stderr. Unknown keys beginning with an asterisk or ? are treated as file
// mappings (per cpmemu convention).
bool load_config_file(const std::string &path, Config &cfg);

// Expand $VAR and ${VAR} references in s against the process environment.
std::string expand_env_vars(const std::string &s);

// Resolve a bare program name to an absolute host path.  Search order:
//   1. CWD
//   2. directories from DOSIZ_PATH (colon-separated)
// Within each directory, tries the name as given if it already has a
// .com/.exe extension, otherwise tries NAME.COM then NAME.EXE
// (case-insensitively, per DOS convention).  Returns empty on miss.
std::string resolve_program_path(const std::string &name);

// Given a resolved program path, return the matching sidecar .cfg path if
// it exists (same directory, same basename, .cfg extension).  Empty
// string if no sidecar is present.
std::string sidecar_cfg(const std::string &program_path);

} // namespace dosiz

#endif // DOSIZ_CONFIG_H

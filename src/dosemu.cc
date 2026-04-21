//
// dosemu.cc — command-line entry point.
//
// dosemu links dosbox-staging's CPU + PC-hardware emulator in-process and
// traps DOS INT 21h to C++ implementations running on the host, the way
// cpmemu handles CP/M BDOS calls. This file parses the CLI and .cfg into a
// dosemu::Config, then hands control to the (not-yet-wired) emulator bridge.
//

#include "bridge.h"
#include "config.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

static void print_usage(const char *prog) {
  std::fprintf(stderr,
    "Usage: %s [options] <program.exe> [args...]\n"
    "       %s <config.cfg>\n"
    "\n"
    "Options:\n"
    "  --help, -h         Show this message\n"
    "  --version          Print version and exit\n"
    "  --window           Open an SDL window (default: headless)\n"
    "  --machine=VAL      PC machine type (default: svga_s3)\n"
    "  --cpu=VAL          CPU type (default: auto)\n"
    "  --memsize=N        DOS memory in MB (default: 16)\n"
    "  --verbose, -v      Trace DOS syscalls\n",
    prog, prog);
}

static bool ends_with(const std::string &s, const std::string &suffix) {
  return s.size() >= suffix.size() &&
         std::equal(suffix.rbegin(), suffix.rend(), s.rbegin(),
                    [](char a, char b) {
                      return std::tolower(static_cast<unsigned char>(a)) ==
                             std::tolower(static_cast<unsigned char>(b));
                    });
}

int main(int argc, char **argv) {
  dosemu::Config cfg;

  int i = 1;
  for (; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--help" || a == "-h") { print_usage(argv[0]); return 0; }
    if (a == "--version") {
      std::printf("dosemu 0.1.0-dev (linked against dosbox-staging %s)\n",
                  dosemu::bridge::dosbox_version());
      return 0;
    }
    if (a == "--window")            { cfg.headless = false; continue; }
    if (a == "--verbose" || a == "-v") { cfg.verbose = 1; continue; }
    if (a.rfind("--machine=", 0) == 0) { cfg.machine    = a.substr(10); continue; }
    if (a.rfind("--cpu=",     0) == 0) { cfg.cputype    = a.substr(6);  continue; }
    if (a.rfind("--memsize=", 0) == 0) { cfg.memsize_mb = std::atoi(a.c_str()+10); continue; }
    if (a == "--") { ++i; break; }
    if (a.size() > 1 && a[0] == '-') {
      std::fprintf(stderr, "dosemu: unknown option: %s\n", a.c_str());
      return 1;
    }
    break;
  }

  if (i >= argc) {
    print_usage(argv[0]);
    return 1;
  }

  std::string first = argv[i];
  if (ends_with(first, ".cfg") || ends_with(first, ".conf")) {
    if (!dosemu::load_config_file(first, cfg)) return 1;
    for (++i; i < argc; ++i) cfg.args.emplace_back(argv[i]);
  } else {
    cfg.program = first;
    for (++i; i < argc; ++i) cfg.args.emplace_back(argv[i]);
  }

  if (cfg.program.empty()) {
    std::fprintf(stderr, "dosemu: no program to run (set 'program =' in cfg or pass on CLI)\n");
    return 1;
  }

  std::vector<const char *> argv_c;
  argv_c.reserve(cfg.args.size());
  for (const auto &a : cfg.args) argv_c.push_back(a.c_str());
  int rc = dosemu::bridge::run_program(cfg.program.c_str(),
                                       argv_c.data(), argv_c.size(),
                                       cfg.headless, cfg.verbose);
  if (rc < 0) return 1;
  return rc;
}

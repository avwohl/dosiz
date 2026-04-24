//
// dosiz.cc — command-line entry point.
//
// dosiz links dosbox-staging's CPU + PC-hardware emulator in-process and
// traps DOS INT 21h to C++ implementations running on the host, the way
// cpmemu handles CP/M BDOS calls. This file parses the CLI and .cfg into a
// dosiz::Config, then hands control to the (not-yet-wired) emulator bridge.
//

#include "bridge.h"
#include "config.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>
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
  dosiz::Config cfg;

  int i = 1;
  for (; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--help" || a == "-h") { print_usage(argv[0]); return 0; }
    if (a == "--version") {
      std::printf("dosiz 0.1.0-dev (linked against dosbox-staging %s)\n",
                  dosiz::bridge::dosbox_version());
      return 0;
    }
    if (a == "--window")            { cfg.headless = false; continue; }
    if (a == "--verbose" || a == "-v") { cfg.verbose = 1; continue; }
    if (a.rfind("--machine=", 0) == 0) { cfg.machine    = a.substr(10); continue; }
    if (a.rfind("--cpu=",     0) == 0) { cfg.cputype    = a.substr(6);  continue; }
    if (a.rfind("--memsize=", 0) == 0) { cfg.memsize_mb = std::atoi(a.c_str()+10); continue; }
    if (a == "--") { ++i; break; }
    if (a.size() > 1 && a[0] == '-') {
      std::fprintf(stderr, "dosiz: unknown option: %s\n", a.c_str());
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
    // Explicit config file.
    if (!dosiz::load_config_file(first, cfg)) return 1;
    for (++i; i < argc; ++i) cfg.args.emplace_back(argv[i]);
  } else {
    // Program name (bare, with extension, or with path).  Resolve on
    // DOSIZ_PATH and auto-load a sidecar .cfg if present.
    std::string resolved = dosiz::resolve_program_path(first);
    if (resolved.empty()) {
      std::fprintf(stderr, "dosiz: cannot find '%s' on DOSIZ_PATH\n",
                   first.c_str());
      return 1;
    }
    const std::string sidecar = dosiz::sidecar_cfg(resolved);
    if (!sidecar.empty()) {
      if (!dosiz::load_config_file(sidecar, cfg)) return 1;
      if (cfg.verbose > 0)
        std::fprintf(stderr, "dosiz: loaded sidecar %s\n", sidecar.c_str());
    }
    // The resolved path always wins over any `program =` inside the sidecar.
    cfg.program = resolved;
    for (++i; i < argc; ++i) cfg.args.emplace_back(argv[i]);
  }

  if (cfg.program.empty()) {
    std::fprintf(stderr, "dosiz: no program to run (set 'program =' in cfg or pass on CLI)\n");
    return 1;
  }

  int rc = dosiz::bridge::run_program(cfg);
  if (rc < 0) return 1;
  // dosbox's module singletons have no guaranteed destruction order -- the
  // PIC destructor touches already-freed IO maps on exit, and the heap
  // reports "corrupted size vs. prev_size" during teardown.  _exit skips
  // C++ static destruction entirely; the OS reclaims process resources
  // just fine, and stdout/stderr have already been flushed.
  std::fflush(stdout);
  std::fflush(stderr);
  _exit(rc);
}

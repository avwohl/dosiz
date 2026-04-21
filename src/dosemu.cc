//
// dosemu.cc — command-line entry point for dosemu.
//
// Phase 1: writes a dosbox.conf that mounts the current directory (or the
// paths configured in a .cfg file) as DOS drives, exec's dosbox-staging with
// that conf, and forwards the exit code.
//
// Usage:
//   dosemu [options] PROGRAM.EXE [args...]
//   dosemu config.cfg
//

#include "config.h"
#include "dosbox_conf.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <unistd.h>
#include <sys/wait.h>

static void print_usage(const char *prog) {
  std::fprintf(stderr,
    "Usage: %s [options] <program.exe> [args...]\n"
    "       %s <config.cfg>\n"
    "\n"
    "Options:\n"
    "  --help, -h         Show this message\n"
    "  --version          Print version and exit\n"
    "  --window           Open a dosbox SDL window (default: headless)\n"
    "  --machine=VAL      dosbox machine= value (default: svga_s3)\n"
    "  --cpu=VAL          dosbox cputype= value (default: auto)\n"
    "  --memsize=N        DOS memory in MB (default: 16)\n"
    "  --keep-conf        Do not delete the generated dosbox.conf on exit\n"
    "  --verbose, -v      Print the dosbox command line being run\n"
    "  --dosbox=PATH      Path to the dosbox-staging binary\n"
    "\n"
    "Environment variables:\n"
    "  DOSEMU_DOSBOX      Override path to dosbox-staging binary\n",
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
  bool keep_conf = false;

  int i = 1;
  for (; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--help" || a == "-h") { print_usage(argv[0]); return 0; }
    if (a == "--version")           { std::printf("dosemu 0.1.0-dev\n"); return 0; }
    if (a == "--window")            { cfg.headless = false; continue; }
    if (a == "--keep-conf")         { keep_conf = true; continue; }
    if (a == "--verbose" || a == "-v") { cfg.verbose = 1; continue; }
    if (a.rfind("--machine=", 0) == 0) { cfg.machine  = a.substr(10); continue; }
    if (a.rfind("--cpu=",     0) == 0) { cfg.cputype  = a.substr(6);  continue; }
    if (a.rfind("--memsize=", 0) == 0) { cfg.memsize_mb = std::atoi(a.c_str()+10); continue; }
    if (a.rfind("--dosbox=",  0) == 0) { cfg.dosbox_binary = a.substr(9); continue; }
    if (a == "--") { ++i; break; }
    if (a.size() > 1 && a[0] == '-') {
      std::fprintf(stderr, "dosemu: unknown option: %s\n", a.c_str());
      return 1;
    }
    break;  // Non-option: either .cfg or program name
  }

  if (i >= argc) {
    print_usage(argv[0]);
    return 1;
  }

  std::string first = argv[i];
  if (ends_with(first, ".cfg") || ends_with(first, ".conf")) {
    if (!dosemu::load_config_file(first, cfg)) return 1;
    ++i;
    // Any leftover args become program args (unless program is in cfg).
    for (; i < argc; ++i) cfg.args.emplace_back(argv[i]);
  } else {
    cfg.program = first;
    for (++i; i < argc; ++i) cfg.args.emplace_back(argv[i]);
  }

  if (cfg.program.empty()) {
    std::fprintf(stderr, "dosemu: no program to run (set 'program =' in cfg or pass on CLI)\n");
    return 1;
  }

  std::string dosbox = dosemu::locate_dosbox_binary(cfg, argv[0]);
  if (dosbox.empty()) {
    std::fprintf(stderr,
      "dosemu: cannot find dosbox-staging. Build it via:\n"
      "    ninja -C dosbox-staging/build\n"
      "  or set DOSEMU_DOSBOX=/path/to/dosbox\n");
    return 1;
  }

  std::string conf = dosemu::write_dosbox_conf(cfg);
  if (conf.empty()) return 1;

  if (cfg.verbose > 0)
    std::fprintf(stderr, "dosemu: dosbox=%s conf=%s\n", dosbox.c_str(), conf.c_str());

  // Build argv for execvp. We pass -exit and our generated conf explicitly.
  std::vector<const char *> cargv;
  cargv.push_back(dosbox.c_str());
  cargv.push_back("-conf");
  cargv.push_back(conf.c_str());
  cargv.push_back("-exit");
  cargv.push_back(nullptr);

  pid_t pid = fork();
  if (pid < 0) {
    std::perror("dosemu: fork");
    return 1;
  }
  if (pid == 0) {
    if (cfg.headless) setenv("SDL_VIDEODRIVER", "dummy", 1);
    // Ensure audio never initializes — avoids ALSA/PulseAudio timing issues
    // in ephemeral headless runs.
    setenv("SDL_AUDIODRIVER", "dummy", 1);
    execvp(dosbox.c_str(), const_cast<char *const *>(cargv.data()));
    std::perror("dosemu: execvp");
    _exit(127);
  }

  int status = 0;
  if (waitpid(pid, &status, 0) < 0) {
    std::perror("dosemu: waitpid");
    if (!keep_conf) unlink(conf.c_str());
    return 1;
  }

  if (!keep_conf) unlink(conf.c_str());

  if (WIFEXITED(status))   return WEXITSTATUS(status);
  if (WIFSIGNALED(status)) return 128 + WTERMSIG(status);
  return 1;
}

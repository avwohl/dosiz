//
// dosemu.cc — command-line entry point.
//
// Phase 0 stub.  Phase 1 will wire this up to dosbox_bridge so that running
//   dosemu PROG.EXE
// boots dosbox with CWD mounted as C:, executes PROG.EXE, and exits when the
// program returns to COMMAND.COM.
//

#include <cstdio>
#include <cstdlib>
#include <cstring>

static void print_usage(const char *prog) {
  std::fprintf(stderr,
    "Usage: %s [options] <program.exe> [args...]\n"
    "       %s <config.cfg>\n"
    "\n"
    "Options:\n"
    "  --help           Show this message\n"
    "  --version        Print version and exit\n"
    "\n"
    "Phase 0 stub.  Real execution lands in Phase 1.\n",
    prog, prog);
}

int main(int argc, char **argv) {
  if (argc < 2) {
    print_usage(argv[0]);
    return 1;
  }
  if (std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0) {
    print_usage(argv[0]);
    return 0;
  }
  if (std::strcmp(argv[1], "--version") == 0) {
    std::printf("dosemu 0.0.0-dev\n");
    return 0;
  }

  std::fprintf(stderr,
    "dosemu: Phase 0 stub — dosbox backend not yet wired.\n"
    "  Would run: %s\n", argv[1]);
  return 2;
}

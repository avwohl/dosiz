//
// bridge.h — thin C++ wrapper around the in-process dosbox-staging library.
//
// Isolates every #include of dosbox internals to bridge.cc so dosiz.cc
// stays readable.
//

#ifndef DOSIZ_BRIDGE_H
#define DOSIZ_BRIDGE_H

#include "config.h"

namespace dosiz::bridge {

const char *dosbox_version();

// Full in-process bring-up of dosbox-staging: registers sections, parses
// config, initialises SDL, activates modules, overrides SHELL_Init with our
// own startup hook, installs an INT 21h handler that dispatches to
// host-side C++, builds a minimal PSP with the command tail, loads the
// program (.COM or .EXE by extension), and runs the CPU until it exits via
// INT 21h AH=4Ch.  Returns the DOS exit code (AL) on success, or -1 on
// startup failure.
int run_program(const Config &cfg);

} // namespace dosiz::bridge

#endif

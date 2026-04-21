//
// bridge.h — thin C++ wrapper around the in-process dosbox-staging library.
//
// Isolates every #include of dosbox internals to bridge.cc so dosemu.cc
// stays readable.  As strict DOS replacement lands, the INT 21h callback
// registration will live here.
//

#ifndef DOSEMU_BRIDGE_H
#define DOSEMU_BRIDGE_H

namespace dosemu::bridge {

const char *dosbox_version();

// Full in-process bring-up of dosbox-staging: registers sections, parses
// config, initialises SDL, activates modules, and runs our startup hook
// (instead of SHELL_Init).  verbose: 0=errors only, 1=warnings, 2=info.
// Returns true on success.
bool bring_up_emulator(bool headless, int verbose);

} // namespace dosemu::bridge

#endif

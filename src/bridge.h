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

// Minimal in-process dosbox bring-up.  Instantiates CommandLine + Config,
// calls DOSBOX_Init() to register the section hierarchy, overrides the
// default startup function (SHELL_Init) with our own, and invokes
// Config::StartUp() to demonstrate the seam fires.  Does NOT yet activate
// modules (no CPU, memory, or device init).  Returns true on success.
bool register_sections_and_run_startup();

} // namespace dosemu::bridge

#endif

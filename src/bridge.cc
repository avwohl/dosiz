//
// bridge.cc — the single translation unit in dosemu that touches dosbox
// internals.
//
// Strict cpmemu-style DOS replacement is built up here incrementally:
//   1. link against dosbox-staging libs (done)
//   2. register section hierarchy via DOSBOX_Init()                 (done)
//   3. install our start function via control->SetStartUp()         <-- now
//   4. run the CPU/memory/device init (control->Init())             (next)
//   5. install INT 21h callback, load EXE, call DOSBOX_RunMachine   (next)
//
// The SetStartUp seam is the key to strict replacement: DOSBOX_Init() wires
// SHELL_Init as the default startup via `control->SetStartUp(&SHELL_Init)`
// (dosbox.cpp line 1349).  We override it before calling Config::StartUp(),
// so dosbox's shell -- and its DOS kernel by extension -- never executes.
// No patch to dosbox-staging needed.
//
// All dosbox headers stay confined to this file.
//

#include "bridge.h"

#define SDL_MAIN_HANDLED
#include <SDL.h>

#include "dosbox.h"
#include "control.h"
#include "programs.h"

#include <cstdio>
#include <memory>

namespace dosemu::bridge {

const char *dosbox_version() {
  return DOSBOX_GetVersion();
}

namespace {

void dosemu_startup() {
  // This runs in place of SHELL_Init.  At this point Config::Init() has
  // activated all sections (including [dos], which we will ignore), and the
  // CPU is ready to run.  Next step: install INT 21h callback, load the
  // user's program, call DOSBOX_RunMachine().
  std::fprintf(stderr, "dosemu: startup hook reached; CPU init and EXE "
                       "loader not yet wired up\n");
}

} // namespace

bool register_sections_and_run_startup() {
  static const char *dummy_argv[] = {"dosemu", nullptr};
  auto cmdline = std::make_unique<CommandLine>(1, dummy_argv);
  control      = std::make_unique<Config>(cmdline.get());

  try {
    DOSBOX_Init();
  } catch (const std::exception &e) {
    std::fprintf(stderr, "dosemu: DOSBOX_Init threw: %s\n", e.what());
    return false;
  }

  // Swap out SHELL_Init (which DOSBOX_Init wired as the default) with our
  // own startup function.  control->StartUp() now invokes dosemu_startup
  // instead of running dosbox's shell.
  control->SetStartUp(&dosemu_startup);
  control->StartUp();
  return true;
}

} // namespace dosemu::bridge

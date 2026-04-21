//
// bridge.cc — the single translation unit in dosemu that touches dosbox
// internals.
//
// Strict cpmemu-style DOS replacement is built up here incrementally:
//   1. link against dosbox-staging libs                              (done)
//   2. register section hierarchy via DOSBOX_Init()                  (done)
//   3. install our start function via control->SetStartUp()          (done)
//   4. run the CPU/memory/device init (control->Init())              <-- now
//   5. install INT 21h callback
//   6. load .COM/.EXE host-side, call DOSBOX_RunMachine()
//
// The SetStartUp seam is the key to strict replacement: DOSBOX_Init() wires
// SHELL_Init as the default startup via `control->SetStartUp(&SHELL_Init)`
// (dosbox.cpp line 1349).  We override it before calling Config::StartUp(),
// so dosbox's shell -- and its DOS kernel by extension -- never executes.
//
// All dosbox headers stay confined to this file.
//

#include "bridge.h"

#define SDL_MAIN_HANDLED
#include <SDL.h>

#include "dosbox.h"
#include "control.h"
#include "cross.h"
#include "programs.h"
#include "loguru.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>

namespace dosemu::bridge {

const char *dosbox_version() {
  return DOSBOX_GetVersion();
}

namespace {

void dosemu_startup() {
  // Runs in place of SHELL_Init.  At this point Config::Init() has activated
  // all sections (including [dos], which we ignore), and the CPU is ready
  // to run.  Next step: install INT 21h callback, load user's program,
  // DOSBOX_RunMachine().
  std::fprintf(stderr, "dosemu: startup hook reached; CPU init and EXE "
                       "loader not yet wired up\n");

  // Break out of the machine loop immediately instead of idling.
  shutdown_requested = true;
}

} // namespace

bool bring_up_emulator(bool headless, int verbose) {
  // Squelch dosbox's log stream unless the user asked for it.  Default is
  // errors-only; --verbose bumps to warnings; -vv to info.
  loguru::g_stderr_verbosity = (verbose >= 2) ? loguru::Verbosity_INFO
                              : (verbose >= 1) ? loguru::Verbosity_WARNING
                                               : loguru::Verbosity_ERROR;

  static const char *dummy_argv[] = {"dosemu", nullptr};
  auto cmdline = std::make_unique<CommandLine>(1, dummy_argv);
  control      = std::make_unique<Config>(cmdline.get());

  // Headless mode uses SDL's "offscreen" video driver: no window, no
  // display-server dependency, still provides a real SDL renderer so VGA
  // draw code has a target.  Cross-platform.
  if (headless) {
    setenv("SDL_VIDEODRIVER", "offscreen", 1);
    setenv("SDL_AUDIODRIVER", "dummy", 1);
  }

  try {
    // 1. Register section hierarchy (also wires SHELL_Init as default).
    InitConfigDir();
    DOSBOX_Init();

    // 2. Parse any user config file that happens to exist.  Missing files
    //    are fine — defaults apply.
    control->ParseConfigFiles(GetConfigDir());

    // 3. SDL subsystems required by the hardware modules.
    if (SDL_Init(SDL_INIT_AUDIO | SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
      std::fprintf(stderr, "dosemu: SDL_Init failed: %s\n", SDL_GetError());
      return false;
    }

    // 4. Activate all modules (MEM, CPU, FPU, hardware, BIOS, DOS...).
    control->ParseEnv();
    control->Init();

    // 5. Swap SHELL_Init for our startup function.
    control->SetStartUp(&dosemu_startup);

    // 6. Hand control to dosemu_startup -- dosbox's shell never runs.
    control->StartUp();
  } catch (const std::exception &e) {
    std::fprintf(stderr, "dosemu: bring-up threw: %s\n", e.what());
    return false;
  } catch (char *msg) {
    std::fprintf(stderr, "dosemu: bring-up failed: %s\n", msg);
    return false;
  }

  return true;
}

} // namespace dosemu::bridge

# dosiz project conventions

## Workflow

- When you reach a summary/end-of-turn for a unit of work, commit and push
  to origin/main without asking first. Don't wait for explicit "commit" or
  "push" instructions.

## Backend

- dosiz emulates DOS itself — the INT 21h / INT 31h (DPMI) / INT 67h (EMS+VCPI)
  host, PSP, program loader, and DPMI mode-switching all live in `src/bridge.cc`.
  The CPU + memory + PC hardware come from the **emu88** 386 core, which lives
  in the sibling **qxDOS** repo and is read from `../qxDOS/emu88` — it is *not*
  copied into this tree. It is driven through a thin DOSBox→emu88 compatibility
  shim in `src/compat/`.
- There is **no DOSBox, SDL2, or glib dependency**. The build is plain CMake over
  emu88 + dosiz: `make` (a thin wrapper) or
  `cmake -S src -B build && cmake --build build -j`.
- The `Descriptor`/`DescriptorTable` classes and the native-call callback
  mechanism in `src/compat/`, plus the LIM EMS 4.0 / VCPI provider in
  `src/bridge.cc`, are **derived from dosbox-staging (GPLv2-or-later)**. Keep that
  attribution accurate in `docs/CREDITS.md`.
- **emu88 belongs to qxDOS. Do not fix emu88 bugs from this repo** — edit
  `../qxDOS/emu88/` and validate there, because qxDOS owns the 386 test suites
  (`qxDOS/tests/`: SingleStepTests/80386 at 1.76M cases, plus VESA, hardware,
  OPL, Sound Blaster and UART harnesses). Building dosiz requires a qxDOS
  checkout beside this one; `EMU88_DIR` in `src/CMakeLists.txt` overrides the
  location.
- This used to be a vendored copy under `dosiz/emu88/`, and the two silently
  drifted 129 lines before being consolidated on 2026-08-09 — dosiz had four CPU
  fixes (FUCOMPP, the `pending_seg_idx` segment-cache selection, the unmasked
  linear address for the VESA LFB aperture, and `#UD` instead of a fatal halt)
  that were never ported back. That is why there is now a single copy.

## README stays short
README.md is at most 150 lines. New detail goes in docs/<topic>.md with a one-line link from the README; never add sections to the README.

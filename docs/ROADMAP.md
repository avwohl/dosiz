# Roadmap

## Phase 0 — Scaffolding ✅

Directory layout, CMake/Makefile skeleton, GPLv3 LICENSE, README,
dosbox-staging submodule pinned to v0.82.2, initial public GitHub repo.

## Phase 1 — Subprocess-style DOS launcher ✅

`dosemu PROG.EXE` generates a dosbox.conf on the fly, mounts the current
directory (or the paths named in a `.cfg` file) as DOS drives, and spawns
the bundled `dosbox-staging/build/dosbox` with that conf. Exit code of the
child propagates back to the shell.

Includes:
- CLI option parsing (`--help`, `--version`, `--machine=`, `--cpu=`,
  `--memsize=`, `--window`, `--dosbox=`, `--keep-conf`, `--verbose`)
- `.cfg` file parser ported from cpmemu (env var expansion, per-pattern
  mode overrides, multi-drive mounts, device redirection hooks)
- 11 smoke tests covering CLI + conf generation

## Phase 2 — Headless / CLI integration ✅ (basic)

SDL window suppressed by default via `SDL_VIDEODRIVER=dummy`.  Audio
disabled via `SDL_AUDIODRIVER=dummy` + `[mixer] nosound=true`.  Use
`--window` to get a normal SDL window for graphical programs.

Still open:
- [ ] Surface DOS text-mode VRAM to stdout in real time (so a DOS compiler's
      progress output appears in the terminal, not just after exit)
- [ ] Forward stdin to the DOS keyboard buffer
- [ ] ANSI translation mode for the 80x25 framebuffer

## Phase 3 — cpmemu-parity file handling

The key feature over plain dosbox: transparent long-name ↔ 8.3 mapping and
text/binary EOL conversion, driven by the `.cfg` file's `*.EXT = mode`
lines.

Approaches under consideration:
- **FUSE overlay**: mount a CR/LF-translating view of the host directory
  that dosbox sees as C:.  Pure host-side, no dosbox changes.
- **DOS TSR + INT E0**: guest-resident shim that intercepts INT 21h
  and round-trips through a custom host syscall.  cpmemu-esque, fiddly.
- **In-process fork**: link against dosbox libs and intercept
  DOS_Int21Handler directly.  Powerful but requires a patched dosbox.

Pick after Phase 1 ships and real usage informs the trade-off.

## Phase 4 — DPMI / protected mode programs

dosbox-staging already provides a DPMI host, so DJGPP / DOS4GW binaries
should Just Work once Phase 1 lands.  Work items:
- [ ] Smoke-test a DJGPP hello-world through dosemu
- [ ] Smoke-test WATCOM / DOS4GW binary
- [ ] Document memory/cycle defaults for PM programs

## Phase 5 — Video / sound for graphical programs

`--window` already opens an SDL window.  Work items:
- [ ] Verify full-screen VGA modes
- [ ] Verify Sound Blaster detection / playback
- [ ] ANSI text-mode fallback for terminals

## Phase 6 — Packaging

Copy cpmemu's Debian/RPM packaging scripts and adapt:
- [ ] `.deb` build via `dpkg-buildpackage`
- [ ] `.rpm` build
- [ ] GitHub Actions CI

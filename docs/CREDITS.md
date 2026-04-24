# Credits and Third-Party Components

## dosbox-staging

dosiz embeds [dosbox-staging](https://github.com/dosbox-staging/dosbox-staging)
as a git submodule under `dosbox-staging/`. Licensed GPLv2-or-later.

dosbox-staging provides the 386+ CPU emulator, VGA/SVGA, Sound Blaster, and
the low-level DOS kernel that dosiz initializes but overrides.  The DPMI
0.9 host (INT 31h sub-functions, mode-switch primitives, RM callbacks,
PM exception dispatch), the LE loader, and the INT 21h handlers are
dosiz's own, implemented in `src/bridge.cc` on top of dosbox's CPU state
and callback API.

## FreeDOS

When dosiz uses FreeDOS binaries or source, attribution and source
availability obligations from FreeDOS apply. See https://freedos.org.

## Inspiration

- [cpmemu](https://github.com/avwohl/cpmemu) — the translation-layer approach
  and much of the `.cfg` / file-mapping design.
- [qxDOS](https://github.com/avwohl/qxDOS) — the `dosbox_bridge` and
  `int_e0_hostio` pattern originated there.
- [tnylpo](https://github.com/SvenMb/gbrein_tnylpo) — prior art for CP/M
  syscall translation.

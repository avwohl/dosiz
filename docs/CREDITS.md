# Credits and Third-Party Components

## dosbox-staging

dosemu embeds [dosbox-staging](https://github.com/dosbox-staging/dosbox-staging)
as a git submodule under `dosbox-staging/`. Licensed GPLv2-or-later.

dosbox-staging provides the 386+ CPU emulator, VGA/SVGA, Sound Blaster, the
DPMI host, and the low-level DOS kernel that dosemu builds on top of.

## FreeDOS

When dosemu uses FreeDOS binaries or source, attribution and source
availability obligations from FreeDOS apply. See https://freedos.org.

## Inspiration

- [cpmemu](https://github.com/avwohl/cpmemu) — the translation-layer approach
  and much of the `.cfg` / file-mapping design.
- [qxDOS](https://github.com/avwohl/qxDOS) — the `dosbox_bridge` and
  `int_e0_hostio` pattern originated there.
- [tnylpo](https://github.com/SvenMb/gbrein_tnylpo) — prior art for CP/M
  syscall translation.

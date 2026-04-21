# dosemu

An MS-DOS emulator for Linux that runs DOS programs by translating MS-DOS
system calls to Linux, similar to how [cpmemu](https://github.com/avwohl/cpmemu)
translates CP/M calls. The 386+ CPU, hardware, and DPMI host are provided by
[dosbox-staging](https://github.com/dosbox-staging/dosbox-staging); dosemu
adds cpmemu-style file handling (long-name ↔ 8.3 mapping, text/binary EOL
translation, `.cfg` file for options) and a Linux CLI front-end so command-line
DOS tools (compilers, assemblers) run without opening a window.

**Status:** Early development.

## Why

Most DOS emulators use native FAT disk images. When developing with a DOS
compiler this means shuffling files in and out of a disk image for every
build. dosemu makes the DOS program see Linux files directly, so you can:

- Run a DOS C compiler as if it were a Linux CLI tool
- Use long filenames on the host while presenting 8.3 names to DOS
- Auto-translate CR/LF line endings for text files
- Skip the SDL window entirely for text-only programs

## Architecture

```
argv ──► dosemu CLI ──► dosbox_bridge ──► dosbox-staging
                              │                 │
                              │                 └─ CPU (386+) / VGA / DPMI / etc.
                              │
                              ├─ INT 21h prolog hook  (long-name, EOL translation)
                              ├─ INT E0 host file I/O (R.COM / W.COM style guest tools)
                              └─ .cfg parser          (file mappings, modes, printer redirect)
```

## Building

Requires: CMake 3.24+, a C++20 compiler, SDL2-dev, git.

```sh
git clone --recurse-submodules https://github.com/avwohl/dosemu.git
cd dosemu
cmake -S src -B build
cmake --build build -j
```

## Usage

```sh
dosemu [options] <program.exe> [args...]
dosemu config.cfg
```

See `docs/` once populated.

## License

GPLv3. dosbox-staging is GPLv2-or-later (compatible). FreeDOS code, when used,
retains its original license and is credited in `docs/CREDITS.md`.

## Related Projects

- [cpmemu](https://github.com/avwohl/cpmemu) — CP/M 2.2 emulator with the same translation-layer philosophy
- [qxDOS](https://github.com/avwohl/qxDOS) — iOS/Mac DOS emulator, also built on dosbox-staging

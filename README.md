# dosiz

An MS-DOS emulator that runs DOS programs by emulating the DOS API itself —
trapping INT 21h / INT 31h (DPMI) / INT 67h (EMS) and translating them to C++
implementations on the host — on top of the
[emu88](https://github.com/avwohl/qxDOS) 386 CPU core, which lives in the qxDOS
repo and is built from a sibling checkout. Same design as
[cpmemu](https://github.com/avwohl/cpmemu), which does the equivalent for
CP/M BDOS.

**Status:** real DOS programs, DOS-hosted toolchain binaries, cross-compiled
binaries, and 32-bit DJGPP programs run. The DPMI 0.9 host, LIM EMS 4.0, and the
LE loader are described in [docs/architecture.md](docs/architecture.md), together
with the full status and the list of implemented DOS services.

	dosiz tests/EXE2BIN.EXE              → Open Watcom banner   (real DOS-hosted)
	dosiz tests/HELLO_W.EXE              → hello from watcom    (Watcom cross-compiled)
	dosiz tests/HELLO_B.COM              → hello from bcc       (bcc cross-compiled)
	dosiz tests/LE_MIN.EXE               → exit 0                (hand-crafted LE)
	echo F | dosiz xcopy.exe src dst     → copies src to dst    (FreeDOS xcopy)
	dosiz mTCP-FTP.EXE                   → prints usage banner  (Open Watcom 16-bit, real DOS)

	dosiz wd.exe / vi.exe                → enters 32-bit PM, runs ~0x135 bytes,
	                                        GP-faults on DOS4G pre-entry selector
	                                        setup we don't emulate

## Building

dosiz needs a C++20 compiler, CMake, libm, and a checkout of
[qxDOS](https://github.com/avwohl/qxDOS) beside this one — that repo owns the
emu88 386 core dosiz builds against, so there is only ever one copy of it. No
glib, meson, or DOSBox. SDL2 is the one **optional** dependency: when present at
build time it enables the `--window` VGA display; without it the build is
otherwise dependency-free and headless.

	git clone https://github.com/avwohl/qxDOS.git   # next to dosiz/

If qxDOS lives somewhere else, point dosiz at it:

	cmake -S src -B build -DEMU88_DIR=/path/to/qxDOS/emu88

	# Debian/Ubuntu  (libsdl2-dev is optional, for --window)
	sudo apt install build-essential cmake libsdl2-dev

	# macOS  (sdl2 is optional, for --window)
	brew install cmake sdl2

	# build + smoke-test (all platforms)
	make                         # wrapper: cmake -S src -B build && cmake --build build
	build/dosiz --version        # → dosiz 0.1.0-dev (backend: emu88)
	build/dosiz tests/HELLO.COM  # → dosiz-hello-ok

`make clean` removes the build directory.

## Why

Most DOS emulators use native FAT disk images. When developing with a DOS
compiler that means shuffling files in and out of the disk image for every
build. dosiz makes the DOS program see host files directly, so you can:

- Run a DOS C compiler as if it were a native CLI tool
- Use long filenames on the host while presenting 8.3 names to DOS
- Run text-only programs with no window at all (the default)
- Or open a VGA window with `--window` (text + VGA/SVGA graphics)
- Redirect DOS printer / AUX I/O to host files

Because the syscall layer is native C++ and emu88 is self-contained, dosiz is
intended to run on Linux, macOS, Windows, iOS, iPadOS, and Android — the same
platform set cpmemu already covers.

## Usage

	dosiz [options] PROGRAM.EXE [args...]
	dosiz [options] config.cfg
	dosiz PROG                         # bare name -- search DOSIZ_PATH

`dosiz PROG` looks for `PROG.COM` (preferred) or `PROG.EXE` first in the
current directory, then in each `:`-separated entry of `DOSIZ_PATH`.
Matching is case-insensitive. If a sidecar `PROG.cfg` exists next to the
resolved executable, it is auto-loaded as configuration (drive mounts,
text-mode, file mappings) before the program runs:

	export DOSIZ_PATH=~/dos/bin:/usr/local/dos/bin
	dosiz tcc hello.c                  # finds tcc.exe + tcc.cfg (if any)

Options:

	--help              Show usage
	--version           Print version
	--window            Open a VGA window (needs SDL2 at build time)
	--memsize=N         DOS memory in MB (default: 16)
	--verbose, -v       Trace DOS syscalls

`--window` opens a window and renders VGA text, 320x200x256 graphics (mode 13h),
and SVGA/VESA VBE 2.0 modes (8/15/16/24/32-bpp up to 1280x1024) via an INT 10h
video BIOS, with keyboard input; it needs SDL2 at build time (otherwise it
prints a notice and runs headless). `DOSIZ_FRAME_DUMP=out.ppm` renders the final
screen to a PPM without needing a window. `--machine=NAME` and `--cpu=NAME` are
accepted but currently inert. Audio (Sound Blaster/AdLib), the mouse, and the
joystick are not yet wired on the emu88 backend.

## Example .cfg

See `examples/example.cfg` for a documented sample. Minimal:

	program  = PROG.EXE
	args     = /q /v
	drive_C  = ${HOME}/dos
	drive_D  = /mnt/sources
	memsize  = 16

## Documentation

- [docs/architecture.md](docs/architecture.md): status, architecture, the implemented INT 21h / INT 31h / INT 67h services, the LE loader, and file mapping and text mode.
- [docs/c-toolchain-guide.md](docs/c-toolchain-guide.md): lessons from running real DOS C toolchains under dosiz.
- [docs/watcom-setup.md](docs/watcom-setup.md): building Open Watcom DOS/4G programs under dosiz.
- [docs/djgpp-libc-cpp-crash.md](docs/djgpp-libc-cpp-crash.md): why the DJGPP `cpp.exe` crashes under dosiz.
- [docs/emu88-cpu-backend.md](docs/emu88-cpu-backend.md): the emu88 CPU backend.
- [docs/original-brief.md](docs/original-brief.md): the original project brief.
- [docs/CREDITS.md](docs/CREDITS.md): third-party attributions.
- [DEBUGGING.md](DEBUGGING.md): debugging aids and environment variables.
- [CHANGELOG.md](CHANGELOG.md): the change log.

## License

GPLv3. Parts of the CPU compatibility shim (`src/compat/`) and the INT 67h
EMS/VCPI provider are derived from dosbox-staging (GPLv2-or-later, compatible).
Third-party attributions in `docs/CREDITS.md`.

## Related Projects

- [cpmemu](https://github.com/avwohl/cpmemu) — Z80/CP/M emulator that translates the BDOS and BIOS calls of CP/M 2.2 programs to the host file system. It is the template for the translation layer and the origin of the `.cfg` format.
- [qxDOS](https://github.com/avwohl/qxDOS) — DOS emulator app for iOS and macOS. It supplies the emu88 CPU core that dosiz includes.
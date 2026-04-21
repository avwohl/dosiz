# dosemu

An MS-DOS emulator that runs DOS programs by linking the
[dosbox-staging](https://github.com/dosbox-staging/dosbox-staging) CPU and
PC-hardware emulator in-process and trapping DOS INT 21h calls to C++
implementations running on the host. Same design as
[cpmemu](https://github.com/avwohl/cpmemu), which does the equivalent for
CP/M BDOS.

**Status:** basic .COM and .EXE programs run.

	dosemu tests/HELLO.COM           → prints dosemu-hello-ok
	dosemu tests/HELLO.EXE           → prints dosemu-hello-exe-ok
	dosemu tests/SYSCALLS.COM        → dosver=6.22 / alloc=0x2000 / int21-set-ok
	dosemu tests/WRITE.COM ci-tail   → creates WROTE.TXT in CWD

dosbox-staging is linked in-process for CPU + PC hardware. DOS INT 21h is
handled entirely by C++ host code. Currently implemented:

	02  putchar            25  set int vector     3F  read handle
	09  print string       30  get DOS version    40  write handle
	0E  set drive          35  get int vector     42  seek handle
	19  get drive          3B  chdir              44  ioctl (basic)
	3C  create handle      3D  open handle        47  get cwd
	3E  close handle       48  allocate (bump)    49  free (no-op)
	4A  resize (stub)      4C  exit

PSP command tail at offset 80h is populated from argv. Drive mounts and
per-file / per-pattern mappings come from a `.cfg` file:

	program        = PROG.EXE
	args           = /q /v
	drive_C        = /home/me/dos
	drive_D        = /mnt/sources
	default_mode   = text           # all files: CRLF<->LF
	HELLO.TXT      = /real/path/hello_long_name.txt text
	*.BAS          = text           # mode-only wildcard override

Text mode strips CR on write and expands LF to CRLF on read so files
live on the host in Unix format. No subprocess, no dosbox shell, no
generated dosbox.conf.

## Building

One-time setup: build dosbox-staging's static libraries via its own meson.

	sudo apt install build-essential cmake ninja-build meson pkg-config \
	  libsdl2-dev libsdl2-net-dev libpng-dev libopusfile-dev \
	  libspeexdsp-dev libfluidsynth-dev libslirp-dev libasound2-dev \
	  libxi-dev libglib2.0-dev

	meson setup dosbox-staging/build --buildtype=release dosbox-staging
	ninja -C dosbox-staging/build

Then build dosemu itself:

	cmake -S src -B build
	cmake --build build -j$(nproc)

	build/dosemu --version   # should report the linked dosbox-staging version

## Why

Most DOS emulators use native FAT disk images. When developing with a DOS
compiler that means shuffling files in and out of the disk image for every
build. dosemu makes the DOS program see host files directly, so you can:

- Run a DOS C compiler as if it were a native CLI tool
- Use long filenames on the host while presenting 8.3 names to DOS
- Skip the SDL window entirely for text-only programs
- Redirect DOS printer / AUX I/O to host files
- Drive graphical DOS programs with `--window`

Because the syscall layer is native C++, dosemu is intended to run on
Linux, macOS, Windows, iOS, iPadOS, and Android — the same platform set
cpmemu already covers.

## Architecture

	dosemu binary
		dosbox-staging CPU + PC hardware (linked as library)
		host-side DOS: INT 21h handler → C++ file / memory / process calls
		.cfg parser (cpmemu-style)

No subprocess, no generated dosbox.conf. The guest sees a DOS; the host
implements what that DOS does.

## Usage

	dosemu [options] PROGRAM.EXE [args...]
	dosemu config.cfg

Options:

	--help              Show usage
	--version           Print version
	--window            Open an SDL window (default: headless)
	--machine=NAME      PC machine type (default: svga_s3)
	--cpu=NAME          CPU type (default: auto)
	--memsize=N         DOS memory in MB (default: 16)
	--verbose, -v       Trace DOS syscalls

## Example .cfg

See `examples/example.cfg` for a documented sample. Minimal:

	program  = PROG.EXE
	args     = /q /v
	drive_C  = ${HOME}/dos
	drive_D  = /mnt/sources
	memsize  = 16
	cputype  = 486

## License

GPLv3. dosbox-staging is GPLv2-or-later (compatible). Third-party
attributions in `docs/CREDITS.md`.

## Related Projects

- [cpmemu](https://github.com/avwohl/cpmemu) — CP/M 2.2 emulator; the
  translation-layer template and the origin of the `.cfg` format
- [qxDOS](https://github.com/avwohl/qxDOS) — iOS/Mac DOS emulator, also
  built on dosbox-staging; source of the `dosbox_bridge` pattern

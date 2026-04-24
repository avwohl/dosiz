# C toolchains under dosemu -- field guide

Accumulated lessons from running real DOS C toolchains through dosemu.
Target audience: us, later, specifically for a MicroPython port but
applicable to any project compiling C to DOS targets under this
emulator.  Things we spent hours rediscovering the first time so we
don't do it again.

Scope: the three realistic paths to a working C binary under dosemu.

    Toolchain        Target                C dialect  Status
    ---------        ------                ---------  ------
    DJGPP            32-bit DPMI (go32-v2) C99        works (see cpp.exe caveat)
    Open Watcom      32-bit DOS/4G LE      C89        works (see wlink.lnk fix)
    Open Watcom      16-bit DOS MZ         C89        works (needs lib286)

No other toolchains investigated seriously.  Turbo C, MSC 6, Borland
C++, Zortech etc. all predate dosemu's DPMI/LE emulation arc and were
not tried.


## TL;DR: which toolchain for what

- **MicroPython-style port**: DJGPP.  C99 required (designated init,
  inline, VLAs, compound literals, // comments, mixed decls).  DJGPP
  libc 2.05's newlib has decent stdio, stdint.h, pthread-ish stubs,
  and real 64-bit long long.  Watcom's C89 will not compile mainline
  MicroPython.
- **New C89 project you control**: Open Watcom 32-bit DOS/4G.
  Faster, less ceremony, no DPMI-host weirdness.
- **Size-constrained (< 640 KB) tool**: Open Watcom 16-bit DOS.
  Pure MZ format, no extender, but you're in the 16-bit ptr world
  forever.
- **Linking against closed-source vendor libs shipped for Turbo C /
  MSC / Borland**: out of scope, not tested.


## Decision tree for picking a compiler

    Does your code use C99 features?
      |--yes--> DJGPP 32-bit (required)
      |--no---> Does it need > 640 KB RAM?
                  |--yes--> Open Watcom 32-bit DOS/4G
                  |--no---> Open Watcom 16-bit DOS
                              (smaller binaries, no extender)


## DJGPP setup

DJGPP produces 32-bit DPMI-client binaries.  The go32-v2 stub (MZ
wrapper) sits in front of a COFF payload; when DOS runs the binary,
go32-v2 probes for a DPMI host, switches to PM, and jumps to the
COFF main.  Dosemu's ring-3 DPMI host handles this natively since
commit `2bafe81`.

### Getting DJGPP

- **Pre-built from delorie**:
  <https://www.delorie.com/pub/djgpp/current/> has gcc 9.3 (gcc930b.zip)
  and gcc 12.2 (gcc122b.zip).  Each is a ~20-30 MB zip of the install
  tree (bin/, lib/, include/).  The docs refer to this as "delorie"
  and the zipsets as `djdev205`, `djlsr205`, `gccNNb`, etc.
- **Cross-compile toolchain** for rebuilding our own DJGPP binaries
  from Unix: <https://github.com/andrewwutw/build-djgpp>.  Produces
  `i586-pc-msdosdjgpp-{gcc,g++,ld,...}` on PATH.  Used by
  `tests/djgpp/build.sh` when a DJGPP cross-compiler is present.

### Environment DJGPP wants

    DJGPP=C:\DJGPP\DJGPP.ENV    required by DJGPP's _init code
    PATH=...;C:\DJGPP\BIN       for gcc's internal spawns

dosemu inherits the host-side `DJGPP` env var (passed through by
`build_env_block()` in src/bridge.cc).  Set `DOSEMU_PATH='C:\DJGPP\BIN'`
to extend the DOS PATH beyond the hardcoded `C:\`.

### The cpp.exe stack smash

DJGPP libc 2.05's `__crt0_load_environment_file` (in
`src/libc/crt0/c1loadef.c`) has a stack-buffer overflow that manifests
only for specific programs with specific stack layouts -- gcc 9.3 and
12.2 `cpp.exe` are two known victims.  The overrun pops 4 bytes of
djgpp.env content as a return address and traps #UD at a bogus EIP.

Our diagnosis: `patches/djgpp-libc-c1loadef-stack-smash.patch` replaces
the unbounded `alloca(fsize)` with a realloc-grown malloc buffer.
Verify script in `patches/verify-c1loadef-patch.sh`.  Details in
`docs/djgpp-libc-cpp-crash.md`.

**Workaround if you're compiling and cpp.exe is crashing**: invoke
gcc directly without the driver wrapper, or use a trimmed djgpp.env
(< 2683 bytes of expanded content).  The crash threshold is exact.

Our own DJGPP-built fixtures (DJ_WRITE through BIGTEST) are unaffected
because their startup call chain doesn't put a return address at the
offset the overrun reaches.  But `cpp.exe` specifically does.

### DJGPP libc quirks we've hit

- `putenv` **copies** the passed string (non-POSIX, see
  `src/libc/compat/stdlib/putenv.c`).  Convenient when patching
  __crt0_load_environment_file to malloc+free its expansion buffer,
  breaks any assumption of POSIX semantics.
- The go32-v2 stub **clears `[DS:0x764]`** (argv[0] scratch) after
  PM switch on nested exec.  We work around this in `AH=3D` by
  reconstructing the path when we see a nested process opening
  `""` followed by `":\..."` tail.  See bridge.cc around the
  `dos_path.empty()` branch.
- 2015-era (libc 2.05) DJGPP programs expect `AH=65 AL=02/04/06/07`
  to return the 5-byte `[id + far ptr]` format, not the 41-byte
  general-info struct.  dosemu gets this right now (commit
  `4062658`), but if you add more 2000-era DJGPP tools and they
  crash with "recursive-fault loop vec=6 cs:eip=0037:<bogus>", it's
  this class of bug.
- 2000-era DJGPP's `setlocale` pokes `AH=63` looking for a
  properly-terminated DBCS lead-byte table.  DS:SI=0:0 (pointing at
  the IVT) makes it interpret interrupt vectors as DBCS ranges and
  misbehave.  dosemu installs a `[0,0,0,0]` terminator at linear
  `0x0900` (commit `44c9e19`).

### DJGPP under FreeCOM (interactive shell)

Our FreeCOM fixture COMMAND.COM (FreeDOS 0.86, XMS_Swap build)
interacts with DJGPP children through an additional layer:

- FreeCOM's init reads its own MCB at `_psp-1` to derive
  `SwapTransientSize` (how many paras to request back after each
  child exec).  dosemu writes a valid MCB header at PSP_SEG-1 so
  `mcb_size` reads a real value (commit `bcfcaeb`).  Without this,
  FreeCOM's REPL hangs silently after any external spawn.
- When a DJGPP (PM) child exits back to FreeCOM (RM), `cpu.code.big`
  and `cpu.idt` are still set to DJGPP's values.  dosemu snapshots
  both at AH=4B entry and restores on exit (commit `493d1c6`).
  Without this, the callback-trampoline IRET decodes as 32-bit IRETD
  and pops garbage CS:EIP.


## Open Watcom setup

OW produces either 32-bit DOS/4G-bound LE binaries (via wcc386 +
wlink + dos4gw/wstub) or plain 16-bit DOS MZ binaries (via wcc +
wlink).  Both work through dosemu as of this session.

### Getting Open Watcom

Only the Windows-x86 installer is useful -- the Linux and macOS ones
are host-platform ELF/Mach-O runnable binaries of the tools, not what
we want.  We want the DOS binaries (`binw/wcc386.exe` etc.) to run
*under dosemu*.

    curl -LO https://github.com/open-watcom/open-watcom-v2/releases/download/Current-build/open-watcom-2_0-c-win-x86.exe
    7z x -y open-watcom-2_0-c-win-x86.exe 'binw/*' 'h/*' 'lib386/*' 'lib286/*'

Put the extracted tree at `~/ow` (what dosemu's tests assume).  Total
~15 MB after extraction.

### The wlink.lnk gotcha

**The official OW v2 release ships no `binw/wlink.lnk`.**  Without
one, wlink's default system-def synthesis produces DOS/4G LE binaries
that dos4gw.exe rejects with:

    DOS/4GW fatal error (1012): FOO.EXE is not a WATCOM program

This is a distribution bug in OW v2, not a dosemu issue -- confirmed
by running OW's own `owcc` driver native on Windows against the same
release.

**Fix**: drop `patches/watcom-wlink.lnk` (mirrors the Watcom 11.0c
`wlsystem.lnk` minimal `system dos4g` block) into
`$WATCOM/binw/wlink.lnk`.  See `docs/watcom-setup.md` for the full
walkthrough.

### Environment Watcom wants

    WATCOM=C:\                  install root, mapped to DOS drive
    INCLUDE=C:\H                header search path (wcc386 looks here)
    DOSEMU_PATH=C:\BINW         extends DOS PATH so wstub finds dos4gw

dosemu passes these through from the host env via `build_env_block()`.
Add more via `DOSEMU_PATH` as needed (semicolon-separated DOS path).

### C dialect is C89 only

Watcom wcc/wcc386 is strictly C89.  No:

    for (int i = 0; ...)        // C99 mixed decls
    int arr[n];                 // VLA
    struct foo x = {.a = 1};    // designated init
    (struct foo){...}           // compound literals
    // one-line comments        // reject with -s ANSI; accept with -oh extensions

Declarations must be at the start of a block.  MicroPython mainline
won't compile with Watcom without a significant preprocessing pass.

### 32-bit DOS/4G pipeline

    cd ~/ow
    cat > hello.c <<'EOF'
    #include <stdio.h>
    int main(void) { printf("hello\n"); return 0; }
    EOF
    cat > link.cmd <<'EOF'
    system dos4g
    file hello
    name hello.exe
    EOF
    WATCOM='C:\' INCLUDE='C:\H' DOSEMU_PATH='C:\BINW' \
      dosemu binw/wcc386.exe hello.c
    WATCOM='C:\' DOSEMU_PATH='C:\BINW' \
      dosemu binw/wlink.exe @link.cmd
    DOSEMU_PATH='C:\BINW' dosemu hello.exe

The compile step emits `hello.obj` (OMF).  The link step reads OMF
COMMENT records (0xA3 "default library") embedded by wcc386 and
pulls in `clib3r.lib` automatically -- you don't need to list it.

### 16-bit DOS pipeline (system=dos)

Same shape, different tools:

    WATCOM='C:\' INCLUDE='C:\H' DOSEMU_PATH='C:\BINW' \
      dosemu binw/wcc.exe -ml hello.c          # -ml = large memory model
    WATCOM='C:\' DOSEMU_PATH='C:\BINW' \
      dosemu binw/wlink.exe @link16.cmd        # link16.cmd: "system dos"
    dosemu hello.exe

Needs `lib286/` installed.  Pure MZ output, no extender, runs directly.

### Memory model flags

Watcom's 16-bit memory models (`-m*`):

    -ms    small     (64K code + 64K data)         int = 16
    -mm    medium    (any code + 64K data)         int = 16
    -mc    compact   (64K code + any data)         int = 16
    -ml    large     (any code + any data)         int = 16
    -mh    huge      (any code + huge data)        int = 16

For 32-bit (wcc386), all pointers are 32-bit flat -- no memory model
flag needed.

### What still doesn't

Nothing substantive.  The Watcom pipeline is complete through dosemu
as of 2026-04-23.


## Bound extenders: DOS/4GW vs DOS/16M vs PMODE/W

DOS extenders are bundled with the compiler output -- the first
paragraphs of an MZ contain the extender's loader, the tail contains
the LE/LX/REX payload.  dosemu auto-detects extender signatures in
the MZ stub (see `load_program_at` in bridge.cc) and treats the file
as plain MZ when detected, letting the extender's own loader handle
the PM switch.

Signatures we recognize:

    "DOS/4G"     DOS/4GW, DOS/4G (Rational Systems, ~Tenberry)
    "DOS/16M"    Phar Lap 286 extender (16-bit PM)
    "PMODE/W"    PMODE/W (freeware DOS extender, used by id Tech 1 etc.)

Not recognized -- so handled as plain MZ with no extender
passthrough, which may or may not work:

    DOS/32A      newer Tenberry / Supernar replacement
    CauseWay     Devoe's CauseWay
    Borland PE   Borland PowerPack
    PharLap TNT  Phar Lap's 32-bit extender

Expand the detection list in `bridge.cc` if you need one of these.

### Which extender does what

- **DOS/4GW**: the extender shipped with Watcom.  Loads LE.  The
  stub that's embedded in your hello.exe looks up `dos4gw.exe` on
  PATH at runtime and chains to it.  Our `DOSEMU_PATH=C:\BINW`
  makes this work.  Output: LE format, 32-bit, flat.
- **DOS/32A**: drop-in replacement for DOS/4GW.  Supposedly
  faster.  Not tested under dosemu.
- **PharLap / DOS/16M**: 286 16-bit PM.  Not tested under dosemu.
- **PMODE/W**: smaller stub (~10 KB) than DOS/4GW (~260 KB).  Our
  LE loader handles PMODE/W-bound binaries via the extender-bypass
  path (no special code).


## dosemu-specific knobs that matter

### Environment variables (host side)

    DOSEMU_PATH          Appended to the hardcoded DOS PATH="C:\".
                         Colon-separated on host, semicolons in the
                         DOS env block.  Typical: C:\BINW (Watcom),
                         C:\DJGPP\BIN (DJGPP).

    DOSEMU_DPMI_RING0    Opt out of ring-3 DPMI (default is ring-3
                         since 2bafe81).  Set only if you're running
                         legacy ring-0-dependent DPMI tests.

    DOSEMU_TRACE         Enable verbose INT 21h / INT 31 / XMS / INT16
                         tracing.  One print per call; useful when
                         debugging "why does this program fail at
                         startup".

    DOSEMU_4B_TRACE      AH=4B entry/exit detail -- CPU state, LDT
                         restores, IDT base, IRET frame bytes.

    DOSEMU_EXC_TRACE     PM exception dispatch detail -- fault vector,
                         CS:EIP at fault, ring-0 stack frame, bytes
                         at the failing EIP.  Essential for debugging
                         "recursive-fault loop" symptoms.

    DOSEMU_CPU_TRACE     Forces core=normal and hooks the interpreter
                         for per-instruction trace.  Very slow; only
                         for last-resort debugging.

    WATCOM, INCLUDE,     Pass-through to the DOS env block.  Needed
    LIB, LIBPATH         for Watcom toolchain.  DJGPP uses DJGPP
                         (already passed through).

### Env block limitations

The DOS env block is ENV_BYTES (currently 4 KB) total.  Each var
is ASCIIZ.  We truncate host-side values > 200 bytes as a sanity
cap.  If a host env var like `PATH` is very long and you want it
in the DOS env, stage it via `DOSEMU_PATH` (which is purpose-built
for extending the DOS PATH) rather than hoping the full host PATH
passes through -- it doesn't.

### Drive mapping

dosemu maps the host current directory to `C:\` at startup.  There's
no way to mount multiple drives; everything has to be under one
host tree, or `cd`'d into before invocation.  For toolchains with
complex layouts (Watcom's `binw/`, `h/`, `lib386/`, ...), run dosemu
from the install root so everything appears under `C:\`.


## Binary identification cheat sheet

Given a DOS `.exe`, how do you tell which toolchain produced it?

    # DJGPP COFF-go32 binary (MZ stub + COFF payload)
    head -c 2000 foo.exe | strings | grep -i 'go32-v2\|stub loader'
    # output will contain "go32-v2" or "DJ Delorie"

    # Watcom DOS/4G LE binary
    head -c 2000 foo.exe | strings | grep -E 'DOS/4G|Tenberry|Rational Systems'

    # Watcom 16-bit DOS MZ
    file foo.exe                  # "MS-DOS executable" no extender
    # Plus:
    python3 -c "d=open('foo.exe','rb').read(); print(d.find(b'LE'), d.find(b'LX'))"
    # both -1 means pure MZ (16-bit DOS)

    # PMODE/W
    head -c 2000 foo.exe | strings | grep 'PMODE/W'

    # Generic LE detection (any extender)
    python3 -c "d=open('foo.exe','rb').read(); off=d.find(b'LE'); print(f'LE at 0x{off:x}' if off>=0 else 'no LE')"


## Library / runtime caveats by toolchain

                          DJGPP libc 2.05         Watcom clib3r.lib
                          ---------------         -----------------
    C dialect             C99                     C89 only
    int size              32-bit (long 32, LL 64) 16 or 32 by model
    file I/O              POSIX-ish (open, etc.)  DOS-ish (_dos_open)
    stdout buffering      line-buffered on TTY    fully buffered
    printf %lld           supported               %Ld or %I64d
    malloc                from PM heap            conv mem (16-bit)
                                                  pm heap (32-bit)
    stdint.h              present                 missing (define yourself)
    pthread               pthread-stub (no-op)    missing
    long long             64-bit real             32-bit only unless 386+
    putenv copies         YES (non-POSIX)         YES (standard)
    setlocale             minimal                 minimal

### For MicroPython specifically

MicroPython mainline assumes:
- `<stdint.h>` with proper uint8_t / uint32_t / int64_t types
- Mixed declarations and code (C99)
- `__attribute__((...))` or equivalent
- `long long` as 64-bit
- Some POSIX-ish file I/O

All pointing at **DJGPP**.  If `cpp.exe` crashes under your djgpp.env,
either apply our patch (see above) or invoke gcc via `gcc.exe` directly
with `-E` suppressed (the driver sometimes runs cpp transparently).
Build tips:

- Use make from delorie (`mak44b.zip`).  We've regression-tested
  make through FreeCOM (MAKE test in run.sh).
- Keep djgpp.env expanded size **< 2683 bytes** to dodge the
  cpp-crash threshold.  Delete unused FOO=%DJDIR%/... lines.
- Link with `-static` to avoid runtime library path issues.
- If you're porting modules with assembly, write in 386 AT&T
  syntax for `gas` (DJGPP's binutils).


## Known-good archive URLs

Not guaranteed to be stable; verify before relying.

    DJGPP base  (djdev205.zip, djlsr205.zip, gccNNb.zip, binutils):
      https://www.delorie.com/pub/djgpp/current/

    DJGPP build toolchain (Unix cross-compile):
      https://github.com/andrewwutw/build-djgpp

    Open Watcom v2 Current-build:
      https://github.com/open-watcom/open-watcom-v2/releases/download/Current-build/

    Open Watcom 11.0c (commercial release zips, includes working
    wlsystem.lnk):
      https://openwatcom.org/ftp/archive/11.0c/zips/
      (note: Sybase license; don't redistribute)

    Watcom C/C++ earlier commercial releases:
      https://winworldpc.com/product/watcom-c-c/

    FreeDOS FreeCOM (for interactive shell testing):
      https://github.com/FDOS/freecom

    GNU utilities built for DJGPP (grep, sed, make, etc):
      https://www.delorie.com/pub/djgpp/current/  (under contrib/)


## Patches we carry

    patches/djgpp-libc-c1loadef-stack-smash.patch
        Fixes cpp.exe crash (see docs/djgpp-libc-cpp-crash.md).
        Drafted for upstream; DJGPP libc hasn't had a release since
        2015.  Includes verification script.

    patches/watcom-wlink.lnk
        Drop-in wlink config that makes OW v2's wcc386 + wlink
        produce binaries dos4gw.exe actually accepts.  Needed
        because v2 ships no working wlink.lnk.


## Regression testing your port

- Add a fixture under `tests/djgpp/dj_<thing>.c`; `tests/djgpp/build.sh`
  will pick it up if DJGPP cross-compile is on PATH.  Commit the
  built `.exe` -- CI doesn't need a DJGPP cross-compiler.
- Name the marker you print `dj-<thing>=ok` (on success) and add an
  entry to `tests/djgpp/run.sh`.  Pattern is one-line-per-test with
  a `grep -F "dj-<thing>=ok"` check.
- For Watcom tests, the `WATCOM32` / `WATCOM16` gates in run.sh
  already run a full compile-link-run cycle.  If you want to
  regression-test a *specific* Watcom program, add it to that block
  similarly.

For a MicroPython port, the natural fixtures:

    tests/djgpp/dj_mpy_smoke.c    // basic module import + expression eval
    tests/djgpp/dj_mpy_math.c     // int arithmetic, ensure long long works
    tests/djgpp/dj_mpy_file.c     // file I/O through MP's vfs

Each linked against your MicroPython static build, printing a
deterministic marker on success.  Keep the test binaries < 4 MB so
they don't dwarf BIGTEST.EXE in the commit tree.


## Things we considered adding and didn't

- **Ship a pre-bundled DJGPP install**: too big (~100 MB) for a
  repo.  Reference-link to delorie and andrewwutw instead.
- **Auto-detect DJGPP and export PATH**: surprise behavior.  Leave
  it to the user's invocation.
- **Bundle Open Watcom in CI**: same size argument.  CI runs our
  committed .EXE fixtures only; the Watcom path is user-locally
  gated in `run.sh`.
- **Embed dos4gw.exe directly in our LE loader**: considered and
  rejected.  The user-facing `DOSEMU_PATH` setup is explicit and
  lets people pick their own extender version.


## Session history

This doc is the condensed result of a 2026-04-22 to 2026-04-23
session that went from "33/33 suite, can't run cpp.exe" to "39/39
suite, full DJGPP + Open Watcom 32-bit + 16-bit pipelines all
working through dosemu".  Twelve-ish commits, each root-caused to
a specific DOS interface we were approximating badly.  See `WIP.md`
for the session-by-session play-by-play if you need to trace why a
specific behavior is the way it is.

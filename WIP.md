# What's left — 2026-04-22 backlog

Work this list in order, top to bottom. Small wins first; each
bullet is a commit or a small series.  Mark done with ~~strike~~
when landed.  Suite is 29/29 at the start of the backlog.

## Small (single-session)
1. ~~**LE selector fixups (types 0x02/0x03/0x06).**~~  Already done
   in commit d21def3 (2026-04-21); the "Next-session pick list"
   in the earlier-session notes below was stale when I wrote this
   backlog.
2. ~~**`flex -o out.c`.**~~ Investigated; argv parsing is correct
   on our side (verified via DJ_ARGV).  With `-o` flex probes
   the output file for read (AL=00, ENOENT → bails "can't open
   out.c").  Pre-creating the file changes the error to
   "could not create " (trailing space -- internal outfile
   variable is cleared by flex's freopen-on-stdout dance).
   Concluding this is a flex-port bug, not a dosemu issue.
   Workaround: let flex write to its default `lex.yy.c`
   (equivalently `lexyy.c` in 8.3) and rename afterwards.

## Medium (multi-session)
3. ~~**DOS/4GW transfer-buffer allocation.**~~ Also already fixed
   somewhere along the way.  With `DOSEMU_DPMI_RING3=1
   DOSEMU_LE_AS_MZ=1`, `wcc386.exe hello.c` now runs through full
   DOS/4GW init and compiles a tiny C program to a valid 305-byte
   OMF object file.  No transfer-buffer error anymore.  Linking
   with wlink.exe also mostly works (just missing a system
   definition file for the DOS/4G target, which is a config issue
   not a dosemu bug).
4. ~~**Large LE binaries (`wd.exe` ≥600KB).**~~ Also already fixed.
   `wd.exe` (710KB Watcom debugger) runs through init and emits
   its usage banner.  `wcl386.exe` runs.  Large-LE arena seems
   to no longer be a blocker.  (Subsequent compile+link failures
   are Watcom config issues -- missing system definition + libs --
   not dosemu bugs.)
5. ~~**DJGPP→DJGPP nested exec.**~~  **DONE.**  Parent spawns a
   DJGPP child, child runs to completion, parent resumes cleanly
   and continues past spawnlp.  Ten layered root causes found
   and fixed (suite now 30/30, includes new DJ_DJE regression
   gate).
   - **#5.1 (b46f43a):** Path reconstruction for empty `[DS:0x764]`.
   - **#5.2 (9f8cc2a):** Zero child MCB before MZ load.
   - **#5.3 (281e20f):** Preserve client's RM SP in dpmi_entry
     IRETD frame instead of hardcoding 0xFFFC.
   - **#5.4 (b06c848):** Rewrite parent's LDT[1..5] bases on resume.
   - **#5.5 (b06c848):** CPU_SetSegGeneral (PM descriptor reload)
     instead of SegSet16 (val<<4) for PM parent restore.
   - **#5.6 (9b020e9):** Unconditionally restore parent's CR0 on
     AH=4B return -- child exited with PE=1 but parent was RM.
   - **#5.7 (9b020e9):** Snapshot+restore full 2KB LDT memory
     around child execution -- child's dpmi_entry wipes all 256
     slots, clobbering parent's DPMI allocations in slots 6+.
   - **#5.8 (99a63cf):** Snapshot+restore s_pm_exc[] (PM exception
     handlers).  Child's libc installs its own via DPMI AX=0203;
     those handler addresses point into child-memory that's freed
     on exit.  Any post-spawn parent fault would have dispatched
     to the stale child handler and recursive-faulted.  With this
     fix, the fault-dispatcher no longer loops, and the DJGPP
     SIGSEGV message now correctly reports "program=DJ_DJE.EXE"
     (the parent) with LDT[6] base=0x120000 (parent's correct
     base, restored from snapshot).

   - **#5.9 (d607b3b):** Save/restore FS and GS in ProcessState,
     and use parent's SAVED selectors (CS/DS/SS/ES/FS/GS) via
     CPU_SetSegGeneral on PM restore instead of hardcoded
     starter-set values.  DJGPP's libc runs on LDT[6..8], not
     the starter-set LDT[1..5].
   - **#5.10 (d607b3b):** Snapshot+restore the 4KB PM_CB_STACK
     (ring-0 callback stack) around child execution.  Parent's
     INT 31 pushed its IRETD frame at the top of this stack;
     child's own ring transitions overwrite it, so the outer
     int31 callback's IRETD would pop garbage CS/EIP.
   - **#5.11 (this commit):** `s_int_gate_bits32` must be
     nested-safe.  The outer wrapper sets it to `true`, calls
     the handler, and the handler's AH=4B recursively runs the
     child which re-enters dosemu_int31_bits32.  The inner
     wrapper's exit path resets the flag to `false`, corrupting
     the outer wrapper's state.  Later, the outer handler's
     `set_cf()` reads this stale flag, picks flags_off=4 (16-bit
     gate) instead of 8 (32-bit gate), and writes the updated
     flags word at `ss:esp+4` -- which in a 32-bit IRETD frame
     is the CS slot, not FLAGS.  The `and ~0x0001` mask on CF
     flipped bit 0 of outer CS from 0x37 to 0x36 (RPL=3 -> RPL=2),
     making the IRETD drop parent to CPL=2 and immediately #GP on
     the next `pop ss` of a DPL=3 selector.  Fix: save-and-restore
     `s_int_gate_bits32` in both wrappers.

   **Suite: 30/30 including DJ_DJE (new regression gate for
   DJGPP→DJGPP nested exec).**

## Larger
6. ~~**`make` with real recipes.**~~  **Done.**  GNU make 4.4
   (delorie `mak44b.zip`) runs a Makefile end-to-end: parses
   rules, stats targets, spawns FreeCOM as SHELL, runs the
   recipe, writes target output.  Regression gate MAKE in the
   suite (now 32/32).  Required:
   - INT 21h AH=71h AL=4E/4F/A1 (LFN findfirst/next/close) now
     returns a real DOS error (0x02 = file not found -> ENOENT)
     instead of the conservative "LFN not supported" AX=0x7100
     CF=1.  DJGPP newlib's stat() maps the latter to EINVAL and
     doesn't retry with SFN, so every target-existence check in
     make failed with "stat: <name>: Invalid argument".
   - Run with `TMPDIR=C:\TMP` (with the directory pre-created) so
     DJGPP's temp-dir probe doesn't warn about a Unix-shaped path.
   - Needs COMMAND.COM (our FreeCOM) in the same directory as
     MAKE.EXE for the shell-spawn to resolve.

   Historical notes preserved:
   FreeCOM 0.86 from
   FDOS/freecom release `com086` is in `tests/COMMAND.COM` and
   boots to a prompt; `ECHO`, `DIR`, `CD`, `TYPE`, `VER`, `EXIT`
   all work.  Required fixes:
   - Content-sniffed loader: the file is named `.COM` but is MZ
     format (>64 KB), so `load_program_at` now peeks magic bytes
     before falling back to extension.
   - INT 21 AH=65 AL=05 (filename-character table) now returns a
     real table with `inclFirst=0x21` (otherwise is_fnchar()
     accepts CR/LF and the internal-command match breaks on any
     command-with-arg).
   - INT 21 AH=44 AL=00 sets bit 6 ("more input available") for
     stdin/stdout/stderr so FreeCOM doesn't early-exit on its
     char-device-at-EOF check `(attr & 0xc0) == 0x80`.
   - Added FREECOM regression gate to the test suite (31/31).

   Remaining: spawning external programs through FreeCOM faults
   with "LE client PM exception" -- the spawned DJGPP child's PM
   entry path isn't coming back through the FreeCOM parent's
   resume cleanly (not the same #5 code path -- FreeCOM parent
   is real-mode-only, so the AH=4B restore is simpler).  Needs
   its own session to untangle.
7. **AH=4B AL=5** (Set execution state -- for debuggers).
8. **QEMM parity** -- structural DPMI-host gap mentioned in the
   earlier-session notes below.

## Maintenance / paper cuts
9. **`DPMI_STAGE3.COM`** hangs -- not in CI.  Either fix,
   document, or remove.
10. **`DOSEMU_CPU_TRACE` forcing `core=normal`** is surprising
    (documented in DEBUGGING.md).  See if the JIT can be made to
    honour memory watchpoints so the trace no longer needs to
    downgrade the core.

---

# dosemu WIP — DJGPP RUNS END-TO-END (2026-04-22, arm64 Mac)

**DJGPP programs now execute their main() and produce correct stdout
output under dosemu.**  No SIGSEGV, no crash in libc startup.

Verified on:
- A tiny `write(1, "hello\n", 6)` program: prints `hello`.
- A program that does `memcpy(buf, argv[1], n); write(1, buf, n)`
  with multiple args: prints all args space-separated.
- A program that checks `argc`: gets correct count (1 for no args,
  3 for two args, etc.).

This was achieved by five landed fixes (all in this session):

1. **Mac port** (3d4ee29): dosemu builds+runs natively on arm64
   Darwin.
2. **uint32_t LDT starter-set bases** (14719af): fixed seg*16 overflow
   in `dosemu_dpmi_entry`.  Resolved "0xF4 DS-load" symptom but still
   crashed elsewhere.
3. **AX=0300 scratch stack relocated** (3ec109d): when client's
   SS=0 in dpmi_regs, use 0x9000:0xFFF0 (top of conv mem) instead
   of 0x0050:0x0F00 which lands inside the MZ stub image at linear
   0x1400 and was corrupting `call read_section` setup bytes.
4. **AX=1687h (DPMI detect) SI=0** (this commit): tells go32-v2
   it doesn't need to allocate private DPMI-host memory.  Without
   this, go32-v2 calls AH=48 and changes ES to the allocated block
   before the DPMI mode-switch far-call, making stubinfo.
   psp_selector alias that block instead of the PSP.
5. **LDT[4] forced to alias PSP_SEG; PSP[0x2C] rewritten with PM
   env selector** (this commit): DJGPP's `_setup_environment` reads
   PSP[0x2C] via stubinfo.psp_selector expecting a *DPMI selector*
   (not a DOS segment number).  Our DOS PSP has the raw ENV_SEG
   value at PSP[0x2C]; we overwrite it with a PM-selector alias
   for the env block.  We also force LDT[4] to alias PSP_SEG
   regardless of go32-v2's ES-at-entry (which it leaves at env_seg
   after scanning env for PATH=), so stubinfo.psp_selector actually
   aliases the PSP.

All 37 existing local fixtures (HELLO, DPMI_* non-STAGE ones, LE_MIN,
SPAWN, etc.) pass on arm64 Mac.

**Verified working DJGPP features** (all via DJGPP 2.05 libc + gcc
12.2 built from andrewwutw/build-djgpp v3.4):
- `printf` with format specifiers (`%d`, `%s`, `%.6f`, `%lX`, etc.).
- `fputs`, `fwrite`, `fread`, stdio buffering (stdin/stdout).
- `fopen` / `fprintf` / `fclose` / `fgets` (file create, write, read).
- `malloc` / `free` (100-block 1KB round-trip, no leaks).
- `argc` / `argv` (correct count and contents from DOS cmd tail).
- `getenv` (COMSPEC, PATH, HOME, USER, LANG all accessible).
- Exit-code propagation (return N from main → dosemu rc=N).

**Sixth fix (5c3eaaf)**: AH=71 (LFN API) returns `AX=0x7100 CF=1`
instead of `AX=0x0001 CF=1`.  DJGPP's libc checks `AX == 0x7100`
to decide whether to retry with the short-filename API (AH=3C
etc.); the default "invalid function" reply made it incorrectly
report EINVAL and never fall back.  `fopen("...", "w")` now creates
files.

**Seventh fix**: `build_env_block` now puts the full relative program
path into argv[0] (DOS-style slashes, uppercase), not just the
basename.  The go32-v2 stub opens argv[0] to read its own COFF
payload; a basename-only path meant programs in subdirectories
(`tests/DJ_*.EXE`) couldn't find themselves.

**Eighth fix**: `build_psp`'s command-tail writer wraps any arg that
contains whitespace in double-quotes, backslash-escaping any
existing `"` or `\`.  DJGPP's crt0 argv parser honors this
convention, so multi-word args passed from the host shell arrive
as single argv entries.

## DJGPP regression test suite

`tests/djgpp/` contains seven DJGPP-compiled smoke tests covering
the features above.  `tests/djgpp/run.sh` runs all of them from
the repo root and prints `PASS`/`FAIL` per test:

  DJ_WRITE    direct write() to stdout
  DJ_PRINTF   printf with %d %s %.*f %lX + exit-code propagation
  DJ_ARGV     argc/argv from PSP cmd tail (also verifies quoted args)
  DJ_ENV      getenv(COMSPEC), getenv(PATH)
  DJ_FILE     fopen/fprintf/fclose/fgets round-trip (needs LFN fix)
  DJ_MALLOC   malloc/free stress (64 blocks × varying sizes)
  DJ_STDIN    fread from stdin piped in

Rebuild the binaries with `tests/djgpp/build.sh` (needs DJGPP
cross-compiler on PATH); the committed binaries are what CI runs.
CI step `DJGPP integration suite` drives `run.sh`.

---

(older notes follow)

# dosemu WIP — end of session 2026-04-22 (DJGPP push + first green CI)

## macOS (arm64) port — 2026-04-22

Dev host moved Linux x86-64 → Darwin arm64 (Apple Silicon, 32GB).  dosemu
now builds and runs natively.  Changes:

- `src/CMakeLists.txt`: `if(APPLE)` branch.  Drops GNU-ld
  `--start-group`/`--end-group` (ld64 does multi-pass arch resolution),
  `stdc++fs`/`atomic` (libc++), and Linux-only deps
  (SDL2_net, slirp, fluidsynth, asound, Xi, libGL).  Adds Mac frameworks
  (CoreFoundation, CoreAudio, AudioUnit/Toolbox, CoreMIDI, IOKit,
  OpenGL) and a BUILD_RPATH for meson's `libmt32emu.dylib`.
- `src/CMakeLists.txt`: `link_directories(${GLIB_LIBRARY_DIRS})` so
  Homebrew's `/opt/homebrew/lib` glib is found (pkg_check_modules gives
  bare names in `${GLIB_LIBRARIES}`).
- `src/bridge.cc`: headless mode forces `SDL_VIDEODRIVER=dummy` on
  `__APPLE__`.  SDL's `offscreen` driver on macOS is EGL-based and
  fails to init without a real GL loader even for non-GL output modes.
  Also forces `[sdl] output=texture` to bypass sdlmain's
  `opengl_driver_crash_workaround`, which still OR's `SDL_WINDOW_OPENGL`
  into CreateWindow flags when the default SDL render driver is opengl.
- dosbox-staging configured with
  `-Duse_sdl2_net=false -Duse_fluidsynth=false -Duse_slirp=false
  -Duse_opengl=false` (plus alsa auto-disabled off-Linux).  `config.h`
  ends up with `C_OPENGL 0`, killing the unconditional GL-context probe
  at `sdlmain.cpp:1329` inside `SetWindowMode`.

DJGPP toolchain: no brew formula.  Used
`andrewwutw/build-djgpp` v3.4 `djgpp-osx-gcc1220.tar.bz2` (60MB,
x86_64 — runs under Rosetta 2 on arm64) installed to `~/djgpp/djgpp`.
Added `~/djgpp/djgpp/bin` to PATH; `DJGPP=~/djgpp/djgpp/setup/djgpp.env`.

Smoke tests green on arm64 Darwin:
- `tests/HELLO.COM` → `dosemu-hello-ok` rc=0
- `tests/DPMI_INTEGRATION.COM` → `dpmi-integration=ok` rc=0
- A fresh-compiled djecho reaches the exact same `0xF4` DS-load state
  described below (see "the remaining mystery").

## 0xF4 mystery SOLVED (2026-04-22, arm64 Mac)

Traced via PSP[0x2C]/stubinfo/LDT dumps from inside the PM-exception
dispatcher (env-gated debug: `DOSEMU_EXC_TRACE`, `DOSEMU_DPMI_TRACE`,
`DOSEMU_LDT_TRACE`).

**Root cause**: `dosemu_dpmi_entry`'s computation of the "starter set"
LDT descriptor bases used `uint16_t` for `cs_base`/`ds_base`/`ss_base`/
`es_base` (= RM_seg * 16).  Any RM segment value >= 0x1000 produces
`seg*16 >= 0x10000`, which overflows uint16 and silently truncates.

Observed with DJGPP go32-v2: at the DPMI entry far-call, its ES register
held 0x2001 (a pre-allocated DOS block).  `0x2001 * 16 = 0x20010`, but
uint16 truncated it to `0x0010`.  That made LDT[4] (the "psp_selector"
per stubinfo[+0x26]) alias linear 0x0010..0x1000F -- i.e. the RM IVT.
DJGPP libc's `_setup_environment` then did:

    movedata(psp_selector, 0x2C, DS, local, 2);   // read PSP[0x2C]
    movedata(*local, 0, DS, env_buf, env_size);   // copy env

The first call reads from linear 0x3C = IVT[0x0F] low word, which holds
`0x00F4` on the BIOS/IVT we expose.  DJGPP then used `0xF4` as a DPMI
selector in the second call, producing the `#GP(err=0xF4)` documented
in the prior session.

**Fix (one-line effect, 4 lines of type changes)**: widen the four
`_base` vars from `uint16_t` to `uint32_t`.  Now LDT[4] aliases
0x20010 as go32-v2 intended, PSP[0x2C] reads no longer hit the IVT,
and the 0xF4 GP is gone.

**Next failure surfaced**: a new first-fault at `cs:eip=000f:0x0022
err=0x400` very early in the go32-v2 stub's PM init.  This is a
different bug that was previously masked -- with LDT[4] aliasing
the IVT, go32-v2 was reading random IVT bytes that happened to keep
the state machine alive until libc's movedata.  With the correct
alias, an earlier code path in the stub now takes a real fault.

**Further investigation (same session, 2026-04-22)**: chased the
`err=0x400` fault through CPU tracing + .data probes.  The fault is a
`pop ss` at stubinfo offset 0x22 (executing the `ds_selector` field as
the `pop ss` opcode 0x17) because control lands there via a
`jmp far cs:[0xe880]` in `___exit`, where `[cs:0xe880]` = `(off=0,
sel=0x000f)`.  Target CS:0 is the stubinfo data at linear 0x1100, not
the intended RM-switch stub code.

Root cause chain:

1. **`__stubinfo` = NULL at runtime.**  The only code path that writes
   `__stubinfo` (at crt0.S:267, `movl %eax, __stubinfo` after
   `call ___sbrk`) does run, but sbrk returns 0 because
   `__what_size_app_thinks_it_is` (at 0xe8d6) starts at 0 instead of
   its .data-file value (0x14f78).

2. **.data not fully loaded into PM memory.**  Probes at fault time
   show:
   - `*0x12e810 = 0x1273c` (matches expected 0x12734+8 after a runtime
     IMMEDIATE-mov write in crt0.S line 225 -- so .data wasn't really
     read; the runtime instruction supplied the value).
   - `*0x12e814 = 0` (expected 0x12 from .data -- missing).
   - `*0x12e81c = 0` (expected 0x270 from .data -- missing).
   - `*0x12e8d6 = 0x80858` (runtime-accumulated from 0, never had the
     initial 0x14f78).

3. **INT 21h trace confirms go32-v2 skipped the .data read.**
   The stub's trace shows AH=3D (open) -> AH=3F CX=6 (MZ magic) ->
   AH=42 DX=0x800 (seek COFF) -> AH=3F CX=0xA8 (COFF header) ->
   AX=0300 sim-RM-INT -> AH=42 EDX=0x1800 (seek .text) -> AH=3F
   ECX=0xD000 (read .text, ~52KB) -> **AH=3E close**.

   Per stub.asm line 467-470, there should be a SECOND read_section
   call for .data (and then BSS zero-fill, file close, DOS mem free).
   The trace jumps straight to close after .text.  Something in
   read_section is erroring out on .text's path and taking a
   short-circuit return that skips .data.

**Further pinpointing (same session, 2026-04-22)**: downloaded DJGPP
source (`djlsr205.zip`), extracted `src/stub/stub.asm` and `src/libc/
crt0/crt0.S`.  Compared against execution trace.

The go32-v2 stub code at offsets 0x2EE and 0x300 in the stub image
(= linear 0x13EE and 0x1400 after MZ load at 0x1100) are the two
`call read_section` instructions (for .text and .data respectively).
The CPU trace shows EIP progresses correctly through 0x2EE -> 0x33A
(read_section entry) -> ... -> returns to 0x2F1 -> 0x2F6 ->
**then garbage from 0x2FB onwards** (executing random bytes as
instructions).

Memory dump at fault time (`DOSEMU_EXC_TRACE=1`, custom probe) shows
stub bytes at 0x13FA..0x13FF have been **overwritten** from the
expected `08 66 8b 0e 27 08` (the end of `mov edi,[0x823]` + all of
`mov ecx,[0x827]`) to `c8 20 00 f0 47 30`.  So the CALL itself
didn't happen -- the 3-byte `e8 37 00 call 0x33a` at 0x300 is intact,
but its setup instructions at 0x2FB-0x2FF are corrupted.

Exactly **6 bytes** corrupted, not a huge range.  That rules out
"entire .text got copied to wrong address via AH=3F with bad DS".
More likely: **our AX=0300 handler writes the simulated-RM CPU state
back to the wrong offset of the dpmi_regs struct** (or writes too
many bytes), clobbering a neighboring 6-byte span of stub code.

**For next session**: diff what our AX=0300 handler writes back to
[ES:EDI+offset] against CWSDPMI's convention.  dpmi_regs is a fixed
0x32-byte real-mode call structure (AX/BX/CX/DX/SI/DI/BP/flags/ES/
DS/FS/GS/IP/CS/SP/SS).  Our handler may be writing past the struct
end into adjacent .data, or computing the struct base with wrong
segment registers.  Specifically: does AX=0300 use the client's DS
when the client has 32-bit mode flags set and the struct pointer is
passed in EDI (not DI), or does it use some other selector?

## Watchpoint data point on [DS:0x19370] (2026-04-22, arm64 Mac)

Added `mem_writed_inline` hook on linear `0x139370` (= client DS base
`0x120000` + `0x19370`) that logs every write.  Rebuilt + ran djecho
with `DOSEMU_DPMI_RING3=1`.

**Result**: the slot is written exactly **one** time during the entire
run, with value **`0x00000000`**.  Never populated.  That confirms
WIP.md's hypothesis word for word — the consumer reads a zero pointer,
dereferences, grabs garbage, tries to load `0xF4` into DS, GP-faults
with `err=0x00F4 EAX=0x000000f4`.

Next step: objdump on fresh djecho COFF to find the WRITER of
`[0x19370]` (not the reader, which is already identified at `0x40f0`
in the movedata wrapper's caller).  Likely a DJGPP library init
routine that's gated on a stubinfo field our DPMI host doesn't provide
correctly.

## Current DJGPP state

All 37 local fixtures + full 60-step CI pipeline green.  `djecho.exe`
and `djasm.exe` now run through:
 1. go32 stub's DPMI init (all INT 31h calls succeed).
 2. COFF load + entry to COFF runtime.
 3. First SIGSEGV in DJGPP libc (DS=0xF4 / `mov ds, [ebp+8]` in a
    movedata wrapper).
 4. DJGPP's PM signal handler runs **end-to-end**: prints the full
    register dump with faulting eip/regs, selector dumps (base/limit
    for cs/ds/es/fs/gs/ss), app-stack range, exception-stack range,
    and call-frame traceback.
 5. Handler's exit cleanup re-faults (currently on a BOUND check at
    0x7724, previously on `cli` at 0x1ACC / `mov ds, 0x37` at 0x7775).
 6. Recursion guard (5-same-fault threshold) catches the loop.
 7. dosemu exits rc=0 with readable SIGSEGV diagnostic on stderr.

The cascade of fixes that got us here:

    commit 09fd59b  CWSDPMI-style exception dispatch (+ring-3 CB alias)
    commit 18d17bd  PM_CB_STACK descriptor D=1 (reg_esp truncation)
    commit f00bd10  AX=0002 cache RPL fix + recursion guard
    commit 2bafe81  IOPL=3 in initial EFLAGS + AX=0001 don't-zero-descriptor

## DJGPP: the remaining mystery

The first real fault (DS=0xF4 at 0x002f:0x7c29 in djecho's libc) has
been narrowed down:

- **Instruction identified** via `i586-pc-msdosdjgpp-objdump` on the
  extracted COFF: `mov %ds, 0x8(%ebp)` in a `movedata`-like wrapper
  at `0x7c20`.  Caller at `0x4177` in a function at `0x40f0` that
  reads a global pointer at `[0x19370]` (in .bss -- zero-initialized)
  and passes fields from the pointed-to struct as movedata args.

- **The pointed-to struct layout matches `__dpmi_regs`** (fs at +0x26,
  ip at +0x2a, cs at +0x2c, ss at +0x30).  Whichever global this is,
  it's meant to hold a real-mode call register block.  When the code
  path fires without the struct ever having been populated (or with
  stale data), the CS field can hold 0xF4 -- a value we never allocate
  as a selector, so loading it into DS #GPs.

- **Allocations verified**: only 4 LDT allocations happen during the
  full djecho run (slots 5/6/7/8 via AX=0000 + AX=000A).  LDT[30]
  is never allocated; 0xF4 doesn't come from our DPMI host response.

Further investigation needs to trace the client's global
initialization path -- specifically, which DJGPP library function
is supposed to populate `[0x19370]` with a valid `__dpmi_regs *`
before anyone in the exit path reads it.  Likely a missing
stubinfo or env-setup piece that our DPMI host doesn't provide
correctly to DJGPP.

**Pragmatic status**: dosemu now terminates cleanly on DJGPP
SIGSEGV (rc=0 with readable diagnostic output).  No actual DJGPP
program execution yet -- main() never runs -- but every underlying
dosemu-side bug that was ALSO on the critical path has been fixed.

## CI: first fully green run in repo history

## CI: first fully green run in repo history

Run `24779875503` (tag `ci-exe2bin-rc`, 2026-04-22) -- **60 steps,
all green**.  This is the first time any workflow in this repo has
executed past the meson-setup step.  History:

- `7eed164` unjammed the YAML parser (unquoted colons in step names).
- `7eef7d8` fixed `meson setup BUILD --buildtype=X SRC` arg order
  -- newer meson rejects that pattern ("unrecognized arguments:
  SRC"); put SRC before the option.
- `5b86668` (`|| true` on the EXE2BIN subshell) got past `bash -e`
  eating EXE2BIN.EXE's rc=255 banner-usage exit.

Triggered by tagging: `git tag ci-<something>; git push origin
ci-<something>`.  Regular `main` pushes don't fire CI (deliberate;
saves 6-10 min per push).



Ring-3 DPMI progressed substantially.  Two new bug categories fixed:

1. **Kernel structures moved above 1MB.**  `GDT_SEG=0x1800`,
   `IDT_SEG=0x1A00`, `LDT_SEG=0x1B00`, `PM_SHIM_SEG=0x1C00`,
   `PM_CB_STACK_SEG=0x1E00`, `TSS_SEG=0x1F00` were in conventional
   memory — a ring-3 DPMI client's 16-bit-RM-aliased selector (base +
   64KB limit) could reach them and corrupt them.  DJGPP's go32 stub
   hit this by memset'ing its transfer buffer (ES=some low seg, limit
   0xFFFF) which included our IDT at 0x1A000.  Renamed to `GDT_BASE`
   etc. at `0x100000..0x109000`; `PM_ARENA_START` moved from 0x100000
   to 0x120000 so the DPMI linear-memory arena sits above the new
   kernel region.

2. **A20 line.**  DOS boots with A20 disabled, which wraps linear
   addresses 0x100000..0x10FFFF back to 0x00000..0x0FFFF (real-mode
   compat).  Writes to our newly-relocated kernel were silently
   landing in the RM IVT and BIOS data area; reads returned bogus
   values (observed: IVT[0x10] = 0x80000FFF instead of F000:xxxx,
   which made the PM-IDT walker skip installing the INT 10h gate).
   `MEM_A20_Enable(true)` is now called both at DPMI entry (in
   `pm_setup_gdt_and_idt`) and at LE launch (in
   `le_install_descriptors`).  All 37 fixtures stay green; LE_MIN
   runs to exit code 0 in the new layout.

**DJGPP djecho.exe now runs:**
- ~76 INT 31h calls successfully (up from ~13 pre-fix).
- go32 stub fully initializes PM, enters COFF code, installs its
  own exception handlers via AX=0202/0203 for vectors 0x00..0x11,
  allocates DPMI memory (128KB + 512KB + 60KB), does several
  sim-RM INT 21h round-trips.
- **Two follow-up fixes landed (commits 661c03c, 8e58220):**
  - AX=0501 prefers pm_arena over MCB, so multi-block allocations
    land above 1MB contiguous, not scattered across the MCB arena
    in conventional memory (which would still put client data near
    our 0x104000 LDT within their extended-limit selector).
  - pm_alloc now zero-fills via mem_writed (fast enough not to
    desync dosbox's timer/IRET bookkeeping, unlike the old
    byte-at-a-time attempt).  DJGPP assumes zeroed allocations.
- Runs millions of CPU instructions in go32+COFF code before now
  hitting a different class of failure: RETF from client code at
  0x002F:0x7856 pops selector 0x0017 (a DS-type LDT entry) as
  new CS, `"RET from illegal descriptor type 0x12"`.  The client
  is popping a struct field it treats as a return-address from
  what looks like a DJGPP exception-handler frame; the push
  sequence (`push [ebx+4]; push [ebx+0x2a]; ...; retf`) suggests
  either DPMI frame-format mismatch OR some earlier corruption
  that left the struct in an invalid state.

**Root cause identified + CWSDPMI-style dispatch landed (commit 09fd59b):**

Downloaded DJGPP source (`djlsr205.zip`) and read `exceptn.S`.  The
handler expects an 8-dword "exception frame" at `[SS:ESP]`:
```
[SP+ 0]  user_exception_return_EIP
[SP+ 4]  user_exception_return_CS
[SP+ 8]  err code
[SP+12]  EIP  (of faulting instruction)
[SP+16]  CS
[SP+20]  EFLAGS
[SP+24]  outer ESP
[SP+28]  outer SS
```
i.e. the handler LRETs through `user_exception_return` (the first two
slots) to unwind, and reads the rest as the CPU state it's handling.
CWSDPMI's `EXPHDLR.C::user_exception()` constructs exactly that layout
before transferring control to the user handler.

Our AX=0203 now (for 32-bit handlers) installs an IDT gate pointing at
a per-vector trampoline which:
1. Runs at ring-0 (PM_CB_SEL DPL=0), reads the CPU-pushed ring-change
   frame on our scratch stack.
2. Builds the 8-dword CWSDPMI frame on a private known-good stack
   (`GDT[15]` = ring-3 alias of `PM_CB_STACK_BASE`) -- CWSDPMI does
   this too because the client's SS:ESP may be corrupt at fault time.
3. Rewrites the stub's IRETD frame so the `66 CF` at the end of the
   callback stub transitions to the user handler at ring-3 with the
   CWSDPMI frame sitting at `[SP+0]`.
4. `user_exception_return` (another trampoline, reached when the user
   handler LRETs through `GDT[14]` = ring-3 alias of CB_SEG) reads the
   frame, restores outer SS:ESP, and IRETDs back to the faulting
   (possibly handler-modified) CS:EIP.

All 37 fixtures still pass.  DJGPP djecho now correctly reaches the
trampoline on its first #GP and runs the user handler -- but that
first #GP fires with `CS=0 EIP=0 outer_SS=0 outer_ESP=0`, meaning the
client's state was already wrecked before the exception.  That's an
earlier-stage bug (probably in our IRETD / RM-callback / ring-3
transition elsewhere); the exception dispatch itself is now correct.

**Second round fixes landed (commit 18d17bd): PM_CB_STACK D=1**
The ring-0 scratch stack descriptor had D=0 (16-bit).  Ring-change
from ring-3 dispatched through the TSS's SS0:ESP0 = (PM_CB_STACK,
0x1000), and CPU pushed the exception frame, but with D=0 only the
low 16 bits of ESP were written by the push logic -- reg_esp kept
the high 16 bits from the client's ring-3 ESP (e.g. 0x0007_0FE8
instead of 0x0000_0FE8).  Every read in our dispatch trampoline then
aimed at a wildly wrong linear address and came back zero.  Setting
bits32=true on GDT[8] fixes this.

With that, DJGPP djecho now runs its PM SIGSEGV handler **end to end**
and prints its full register dump to stderr:

    General Protection Fault at eip=00007c29
    eax=00000000 ebx=0009ffff ecx=000000ff ...
    cs: sel=002f base=00120000 limit=0009ffff
    ...
    Exiting due to signal SIGSEGV

This is a qualitatively new milestone: DJGPP's userspace signal path
is functional under our DPMI host.

**Next gap (for later sessions):**

1. **First real fault.**  Happens at 0x002f:0x7c29, which is DJGPP's
   `__dpmi_int` LEAVEP epilogue -- specifically `popl %ss` (pops a
   saved SS pushed at function entry before the INT 31h AX=0300 call).
   (Source: /tmp/djsrc/src/libc/dpmi/api/d0300_z.S:
   `pushl %ss; ... DPMI(0x0300); ... LEAVEP(... popl %ss; popl %es)`,
   where the macro LEAVEP is from /tmp/djgpp/include/libc/asmdefs.h.)

   The faulting instruction is `8e 5d 08` (`mov ss, [ebp+8]`) -- NOT
   a `popl %ss`.  This is some function in DJGPP libc that takes a
   new SS selector as its first argument and loads it.  Err=0xF4
   decodes to "the passed-in SS value was 0x00F4" (LDT[30], RPL=0),
   which isn't a valid selector for the client.  So the CALLER of
   this function passed a bogus SS.

   **AX=0300 ruled out (2026-04-22).**  303,000+ sim-RM-INT calls
   preserved the ring-change frame every time (zero frame mutations
   observed).

   **Actual instruction identified (2026-04-22, via objdump on the
   extracted COFF from djecho.exe using `i586-pc-msdosdjgpp-objdump`
   from the host's binutils-djgpp package):**
   ```
   7c29: 8e 5d 08    mov %ds, 0x8(%ebp)   ; (not SS as initially decoded)
   ```
   So the fault is loading DS (not SS) from arg1 of a `movedata`-like
   function.  Arg1 at fault time = 0xF4 (matches err).  0xF4 is a
   selector: LDT[30], RPL=0.  Something in djecho's libc grabbed 0xF4
   as a "source selector" to pass to movedata.  Our LDT bitmap only
   records 3 allocations (LDT[5]/[6]/[7] + the starter slots 1-4), so
   LDT[30] was never allocated through AX=0000 -- yet the client has
   this value somewhere.  Likely a pre-initialized constant baked
   into the COFF, or a stubinfo field feeding __djgpp_dos_sel.

   **Practical mitigation landed (commit f00bd10):**
   - AX=0002 cached-path was returning selector with RPL=0 instead of
     the client's CPL.  Harmless for DS-into-ring-3 loads (since
     DPL=3 satisfied MAX(CPL,RPL) <= DPL), but would crash on SS
     loads at ring-3.  Fixed.
   - PM exception dispatcher now detects recursive faults (same cs:eip
     5 times in a row) and CBRET_STOP-terminates, matching CWSDPMI's
     `locked_count > 5` bail.  Without this, DJGPP's SIGSEGV default
     handler would print + `_exit`, but `_exit` itself faults (same
     DS=0xF4 story, different call site at 0x1ACC), re-entering the
     handler, infinite spin.  Now: prints dump once, attempts exit,
     recursion guard fires, dosemu exits rc=0.

   **Outcome:** DJGPP djecho.exe prints its full SIGSEGV diagnostic:
   ```
   General Protection Fault at eip=00007c29
   eax=00000000 ebx=0009ffff ecx=000000ff edx=00000000 ...
   cs: sel=002f base=00120000 limit=0009ffff
   ds: sel=0037 base=00120000 limit=0009ffff
   ...
   Exiting due to signal SIGSEGV
   ```
   No actual program execution (djecho's main() never runs), but
   the DPMI infrastructure is now correct enough that DJGPP's
   runtime diagnostic path is fully functional.  Full program
   execution needs the 0xF4 mystery resolved -- likely involves
   reverse-engineering the stubinfo consumption in go32-v2.

3. **Paging.**  As before, genuine ring-3 isolation needs CR0.PG=1 +
   page tables so a misbehaving client can't overwrite host memory
   through a legitimate selector.  Multi-session.

2. **Secondary fault in exit path.**  After the handler prints the
   dump it calls `_exit(-1)` -> `__exit` -> INT 21h AH=4C.  Inside
   that, a second #GP fires at 0x002f:0x1acc with err=0 (null-selector
   access).  Same class of bug -- also mitigated by the recursion guard.

**Why this is hard to fix without paging:** DPMI clients legitimately
point their own selectors at arbitrary linear addresses.  With no
page tables, any such write lands at the corresponding physical
memory.  Without a mapping layer that can make our host memory
*inaccessible* to ring-3 clients, the client can always trash the
host.  Real DPMI hosts (CWSDPMI, HDPMI, WINOS2) all use paging.

**For next session:**
- **Add paging to our DPMI host.**  CWSDPMI's approach: every
  selector the client uses (their CS/DS/SS/etc. LDT entries) maps
  linearly onto their own allocated pages via a page table.  Writes
  through those selectors to "unowned" linear addresses (like our
  kernel) page-fault; the page fault handler catches it and either
  reports an error to the client or silently ignores.  Alternatively
  `#define run_ring 0` + drop ring-3 entirely, since ring-0 already
  works (23+ fixtures pass) — but that makes real DPMI programs
  (DJGPP, Windows 3.x, FoxPro) impossible.
- **Lower-leverage:** DPMI_DOSMEM fixtures cover AX=0100/0101 which
  DJGPP uses — all pass.  INT 10h reflection works in 16-bit and
  32-bit PM (both fixtures green).  Most of the surface is
  exercised; remaining gaps are mostly edge cases.

---

(Prior-session notes follow.)

# dosemu WIP — end of session 2026-04-21 (continued)

All work is committed and pushed to `github.com/avwohl/dosemu` (main).
No uncommitted changes to rescue. The `dosbox-staging` submodule always
shows "modified content" because the Makefile patches its
`src/gui/sdlmain.cpp` at build time from
`patches/sdlmain-expose-setup.patch`; `make distclean` resets it.

HEAD is at `48bc558` "LE loader: end-to-end execution -- LE_MIN
runs to exit".  First hand-crafted LE binary executes cleanly
through the full pipeline: load -> fixups -> descriptor install ->
PM entry -> 32-bit execution -> PM INT 21h AH=4Ch -> exit 0.

## Resume checklist (fresh machine)

```
git clone --recurse-submodules git@github.com:avwohl/dosemu.git
cd dosemu

sudo apt install -y build-essential cmake ninja-build meson \
    pkg-config libsdl2-dev libsdl2-net-dev libpng-dev \
    libopusfile-dev libspeexdsp-dev libfluidsynth-dev \
    libslirp-dev libasound2-dev libxi-dev libglib2.0-dev \
    patch nasm p7zip-full

# optional -- for building cross-compiled test fixtures:
sudo apt install -y bcc bin86 binutils-djgpp
sudo snap install --edge open-watcom

make                          # 5 min cold; seconds on incremental

build/dosemu tests/HELLO.COM               # prints dosemu-hello-ok
build/dosemu tests/DPMI_INTEGRATION.COM    # end-to-end 32-bit PM smoke test
```

CI runs every fixture on every push via `.github/workflows/ci.yml`.

## Memory state (Claude's)

At `~/.claude/projects/-home-wohl-src-dosemu/memory/`. Read first:

- `MEMORY.md` — index (always loaded into context)
- `architecture.md` — cpmemu-style: dosbox linked in-process, host C++
  implements INT 21h. No subprocess.
- `dpmi_plan.md` — DPMI stages + full spec coverage status.
- `dosbox_setstartup_seam.md` — how we override SHELL_Init without
  patching dosbox core.
- `feedback_no_subprocess.md` — rule: never fork/exec an emulator.
- `feedback_auto_commit_push.md` — wrap-up work → commit + push to
  origin/main without asking.

## DPMI 0.9 status — complete

Every INT 31h sub-function in the DPMI 0.9 spec is implemented or
stubbed. 23 DPMI fixtures in CI, all green.

| AX range          | Coverage                                                          |
|-------------------|-------------------------------------------------------------------|
| `0000-000C`       | Descriptor mgmt (LDT alloc/free/alias/raw get-set, limit, access) |
| `0100-0102`       | DOS memory alloc/free/resize (hybrid RM seg + PM selector)        |
| `0200/0201`       | Real-mode IVT get/set                                             |
| `0202/0203`       | PM exception handler get/set (dispatch **live** via IDT gate)     |
| `0204/0205`       | PM IDT gate get/set                                               |
| `0300/0301/0302`  | Simulate RM INT / call RM procedure (RETF or IRET frame)          |
| `0303/0304`       | Allocate/Free RM callback address (16-bit and 32-bit PM)          |
| `0305/0306`       | State save/restore + raw mode-switch addresses (stubs)            |
| `0400`            | Get DPMI version                                                  |
| `0500`            | Get free memory info                                              |
| `0501/0502/0503`  | Linear memory alloc/free/resize                                   |
| `0600/0601`       | Lock/unlock linear region (no-op; no paging)                      |
| `0602/0603`       | Mark RM region pageable/unpageable (no-op)                        |
| `0604`            | Get page size (4096)                                              |
| `0702/0703`       | Mark page as demand-paging / discard (no-op)                      |
| `0800/0801`       | Physical address mapping (pass-through; no remap)                 |
| `0900/0901/0902`  | Virtual IF state (get-and-{disable,enable,get})                   |
| `0B00-0B03`       | Debug watchpoints (stubs)                                         |

Plus:

- INT 2Fh/1687h detection advertising 32-bit capable.
- Real→PM switch (16-bit and 32-bit client entry).
- PM→RM interrupt reflection (16-bit and 32-bit; CB_IRETD shims
  per-vector in `PM_SHIM_SEG=0x1C00`).
- AX=0203 PM exception handlers actually fire — IDT gate installed
  alongside the sel:off table on set, dosbox's `CPU_Interrupt` path
  dispatches to client.

### DPMI landmines (documented for future maintenance)

1. **`CALLBACK_SCF` is RM-semantics only.** Our `set_cf` uses
   `SegPhys(ss)` + bitness-aware offset (SP+4 vs SP+8) based on
   `cpu.code.big`. Was silently wrong in PM because AH=09/4Ch don't
   propagate CF; stage-4 bad-selector checks were what surfaced it.
2. **`CB_INT21` stub's plain `CF` (16-bit IRET) can't unwind a 32-bit
   gate's 12-byte frame.** CB_IRETD variant with `66 CF` needed for
   32-bit PM. Same applies to every per-vector reflection shim.
3. **IDTR must be swapped to the RM IVT (`base=0, limit=0x3FF`) before
   `CR0.PE=0`.** On a 386, RM INT dispatch uses IDTR, not a fixed IVT.
   Our PM IDT at 0x1A000 holds 8-byte gates; reading them as 4-byte
   seg:off pairs lands the CPU in garbage. This is the *single*
   hardest-to-find bug in the whole mode-switch sequence.
4. **`CPU_SetSegGeneral(cs, ...)` in PM does not fully refresh the
   decoder.** Direct `Segs.val[cs]/Segs.phys[cs]` write + `cpu.code.big
   = desc.Big()` is safer. The next fetched byte (IRET in 030x, CB in
   RM-callback stubs) does a proper CS load via the kernel path.
5. **RM callback's epilogue must restore the stub's entry CS:EIP.**
   During the PM callback we change CS to the PM target and the stop
   callback; without restoring, the stub's trailing `CB` (RETF) never
   executes and the RM caller's return address is never popped.

## Process management (AH=4Bh / 4Ch / 4Dh)

- **AH=4B AL=0** (load + execute): parameterized loaders
  (`load_com_at`, `load_exe_at`, `load_program_at`); child gets 64KB
  from `mcb_allocate`, fresh PSP with env-block **copied** (not
  aliased) so child mutations don't corrupt parent.
- **AH=4B AL=1** (load without execute): SS:SP + CS:IP written to
  caller's parameter block output fields.
- **AH=4B AL=3** (load overlay): loads at caller-specified segment
  with no PSP / no execution.
- **AH=4C**: nested-aware. Records exit code in top of
  `s_process_stack` and returns `CBRET_STOP` (unwinds nested
  `DOSBOX_RunMachine`); top-level exit sets `shutdown_requested`.
- **AH=4D**: reads `s_last_child_exit` populated by AH=4B on restore.
- 3-level chain verified: `GRAND.COM` → `MIDDLE.COM` → `CHILD.COM`.

## LE loader — end-to-end

Commits `dbbe111` + `b487526` + `98ce926` + `db1f0c6` + `cbefb92` +
`2db8310` + `48bc558`:

- MZ binaries get their `lfanew` (file offset 0x3C) checked for "LE"
  or "LX" signature.
- `load_le_inspect` dumps header + object table to stderr.
- `le_load_objects` walks the object table, allocates per object
  (MCB first, pm_arena >= 1MB fallback), copies "legal" pages
  (type 0) from `data_pages` into host memory. Last-page trim
  honored.
- `le_apply_fixups` walks `fixup_page_table` @ le_off+0x68 +
  `fixup_record_table` @ le_off+0x6C. Source types 0x05/0x07/0x08
  land fully as internal-reference patches against the post-load
  host linear address; types 0x02/0x03/0x06 are stubbed (no LDT
  descriptors installed for LE objects yet). Imports/entry-table
  targets log + abort.
- Hand-crafted `tests/LE_MIN.EXE` (387 bytes; regeneratable via
  `tests/gen_le_min.py`) exercises exactly one type-7 fixup. CI
  verifies the walker resolves the target to host_base[obj2] + 0.
- Verified end-to-end on Open Watcom's real `wd.exe` (710KB,
  3 objects, 13655 fixups): all objects allocated, all fixups
  resolved, no crashes. Objects 1+3 land in pm_arena above 1MB;
  obj 2 lands in the MCB arena.
- `le_install_descriptors` allocates a run of LDT slots and writes
  one descriptor per object with base/limit/access/D-bit derived
  from the object's flags.  Selector stashed in `LeObject.ldt_sel`.
- `pm_setup_gdt_and_idt` extracted as shared helper used by both
  `dosemu_dpmi_entry` and the new LE launch path.
- `le_launch_pm_prep` seeds GDT/IDT from RM; `dosemu_startup`
  gains an `is_pm` branch that flips CR0.PE, CPU_LLDT, loads
  DS/ES/SS from LDT selectors, CPU_JMP to entry CS:EIP, then
  DOSBOX_RunMachine.
- `LE_MIN.EXE` code rewritten to `mov eax, imm32; mov ah, 4Ch;
  int 21h`: the PM INT 21h handler picks up AL=0 and exits rc=0.
  CI asserts rc=0.

### Still missing for real LE binaries

| Piece | Sketch |
|---|---|
| Selector-bearing fixups (0x02/0x03/0x06) | `le_apply_fixups` currently writes 0 for the selector field of these fixup types. Now that descriptors are installed, replace the stub with `objects[tgt_obj-1].ldt_sel`. Unlocks 16:16 and 16:32 pointers which real clients use for vtables + function pointers. |
| DPMI service hand-off | Real DOS4G/W clients skip our built-in DPMI switch and do their own init. `wd.exe` currently aborts on an un-installed interrupt vector (`INT:Gate Selector points to illegal descriptor with type 0x0`) shortly after entry. Probably needs a RM-stub boot that sets up INT 21h-equivalent RM vectors the client can reflect through. |
| RM INT reflection for LE client | Our PM IDT reflection path covers vectors whose IVT[] points at CB_SEG. For LE clients we may need to reflect INT 21h/31h/2Fh/etc. unconditionally. |
| Import resolution | LE supports imports from other modules (imp-ord / imp-name reference types). `le_apply_fixups` logs + aborts on these. Real clients may need them; for Watcom utils like `wd.exe` imports are typically RTL-internal and live in the same image. |

Roughly another ~200-400 lines to reach "hello-world LE binary runs
end-to-end" from here.

## Available Open Watcom tools

Not in the repo. On this machine: `~/ow/`. Grab elsewhere:

```
mkdir -p ~/ow && cd ~/ow
curl -sL -o ow.exe \
  'https://github.com/open-watcom/open-watcom-v2/releases/download/Current-build/open-watcom-2_0-c-win-x86.exe'
7z x -y ow.exe 'binw/*' 'lib286/*' 'h/*'
```

Real-mode MZ tools that run in dosemu today:

	binw/owcc.exe       C compiler driver
	binw/exe2bin.exe    EXE→COM converter (in repo as tests/EXE2BIN.EXE)
	binw/cmdedit.exe
	binw/dos32a.exe     probes DPMI, runs silent without a client to load
	binw/edbind.exe
	binw/ms2wlink.exe

LE binaries the loader now detects but doesn't yet execute:

	binw/wd.exe         600KB+, exceeds MCB arena
	binw/wcl386.exe     similar
	binw/pmwsetup.exe   PM-stub variant

## Cross-compile flow

```
mkdir -p ~/dosemu-watcom-test && cd ~/dosemu-watcom-test
cat > hello.c <<'EOF'
#include <stdio.h>
int main(void) { printf("hello from watcom\n"); return 0; }
EOF
snap run open-watcom.owcc-dos    -o hello.exe   hello.c   # 16-bit RM
snap run open-watcom.owcc-dos4g  -o hello32.exe hello.c   # 32-bit DOS4G

~/src/dosemu/build/dosemu hello.exe      # works
~/src/dosemu/build/dosemu hello32.exe    # detects LE, can't run yet
```

## DOS32A status

Loads as MZ, completes real-mode init, does 32-bit PM switch
cleanly (no more "illegal descriptor type" abort). Without a
client binary to load it sits in its own wait loop. The remaining
gap for `DOS32A foo.exe` end-to-end is the LE loader above.

## QEMM parity investigation — structural gap found

Spent time tracing exactly why our DPMI doesn't satisfy DOS/4GW
(with `DOSEMU_FORCE_DPMI=1`) or DJGPP's go32 stub.  Found the
answer in CWSDPMI's open source (`/tmp/cwsdpmi` on dev machine):

**We run DPMI clients at ring 0.  Real DPMI hosts run them at
ring 3.**

CWSDPMI's `GDT.H` shows a 17-entry GDT including dual code/data
selectors (`rcode`/`rdata` ring 0 for the host, `pcode`/`pdata`
ring 3 for client), three TSS selectors (`atss`/`ctss`/`itss`)
for inter-ring transitions, and a dedicated `iret` selector.
`#define run_ring 3`.

Our GDT has 9 entries, no TSS, clients share ring 0 with us.
That's structurally wrong for any real DPMI client.

Concrete symptoms:
- DOS/4GW tries to load selector 0x180 (GDT index 48).  Our GDT
  ends at index 8.
- DJGPP gcc (via go32 stub) takes a PM exception at EIP=0x28b,
  very early in the stub's init -- probably before it even calls
  its first INT 31h.

Closing this requires:
1. Grow GDT to 17+ entries matching CWSDPMI's layout
2. Install a TSS for ring-0/ring-3 transitions
3. Run clients at CPL=3 (flip RPL bits in selectors we hand out
   via AX=0000, flip entry CS/DS/SS to ring 3)
4. Properly dispatch exceptions with inter-level stack switch

That's multi-session work.  Every DPMI test fixture we've written
(DPMI_STAGE* etc.) runs clients at ring 0 and would need updating
too.  Worth doing in a dedicated push once there's concrete
motivation (e.g. if CI needs to cover a specific real DPMI
binary).

For now the practical path (commit 4af6fe5) stands: auto-detect
bound extenders from MZ stub content, suppress DPMI
advertisement for them, let them use their own PM machinery.
`dosemu wcc386.exe hello.c` works with no flags; that's QEMM's
user-facing promise even if the under-the-hood mechanism is
different.

## Available local DPMI-host-using clients for regression anchor

Surveyed `~/ow/binw` (the Open Watcom install).  63 bare-MZ
binaries (16-bit, work today), 26 DOS/4GW-bound, 5 PMODE/W-bound,
2 DOS/16M-bound.  **Zero** binaries that would use an external
DPMI host — Watcom's toolchain is self-contained around
DOS/4GW.  DJGPP gcc binaries are available at
`/tmp/djgpp/bin` on the dev machine but they need a ring-3 DPMI
to run (same blocker as above).

Without a working DPMI client to test against, the ring-3
rewrite has no regression target.  Next tool would need to be
DJGPP itself (once the ring-3 work is done) or an OSS PM program
we can find that uses external DPMI.  Known candidates: early
Windows 3.x setup tools, FoxPro 2.6 in DPMI mode, but
availability + legality is unclear.

## pm_arena memory tier (new this session)

`pm_alloc` / `pm_free` / `pm_resize` claim the extended-memory region
[1MB, memsize) via a simple first-fit free list over linear bytes.
AX=0501 tries the MCB tier first (paras <= 0xFFFFh); on MCB-OOM or
size > 1MB it falls through to `pm_alloc`. Handle encoding
distinguishes the two tiers: SI=0 + DI=mcb_seg for MCB blocks, or
SI:DI = high:low(host linear base) for pm_arena blocks. AX=0502 and
AX=0503 dispatch on SI. Fixture `DPMI_PMALLOC.COM` asks for just
over 1MB and verifies the tier switch + handle encoding + ES-selector
round-trip + in-place shrink + free.

`pm_alloc` intentionally does not zero-fill: touching every byte via
`mem_writeb` in a multi-MB loop was observed to trip dosbox's IRET
bookkeeping on return from the 0501 gate (E_Exit: "IRET:Outer level:
Stack segment not writable"). Clients that want zeros clear the
block themselves. DPMI doesn't require it.

## Next-session pick list

Ordered roughly by leverage / difficulty:

1. **Selector-bearing fixups (0x02/0x03/0x06).** Small win. Wire
   `objects[tgt_obj-1].ldt_sel` into the selector field instead of
   writing 0. Should unblock any real LE binary that uses far
   function pointers or vtables.
2. **Watcom 32-bit binaries — completely different approach landed.**
   Progress ladder:
   - Exception handlers installed (2087074).
   - Gate bitness matches entry BIG (63f0ef3).
   - BIG-bit misread fixed (4e4def8): was 0x4000, spec is 0x2000.
   - set_cf frame-offset bug fixed (0b57f67): was corrupting CS's
     RPL to 1 on every CF-returning PM INT 21h/31h.
   - Extender identified as DOS/4GW (not PharLap -- the EBX="PHAR"
     value is Watcom's pre-set "input signature", the actual
     detection is AH=FFh DH=00 DL=78h per RBIL).
   - DOS/4GW detection stub added (c738cd9): returns
     EAX=0x4734FFFF for that probe.  Alone, insufficient.
   - **DOSEMU_LE_AS_MZ flag landed (5975c42)**: skip the LE
     loader and let the MZ stub run as-is.  For DOS/4GW-bound
     binaries like wcc386.exe, the MZ stub IS the DOS/4GW
     extender; running it means DOS/4GW does its own DPMI init
     via AX=1687h, loads the embedded LE image itself, and sets
     up the selector tables + transfer buffers the Watcom
     runtime expects.
   With DOSEMU_LE_AS_MZ=1 + `dos4gw.exe` in the workdir (the
   extender looks for it by name), wcc386.exe now runs through
   DOS/4GW's full init -- hundreds of INT 21h calls -- and
   emits a real diagnostic: "DOS/16M error: [13] cannot
   allocate transfer buffer".  That's a concrete, actionable
   error, not a crash.
   Next piece: debug the transfer-buffer allocation.  DOS/16M
   (used internally by DOS/4GW for 16-bit-protected-mode data
   shuttling) allocates the buffer via AH=48 in conventional
   memory.  Our MCB arena may be exhausted by DOS/4GW's
   prior resize calls, or AH=48's max-available report may be
   inaccurate.  Straightforward once instrumented.

   Confirmed pattern: vi.exe (OW's vi, also LE CPU=2) fails with
   the exact same AH=30h/AH=FFh/GP trace, confirming this is a
   shared Watcom C runtime bring-up path, not a wd.exe-specific
   bug.  Other ~/ow/binw binaries: pmodew.exe / pmwsetup.exe are
   RM MZ stubs that print `Error Loading EXE!` when run bare;
   wde.exe is a Windows 16-bit NE (not LE).
3. **Cross-build a DJGPP tiny hello** (separate toolchain). Might
   give us a COFF-in-MZ path that's easier than LE for some
   targets.
4. **AH=4B AL=5** (Set execution state).

## Full fixture inventory

Real-mode / non-DPMI:

	HELLO.COM       hand .COM: print + exit
	HELLO.EXE       hand MZ .EXE
	WRITE.COM       AH=3C/40/3E/4C file create/write/close
	CAT.COM         AH=3D/3F/40/3E read/echo
	SIZE.COM        AH=3D/42/3E seek-to-EOF / size
	LISTDIR.COM     AH=4E/4F find-first/next + 8.3 mangle
	ECHOIN.COM      AH=01 stdin + echo
	SYSCALLS.COM    AH=30/48/25/35
	ENVDUMP.COM     PSP:[2Ch] env walker
	MCB_TEST.COM    alloc-free-alloc-same MCB coalescing
	SURVIVE.COM     unimplemented AH soft-fail continues

LE fixtures:

	LE_MIN.EXE            hand-crafted 387-byte LE (gen_le_min.py);
	                      1 code obj + 1 data obj + one type-7 fixup
	                      CI verifies the walker resolves to host_base

DPMI:

	DPMI_PROBE.COM        INT 2Fh/1687h → "dpmi=present"
	DPMI_INT31.COM        INT 31h default denial (AX=FF00 unhandled)
	DPMI_STAGE3.COM       real→PM switch, spins (CI timeout = pass)
	DPMI_STAGE5.COM       INT 21h from 16-bit PM
	DPMI_STAGE5_32.COM    INT 21h from 32-bit PM (CB_IRETD path)
	DPMI_STAGE4.COM       AX=0400/0006/0007 from PM
	DPMI_STAGE4B.COM      LDT AX=0000/0001/0002/0003 + ES alias load
	DPMI_STAGE4C.COM      AX=0200/0201 IVT + AX=0500 memory info
	DPMI_STAGE6.COM       AX=0501 alloc + 0502 free + write/read
	DPMI_STAGE6B.COM      AX=0503 resize shrink+grow
	DPMI_REFLECT.COM      INT 10h AH=0F from 16-bit PM via IVT reflection
	DPMI_REFLECT32.COM    Same, 32-bit PM (shim path)
	DPMI_INTEGRATION.COM  32-bit PM end-to-end: detect+switch+alloc+print+free
	DPMI_SIMRM.COM        AX=0300 simulate RM INT (mode switch)
	DPMI_CALLRM.COM       AX=0301 call RM procedure (RETF)
	DPMI_CALLRMI.COM      AX=0302 call RM procedure (IRET frame)
	DPMI_DOSMEM.COM       AX=0100/0101 DOS memory alloc (RM seg + PM sel)
	DPMI_PMIDT.COM        AX=0204/0205 PM IDT + AX=0900/0901/0902 virtual IF
	DPMI_STUBS.COM        AX=0202/0203 exc handler + 0600/0601/0604/0800
	DPMI_DESCMGMT.COM     AX=0008/0009/000A/000B/000C descriptor mgmt
	DPMI_EXTRA.COM        AX=0102 resize + 0305/0306/0801/0B00 stubs
	DPMI_RMCB.COM         AX=0303/0304 RM callback (16-bit PM proc)
	DPMI_RMCB32.COM       AX=0303/0304 RM callback (32-bit PM proc)
	DPMI_EXC.COM          AX=0203 PM exception actually dispatches (#UD)
	DPMI_PMALLOC.COM      AX=0501 tier-2 pm_arena fallback (>1MB alloc)

Process management:

	SPAWN.COM + CHILD.COM            AH=4B AL=0 + AH=4C (single-level)
	GRAND + MIDDLE + CHILD           3-level AH=4B nesting + AH=4D
	SPAWN_AL1.COM                    AH=4B AL=1 returns CS:IP + SS:SP

Real compiler output:

	HELLO_W.EXE   Open Watcom 16-bit hello
	HELLO_B.COM   bcc 0.16.21 hello
	EXE2BIN.EXE   real Open Watcom DOS-hosted utility (prints usage banner)

External-tool integration:

	FreeDOS xcopy.exe   real-world file copy, wired from dosbox-staging/build

## Commits since the original handoff (1222c44)

```
0930290  INT 21h trace: show full 32-bit regs when called from PM
c484c5e  WIP.md: set_cf fix landed; wd.exe next gap identified
0b57f67  INT 21h/31h: fix set_cf frame offset for 32-bit gates
3666d20  WIP.md: wd.exe next-failure instrumented
d65a0c2  WIP.md: LE BIG bit fix + wd.exe next failure identified
4e4def8  LE loader: BIG bit is 0x2000, not 0x4000
6a8933a  WIP.md: bitness-matched exception gates landed
63f0ef3  LE: match exception-gate bitness to entry object's BIG flag
c84de5b  WIP.md: LE catch-all exception handler landed
2087074  LE: install catch-all exception handler for vectors 0x00..0x1F
b593fa4  WIP.md: selector-bearing fixups landed; wd.exe diagnosis
d21def3  LE loader: wire selector-bearing fixups + fix LE_MIN exit code
62cd044  WIP.md: LE loader end-to-end, first PM execution landed
48bc558  LE loader: end-to-end execution -- LE_MIN runs to exit
2db8310  DPMI: extract pm_setup_gdt_and_idt helper
06b7f4d  WIP.md: LE LDT descriptor install landed
cbefb92  LE loader: install one LDT descriptor per object
7d7c1b0  WIP.md: refresh for LE fixup walker + pm_arena landing
db1f0c6  LE loader: tier object allocation into pm_arena for >1MB binaries
2c63896  DPMI AX=0501: pm_arena tier above 1MB
98ce926  LE loader: apply internal-reference fixups from fixup_page_table
872271d  Convert WIP.txt -> WIP.md and refresh for end-of-session 2026-04-21
b487526  LE loader: page-copy into MCB-allocated host segments
dbbe111  Loader: recognize LE/LX format, report structure on load attempt
671c168  AH=4Bh: AL=1 (load without execute) + AL=3 (load overlay)
93234af  DPMI AX=0203: actually dispatch PM exceptions to the client's handler
61ecf7e  DPMI AX=0303: 32-bit PM callback support
695d9dc  DPMI AX=0303/0304: Allocate/Free Real Mode Callback Address
722ce63  DPMI: full descriptor + mem mgmt + stub set (spec sweep)
6d23533  DPMI: fix AX=020x numbering to match DPMI 0.9 spec
17cffd6  AH=4B: copy env block to fresh MCB for each child
694507c  INT 21h AH=4Dh + 3-level AH=4B nesting
98ff3cd  INT 21h AH=4Bh: Load and Execute Program (AL=0)
17405ce  DPMI stubs: AX=0202/0203 exceptions, 0600/0601 lock, 0604 page, 0800 phys
5fef99a  DPMI AX=0210/0212 + 0900/0901/0902: PM IDT get/set + virtual IF
d33cb2d  DPMI AX=0100/0101: DOS memory alloc/free (hybrid RM seg + PM selector)
a7ac848  DPMI AX=0302: Call Real Mode Procedure With IRET Frame
b38e7ee  DPMI AX=0301: Call Real Mode Procedure With Far Return
7c5e605  DPMI AX=0300: Simulate Real Mode Interrupt (full mode switch)
4d7081e  DPMI_INTEGRATION: end-to-end 32-bit PM smoke test
5cd3751  DPMI stage 6 complete: INT 31h AX=0503 resize memory block
173ee2d  DPMI: 32-bit PM→RM reflection via per-vector CB_IRETD shims
5680c93  DPMI: 16-bit PM→RM interrupt reflection via IVT walk
e2bc815  DPMI: advertise 32-bit support + AX=0204/0205/0500
e915de4  DPMI stage 4 (full): LDT descriptor alloc/free/convert
d7793b9  DPMI stage 6 (minimal): INT 31h AX=0501 alloc + AX=0502 free
ffcdbff  DPMI stage 4 (subset): INT 31h AX=0400 + get/set segment base
5fd7d67  Add CLAUDE.md: commit+push on summary, submodule-dirty is expected
bfe1c76  DPMI stage 5 (32-bit): end-to-end fixture + IRETD callback stub
```

58 commits from the session's start (`1222c44` "WIP.txt: handoff notes").
All on main, all pushed.

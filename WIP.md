# dosemu WIP — end of session 2026-04-21 (continued)

All work is committed and pushed to `github.com/avwohl/dosemu` (main).
No uncommitted changes to rescue. The `dosbox-staging` submodule always
shows "modified content" because the Makefile patches its
`src/gui/sdlmain.cpp` at build time from
`patches/sdlmain-expose-setup.patch`; `make distclean` resets it.

HEAD is at `db1f0c6` "LE loader: tier object allocation into pm_arena
for >1MB binaries".

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

## LE loader — more landed

Commits `dbbe111` + `b487526` + `98ce926` + `db1f0c6`:

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

### Still missing for LE execution

| Piece | Sketch |
|---|---|
| PM descriptor install per object | One LDT entry per object. Access byte from object flags: code(0x0004)→0x9A code-read, else 0x92 data-rw. D-bit from BIG flag (0x4000). |
| Entry point dispatch | Install descriptors, enter PM, set CS:EIP from `entry_obj`:`entry_eip` (LE header offset 0x18:0x1C), SS:ESP from `stack_obj`:`stack_esp` (0x20:0x24), DS from the automatic-data-object selector (LE header offset 0x94). Existing `dpmi_enter_pm_mode` has the GDT/IDT/CR0/LLDT plumbing; the LE path needs its own variant that swaps in per-object LDT descriptors first and jumps to a client-chosen CS:EIP instead of using the retf-to-caller trick. |
| Selector-bearing fixups (0x02/0x03/0x06) | Once descriptors are installed, replace the zero-selector stub with the LDT selector of the target object. |
| Test target | `LE_MIN.EXE` has working fixups but its "code" is `B8 imm32; INT 20h` -- INT 20h is RM terminate, not PM. Change to `B4 4C CD 21` (AH=4Ch INT 21h) and rely on the PM INT 21h handler once entry dispatch is up. |

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

1. **PM descriptor install + entry dispatch for LE.** With
   fixups + pm_arena backing working, this is now the last big
   piece. Plan: add `le_enter_pm` that installs an LDT descriptor
   per object (access byte from flags, D-bit from BIG, base =
   host_base, limit = virt_size - 1), loads GDT/LDT/IDT the same
   way `dpmi_enter_pm_mode` does today, flips CR0.PE=1, and jumps
   to `entry_obj`:`entry_eip` with SS:ESP and auto-data-obj DS set
   up. Then re-test on `LE_MIN.EXE` (after swapping its exit stub
   to AH=4Ch INT 21h) and see how far `wd.exe` gets.
2. **Selector-bearing fixups (0x02/0x03/0x06).** Once LDT
   descriptors exist per object, replace the stub patches in
   `le_apply_fixups` with real selectors.
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

30 commits from the session's start (`1222c44` "WIP.txt: handoff notes").
All on main, all pushed.

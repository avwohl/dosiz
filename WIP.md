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

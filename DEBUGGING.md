# Debugging dosemu

All of these are **env-gated** — they cost nothing in production.
Set them in the environment when you run `dosemu`, one or more at
a time; mix freely.

## Trace env vars

| Variable              | What it logs                                                           |
|-----------------------|------------------------------------------------------------------------|
| `DOSEMU_TRACE`        | Every INT 21h (and some INT 2Fh) call with AX/BX/CX/DX                |
| `DOSEMU_CPU_TRACE`    | Every instruction executed in 16-bit PM + 32-bit PM (with cs:eip, bytes, cpl, esp). Also forces `core=normal` so the trace sees every fetch. Cap with `DOSEMU_CPU_TRACE_LINES=<N>`; default 200 |
| `DOSEMU_DPMI_TRACE`   | One line per DPMI-entry far-call, showing client CS/DS/SS/ES/FS/GS/IP/SP and the bases we compute for the starter-set LDT slots |
| `DOSEMU_LDT_TRACE`    | Every `write_ldt_descriptor` call: slot, selector, base, limit, access, bits32 + client's cs:eip at the time |
| `DOSEMU_EXC_TRACE`    | Every PM exception dispatch (`dosemu_pm_exc_dispatch`): sequence#, vector, faulting CS:EIP, err, outer SS:ESP, eflags |
| `DOSEMU_SIMRM_TRACE`  | Every INT 31h AX=0300 call: target INT, simulated AX/CX/DX/DS/ES + PM DS/SS/ES bases at entry |
| `DOSEMU_WRITE_TRACE`  | Every INT 21h AH=40h call: fd, cx, ds (selector+base), dx, and first 40 bytes of the buffer |
| `DOSEMU_DPMI_RING3`   | **Not** a trace — a mode switch.  Enables ring-3 DPMI clients.  Required for DJGPP programs |
| `DOSEMU_LE_AS_MZ`     | Skip the LE loader and let the MZ stub run.  For DOS/4GW-bound binaries (Watcom C/C++) |
| `DOSEMU_FORCE_DPMI`   | Force DPMI advertisement even for bound extenders                      |
| `DOSEMU_NO_DPMI`      | Never advertise DPMI                                                   |
| `DOSEMU_PATH`         | Colon-separated search path for program lookup                         |

## Typical debugging recipes

**"DJGPP crashed, what vector, where?"**
```
DOSEMU_DPMI_RING3=1 DOSEMU_EXC_TRACE=1 build/dosemu foo.exe
```
Look at `[exc#0]` — that's the first real fault.  Subsequent
`[exc#N]` lines are usually the DJGPP signal-handler re-faulting
during its own cleanup.

**"Is the client actually executing my fix?"**
```
DOSEMU_DPMI_RING3=1 DOSEMU_CPU_TRACE=1 DOSEMU_CPU_TRACE_LINES=50 build/dosemu foo.exe
```
Cap small — the trace gets huge fast.  Grep for specific EIPs you
care about.

**"Who's calling INT 21h AH=XX?"**
```
DOSEMU_TRACE=1 build/dosemu foo.exe 2>&1 | grep 'AH=XX'
```
The trace also surfaces "unimplemented INT 21h AH=YYh" warnings
once per distinct AH — useful for mapping what a program needs
that we don't implement.

**"DPMI entry isn't setting up LDT right"**
```
DOSEMU_DPMI_RING3=1 DOSEMU_DPMI_TRACE=1 DOSEMU_LDT_TRACE=1 build/dosemu foo.exe
```
First DPMI_TRACE line shows client register state at entry;
LDT_TRACE lines show every descriptor write and who writes it.

## Submodule-level hooks

`dosbox-staging/include/paging.h` and
`dosbox-staging/src/hardware/memory.cpp` hold pinpoint memory
watchpoints (`djgpp_*_watch` extern helpers) that were used during
the 0xF4 investigation.  Most are removed in the current tree;
re-add them when you need to catch a specific linear-address write
and can't easily instrument the C++ side.  Pattern:

```cpp
// paging.h mem_write{b,w,d}_inline:
if (address == 0xTARGET_LIN) {
    extern void my_watch(...);
    my_watch(...);
}
// memory.cpp:
void my_watch(uint32_t val) {
    fprintf(stderr, "[my-watch] [0xTARGET_LIN] <- 0x%08x (cs:eip=%04x:%08x)\n",
            val, SegValue(cs), reg_eip);
}
```

Writes via the dynrec JIT core bypass `mem_write*_inline`, so
always combine with `DOSEMU_CPU_TRACE=1` which forces
`core=normal`.

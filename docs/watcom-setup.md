# Building Open Watcom DOS/4G programs under dosiz

`wcc386 hello.c && wlink hello` works end-to-end under dosiz -- you
can compile C programs to DOS/4G LE executables, link them, and run
the result.  A fresh Open Watcom v2 release needs exactly one config
file in place (`binw/wlink.lnk`) to satisfy its own `wlink`, and a
few environment variables on the host side.

## TL;DR

1. Download the Open Watcom v2 Windows-x86 release:
   <https://github.com/open-watcom/open-watcom-v2/releases/download/Current-build/open-watcom-2_0-c-win-x86.exe>
2. Extract with `7z x open-watcom-2_0-c-win-x86.exe 'binw/*' 'lib386/*' 'h/*'`
3. Drop [`patches/watcom-wlink.lnk`](../patches/watcom-wlink.lnk)
   into `<watcom>/binw/wlink.lnk`
4. Set `WATCOM=C:\`, `INCLUDE=C:\H`, `DOSIZ_PATH=C:\BINW` when invoking dosiz
5. Build from inside the `<watcom>` directory

    ```
    cd ~/ow
    cat > hello.c <<'EOF'
    #include <stdio.h>
    int main(void) { printf("hello-watcom-ok\n"); return 0; }
    EOF
    cat > link.cmd <<'EOF'
    system dos4g
    file hello
    name hello.exe
    EOF
    WATCOM='C:\' INCLUDE='C:\H' DOSIZ_PATH='C:\BINW' \
      dosiz binw/wcc386.exe hello.c
    WATCOM='C:\' DOSIZ_PATH='C:\BINW' \
      dosiz binw/wlink.exe @link.cmd
    DOSIZ_PATH='C:\BINW' dosiz hello.exe
    ```

    Output: `hello-watcom-ok`.

## What goes wrong out of the box

The current open-watcom-v2 release does NOT include a working
`binw/wlink.lnk`.  Its `binw/specs.owc` drives the `owcc` compiler
driver, which synthesizes its own `wlink` directives based on a
hardcoded `-bt=dos` arch setting -- but without a `wlink.lnk`
specifying the `system dos4g` definition, `wlink`'s default
behavior produces a malformed LE that `dos4gw.exe` then rejects
with:

    DOS/4GW fatal error (1012): FOO.EXE is not a WATCOM program

The fix is exactly the small `wlink.lnk` in
[`../patches/watcom-wlink.lnk`](../patches/watcom-wlink.lnk).  It
mirrors the `system dos4g` definition from the 11.0c
`wlsystem.lnk` (minimal -- just `format os2 le`, `option osname`,
library paths, and the wstub.exe launcher).  With it in place,
`wlink` emits a DOS/4G LE with the right entry chain and dos4gw
accepts it.

## Why this is a dosiz doc, not a bug report

The issue reproduces when running `wcl386`/`wlink` natively on
Windows against the same release.  It's an Open Watcom
distribution issue, not a dosiz emulation issue.  Confirming that
the release needs this config file is itself useful information
for anyone trying to build DOS/4G binaries with the fresh release.

## Environment variables

    WATCOM         The Watcom install root, mapped to the DOS drive.
                   For the "run from ~/ow" pattern above this is "C:\".
    INCLUDE        Header search path.  `C:\H` for Watcom's h/ dir.
    DOSIZ_PATH    dosiz-only extension to the DOS PATH.
                   Point at BINW so wstub.exe can find dos4gw.exe.

These pass through from the host environment to the dosiz-simulated
DOS env block (implemented in `src/bridge.cc:build_env_block`).

## What works

- `wcc386 hello.c` -> valid OMF object file
- `wlink` -> 20KB DOS/4G LE executable
- running the LE executable through wstub + dos4gw

Verified against `printf`, `atoi`, multi-digit arithmetic,
`argc/argv` access.  Complex stdio + math + argv programs build
and run to completion.

## 16-bit DOS (tiny/small/large model) also works

Same pipeline, different tools:

    cd ~/ow
    cat > link16.cmd <<'EOF'
    system dos
    file hello
    name hello.exe
    EOF
    WATCOM='C:\' INCLUDE='C:\H' DOSIZ_PATH='C:\BINW' \
      dosiz binw/wcc.exe -ml hello.c
    WATCOM='C:\' DOSIZ_PATH='C:\BINW' \
      dosiz binw/wlink.exe @link16.cmd
    dosiz hello.exe

The 16-bit path needs the `lib286/` subdirectory (ships with the
same `open-watcom-2_0-c-win-x86.exe` installer; extract with
`7z x ... 'lib286/*'`).  The CRT init sequence calls `AH=63` (Get
DBCS lead-byte table) early during `_cstart_` -- dosiz returns a
properly-terminated `[0,0,0,0]` table at linear `0x0900` so the
walk succeeds and setlocale continues.  Before that fix the startup
walked into the IVT interpreting random interrupt vectors as DBCS
lead-byte ranges and silently aborted.

Verified: printf, atoi, argc/argv, multi-digit arithmetic through
the 16-bit wcc + system dos pipeline all work end-to-end.

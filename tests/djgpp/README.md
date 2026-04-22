# DJGPP regression test suite

End-to-end smoke tests that prove dosemu's ring-3 DPMI host is
correct enough to run DJGPP-compiled programs.

## What's here

- `dj_*.c` — short DJGPP C programs exercising one feature each
  (direct `write()`, `printf`, `argv`, `getenv`, file I/O,
  `malloc`, piped stdin).  Each prints `dj-<name>=ok` on success.
- `build.sh` — rebuilds every `.c` into the corresponding
  `tests/DJ_*.EXE` fixture.  Requires `i586-pc-msdosdjgpp-gcc` on
  `PATH`; the [`andrewwutw/build-djgpp`][bd] v3.4 bundle works
  on macOS and Linux.
- `run.sh` — runs every committed `DJ_*` fixture + a couple of
  regression-specific assertions (quoted args, real third-party
  DJGPP `banner.exe`).  Returns nonzero if any test failed.

[bd]: https://github.com/andrewwutw/build-djgpp/releases

The compiled `.EXE` binaries live in `tests/` (one level up) so
CI can run them without a DJGPP toolchain.  Rebuild and commit
them when a test source changes.

## Running locally

```
make                    # build dosemu if not already
./tests/djgpp/run.sh
```

## Adding a test

1. Drop `dj_foo.c` in this directory.
2. Have `main()` print a distinctive line like `dj-foo=ok\n` at
   successful exit.
3. `./build.sh` to produce `tests/DJ_FOO.EXE`.
4. Add `run_one DJ_FOO "dj-foo=ok" 0 ""` (expected rc, stdin,
   then any argv tail) in `run.sh`.
5. Commit the `.c`, the `.EXE`, and the `run.sh` edit together.

## What the suite covers

| Fixture     | What it exercises                                    |
|-------------|------------------------------------------------------|
| DJ_WRITE    | `write(1, ...)` → `_write_int` → `dosmemput` → AH=40 |
| DJ_PRINTF   | stdio buffering, %d/%s/%.f/%lX, exit-code propagation |
| DJ_ARGV     | argc/argv from DOS PSP cmd tail                      |
| DJ_ENV      | `getenv` of COMSPEC and PATH                         |
| DJ_FILE     | fopen/fprintf/fclose/fgets (LFN-fallback → AH=3C)    |
| DJ_MALLOC   | malloc/free stress (64 blocks × varying sizes)       |
| DJ_STDIN    | fread from piped stdin                               |
| DJ_SIGNAL   | SIGFPE handler + siglongjmp recovery (#DE → POSIX)   |
| DJ_QUOTED   | quoted multi-word args survive PSP cmd-tail encoding |
| BANNER      | real third-party DJGPP tool (banner.exe, 2005)       |
| GREP        | real GNU grep 2.28 (grep228b from delorie.com)       |
| DIFF        | real GNU diff (dif37b from delorie.com) — text-mode read regression gate |
| CAT         | real GNU cat (txt20b) — std-handle lseek/fstat regression gate |
| SED         | real GNU sed 4.8 (sed48b) — regex substitution             |
| SORT        | real GNU sort (txt20b) — buffered I/O + qsort              |

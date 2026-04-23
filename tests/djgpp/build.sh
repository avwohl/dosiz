#!/bin/bash
# build.sh -- compile every tests/djgpp/dj_*.c into tests/DJ_*.EXE
#
# Requires a DJGPP cross-compiler on PATH (i586-pc-msdosdjgpp-gcc).
# The andrewwutw/build-djgpp v3.4 macOS + Linux bundles work; so
# does a manually-built cross-gcc.  The produced COFF-go32-exe
# binaries are OS-independent DOS executables, so CI doesn't need
# a DJGPP toolchain -- it runs the committed binaries.
#
# Re-run this script after editing any tests/djgpp/dj_*.c source,
# and commit the updated .EXE files.
#
# Output binaries go to tests/DJ_<UPPERNAME>.EXE (DOS 8.3 convention).

set -euo pipefail

if ! command -v i586-pc-msdosdjgpp-gcc >/dev/null; then
    echo "error: i586-pc-msdosdjgpp-gcc not on PATH" >&2
    echo "  install from https://github.com/andrewwutw/build-djgpp" >&2
    exit 1
fi

cd "$(dirname "$0")"
out_dir="../"

for src in dj_*.c; do
    base="${src%.c}"                        # dj_write
    upper=$(echo "$base" | tr '[:lower:]' '[:upper:]')   # DJ_WRITE
    out="${out_dir}${upper}.EXE"
    i586-pc-msdosdjgpp-gcc -O2 -Wall -Wextra -o "$out" "$src"
    printf "  built  %s  (%d bytes)\n" "$out" "$(wc -c <"$out" | tr -d ' ')"
done

# 2.3 MB C++ binary: STL + regex + global ctor/dtor.  Kept separate
# from the dj_*.c set because it needs g++ and the C++ runtime, and
# takes noticeably longer to compile.  -O0 on purpose so the binary
# doesn't shrink below the 2 MB mark that probes our MCB allocator's
# large-alloc path.
if command -v i586-pc-msdosdjgpp-g++ >/dev/null; then
    i586-pc-msdosdjgpp-g++ -O0 -o "${out_dir}BIGTEST.EXE" bigtest.cpp
    printf "  built  %sBIGTEST.EXE  (%d bytes)\n" "$out_dir" "$(wc -c <"${out_dir}BIGTEST.EXE" | tr -d ' ')"
fi

#!/bin/bash
# run.sh -- run every DJGPP test fixture and verify its marker.
#
# Each test program prints "dj-<name>=ok" on success.  This harness
# invokes each from the repo root (so C: = repo root and the DJGPP
# stub can find its own .exe at C:\tests\DJ_*.EXE), captures stdout,
# strips DOS CR bytes, and greps for the expected marker.
#
# Specific invocations:
#   DJ_ARGV: passed the args "hello world" and additionally validates
#            argc=3 and argv[1]=hello in the output.
#   DJ_STDIN: fed "pipe-payload\n" on stdin; validates the program
#             echoed it back.
#   DJ_PRINTF: expects exit code 7 (the program returns 7 to verify
#              exit-code propagation).
#
# Run `./tests/djgpp/run.sh` from the repo root.  Prints one line per
# test and exits 0 iff every test passed.

set -eu
cd "$(dirname "$0")/../.."

if [[ ! -x build/dosemu ]]; then
    echo "error: build/dosemu not found; run 'make' first" >&2
    exit 1
fi

rm -f tests/djfile.tmp

export DOSEMU_DPMI_RING3=1

pass=0
fail=0
run_one() {
    local name="$1"; shift
    local marker="$1"; shift
    local expect_rc="$1"; shift
    local input="$1"; shift
    # remaining args are program args
    local exe=$(ls tests/${name}.* 2>/dev/null | head -1)
    if [[ -z "$exe" ]]; then
        printf "  %-12s MISSING (no binary)\n" "$name"
        fail=$((fail + 1))
        return
    fi
    local out rc
    if [[ -n "$input" ]]; then
        out=$(printf '%s' "$input" | ./build/dosemu "$exe" "$@" 2>/dev/null) && rc=$? || rc=$?
    else
        out=$(./build/dosemu "$exe" "$@" 2>/dev/null) && rc=$? || rc=$?
    fi
    local got_marker=$(echo "$out" | tr -d '\r' | grep -F "$marker" || true)
    if [[ "$rc" == "$expect_rc" && -n "$got_marker" ]]; then
        printf "  %-12s PASS\n" "$name"
        pass=$((pass + 1))
    else
        printf "  %-12s FAIL (rc=%s expect=%s marker=%s)\n" \
            "$name" "$rc" "$expect_rc" "${got_marker:-missing}"
        fail=$((fail + 1))
    fi
}

run_one DJ_WRITE  "dj-write=ok"  0 ""
run_one DJ_PRINTF "dj-printf=ok" 7 ""
run_one DJ_ARGV   "dj-argv=ok"   0 ""   hello world
run_one DJ_ENV    "dj-env=ok"    0 ""
run_one DJ_FILE   "dj-file=ok"   0 ""
run_one DJ_MALLOC "dj-malloc=ok" 0 ""
run_one DJ_STDIN  "dj-stdin=ok"  0 "pipe-payload"
run_one DJ_SIGNAL "dj-signal=ok" 0 ""

# Extra assertion: DJ_ARGV called with a quoted multi-word arg must
# deliver it as a single argv entry (regression gate for the PSP
# cmd-tail quoting fix).
qout=$(./build/dosemu tests/DJ_ARGV.exe "a b c" simple 2>/dev/null | tr -d '\r')
if echo "$qout" | grep -q "^\[1\]=a b c$" && echo "$qout" | grep -q "^argc=3$"; then
    printf "  %-12s PASS\n" "DJ_QUOTED"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL\n" "DJ_QUOTED"
    echo "output was:"
    echo "$qout" | sed 's/^/    /'
    fail=$((fail + 1))
fi

# Real third-party DJGPP tool: banner.exe (bann10b from the
# delorie.com archive, built 2005).  Rendering "HI" should produce
# ASCII art containing 7 rows of '#' characters.  This is a "no
# regression on external tools" gate -- if an aggressive change
# to dosemu's DPMI host breaks a real program, this catches it.
bout=$(./build/dosemu tests/BANNER.EXE HI 2>/dev/null | tr -d '\r')
if echo "$bout" | grep -q '#######' && [[ $(echo "$bout" | grep -c '#') -ge 7 ]]; then
    printf "  %-12s PASS\n" "BANNER"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (expected ASCII-art H and I with #s)\n" "BANNER"
    fail=$((fail + 1))
fi

# Another real DJGPP tool: grep (grep228b from delorie.com).  Needs
# AH=52 (fake sysvars), AH=57 (file date/time), AH=60 (canonicalize),
# AH=65 (country info), and AH=3E-of-stdout-as-no-op to run.  Feed a
# 4-line fixture and verify it matches the lines we expect, then
# verify the no-match case returns rc=1 (grep convention).
#
# Both dosemu and grep.exe are copied into a tmpdir so cwd-based
# drive mount lands there and grep's self-open via argv[0] finds
# grep.exe as C:\GREP.EXE.
gdir=$(mktemp -d)
cp build/dosemu "$gdir/"
cp tests/GREP.EXE "$gdir/"
printf "alpha\nbeta\nalphabet\ngamma\n" > "$gdir/input.txt"
(cd "$gdir" && DOSEMU_DPMI_RING3=1 ./dosemu GREP.EXE alpha input.txt 2>/dev/null) > "$gdir/out" && rc=$? || rc=$?
got=$(tr -d '\r' < "$gdir/out")
(cd "$gdir" && DOSEMU_DPMI_RING3=1 ./dosemu GREP.EXE nomatch input.txt 2>/dev/null) > /dev/null && rc2=$? || rc2=$?
rm -rf "$gdir"
if [[ "$rc" == "0" && "$got" == *"alpha"$'\n'"alphabet"* && "$rc2" == "1" ]]; then
    printf "  %-12s PASS\n" "GREP"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (match-rc=%s no-match-rc=%s out=%q)\n" \
        "GREP" "$rc" "$rc2" "$got"
    fail=$((fail + 1))
fi

# Another real DJGPP tool: diff (dif37b from delorie.com).  This is
# the regression gate for the AH=3F text-mode fix: diff reads the
# whole file, then does lseek(-filesize, SEEK_CUR) to rewind.  If
# AH=3F inflates byte counts with \n->\r\n for regular files, that
# seek fails with EINVAL and diff bails.  Fixture: two 3-line files
# differing on line 2; expected output is "2c2" diff format.
ddir=$(mktemp -d)
cp build/dosemu "$ddir/"
cp tests/DIFF.EXE "$ddir/"
printf "line1\nline2\nline3\n" > "$ddir/a.txt"
printf "line1\nCHANGED\nline3\n" > "$ddir/b.txt"
(cd "$ddir" && DOSEMU_DPMI_RING3=1 ./dosemu DIFF.EXE a.txt b.txt 2>/dev/null) > "$ddir/out" && drc=$? || drc=$?
dgot=$(tr -d '\r' < "$ddir/out")
rm -rf "$ddir"
if [[ "$drc" == "1" && "$dgot" == *"2c2"* && "$dgot" == *"< line2"* && "$dgot" == *"> CHANGED"* ]]; then
    printf "  %-12s PASS\n" "DIFF"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s out=%q)\n" "DIFF" "$drc" "$dgot"
    fail=$((fail + 1))
fi

# GNU cat (coreutils / txt20b).  cat probes stdout with fstat +
# lseek(fd=1, 0, SEEK_CUR) to classify it; the fix of the day was
# handling std-handle lseek gracefully (return pos=0 on ESPIPE
# rather than EBADF).  Regression gate for that.
cdir=$(mktemp -d)
cp build/dosemu tests/CAT.EXE "$cdir/"
printf "banana\napple\ncherry\n" > "$cdir/in.txt"
(cd "$cdir" && DOSEMU_DPMI_RING3=1 ./dosemu CAT.EXE in.txt 2>/dev/null) > "$cdir/out" && crc=$? || crc=$?
cgot=$(tr -d '\r' < "$cdir/out")
rm -rf "$cdir"
if [[ "$crc" == "0" && "$cgot" == "banana"$'\n'"apple"$'\n'"cherry" ]]; then
    printf "  %-12s PASS\n" "CAT"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s out=%q)\n" "CAT" "$crc" "$cgot"
    fail=$((fail + 1))
fi

# GNU sed 4.8 (sed48b).  's/X/Y/' substitution -- exercises the
# same std-handle + disk-read paths as cat but with regex work.
sdir=$(mktemp -d)
cp build/dosemu tests/SED.EXE "$sdir/"
printf "hello world\nfoo bar\nhello again\n" > "$sdir/in.txt"
(cd "$sdir" && DOSEMU_DPMI_RING3=1 ./dosemu SED.EXE 's/hello/HOWDY/' in.txt 2>/dev/null) > "$sdir/out" && src=$? || src=$?
sgot=$(tr -d '\r' < "$sdir/out")
rm -rf "$sdir"
if [[ "$src" == "0" && "$sgot" == *"HOWDY world"* && "$sgot" == *"HOWDY again"* ]]; then
    printf "  %-12s PASS\n" "SED"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s out=%q)\n" "SED" "$src" "$sgot"
    fail=$((fail + 1))
fi

# GNU sort (coreutils / txt20b).  Alphabetic sort -- verifies
# buffered I/O + qsort work end-to-end.
odir=$(mktemp -d)
cp build/dosemu tests/SORT.EXE "$odir/"
printf "banana\napple\ncherry\nbanana\n" > "$odir/in.txt"
(cd "$odir" && DOSEMU_DPMI_RING3=1 ./dosemu SORT.EXE in.txt 2>/dev/null) > "$odir/out" && orc=$? || orc=$?
ogot=$(tr -d '\r' < "$odir/out")
rm -rf "$odir"
if [[ "$orc" == "0" && "$ogot" == "apple"$'\n'"banana"$'\n'"banana"$'\n'"cherry" ]]; then
    printf "  %-12s PASS\n" "SORT"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s out=%q)\n" "SORT" "$orc" "$ogot"
    fail=$((fail + 1))
fi

echo ""
echo "  ${pass} passed, ${fail} failed"
exit "$fail"

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

if [[ ! -x build/dosiz ]]; then
    echo "error: build/dosiz not found; run 'make' first" >&2
    exit 1
fi

rm -f djfile.tmp

# Note: ring-3 DPMI is now the default.  `DOSIZ_DPMI_RING0=1` is
# available as an opt-out for the in-tree ring-0 DPMI fixtures.

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
        out=$(printf '%s' "$input" | ./build/dosiz "$exe" "$@" 2>/dev/null) && rc=$? || rc=$?
    else
        out=$(./build/dosiz "$exe" "$@" 2>/dev/null) && rc=$? || rc=$?
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

# EMS (LIM 4.0) and VCPI probes -- INT 67h + AX=DE00 respectively.
# Regression gate that dosbox-staging's ems.cpp stays wired through
# our dosbox-init path.
run_one EMS_PROBE  "ems-ok"     0 ""
run_one VCPI_PROBE "vcpi-ok"    0 ""
run_one HMA_PROBE  "hma-ok"     0 ""

run_one DJ_WRITE  "dj-write=ok"  0 ""
run_one DJ_PRINTF "dj-printf=ok" 7 ""
run_one DJ_ARGV   "dj-argv=ok"   0 ""   hello world
run_one DJ_ENV    "dj-env=ok"    0 ""
run_one DJ_FILE   "dj-file=ok"   0 ""
run_one DJ_MALLOC "dj-malloc=ok" 0 ""
run_one DJ_STDIN  "dj-stdin=ok"  0 "pipe-payload"
run_one DJ_SIGNAL "dj-signal=ok" 0 ""
run_one DJ_EXEC   "dj-exec=ok"   0 ""
run_one DJ_DJE    "dj-dj-exec=ok" 0 ""
run_one BIGTEST   "bigtest-done" 0 ""

# GNU make 4.4 + FreeCOM as $SHELL.  Build a tiny Makefile and check
# the rule's output arrives on stdout and the target file is created.
# This exercises: DJGPP stat() (LFN AL=4E fallback to ENOENT), exec
# via AH=4B of FreeCOM, FreeCOM's parsing of /c argument, FreeCOM
# running a built-in and exiting cleanly back to make.
mdir=$(mktemp -d)
cp build/dosiz tests/MAKE.EXE tests/COMMAND.COM "$mdir/"
mkdir "$mdir/TMP"
cat > "$mdir/Makefile" <<'MAKEEOF'
all: out.txt
out.txt:
	@echo make-hello > out.txt
	@echo built
MAKEEOF
(cd "$mdir" && TMPDIR=C:\\TMP timeout 20 ./dosiz MAKE.EXE 2>/dev/null) > "$mdir/out"
mout=$(tr -d '\r' < "$mdir/out")
mfile=$(tr -d '\r' < "$mdir/out.txt" 2>/dev/null)
rm -rf "$mdir"
if [[ "$mout" == *"built"* && "$mfile" == "make-hello" ]]; then
    printf "  %-12s PASS\n" "MAKE"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (stdout=%q target=%q)\n" "MAKE" "$mout" "$mfile"
    fail=$((fail + 1))
fi

# FreeCOM (FreeDOS's COMMAND.COM) smoke test: boot the shell, run a
# built-in (ECHO) that requires the AH=65 AL=05 NLS filename-character
# table (without it, is_fnchar() rejects CR/LF and the internal-command
# match fails on any command-with-arg), and exit.  Regression gate for
# the content-sniffed .COM/.EXE loader and the NLS fix.  We run from
# the repo root (not a tmpdir) so FreeCOM can self-locate via
# %COMSPEC%=C:\TESTS\COMMAND.COM for its strings resource.
fcout=$(printf 'echo fc-hello\r\nexit\r\n' | timeout 10 ./build/dosiz tests/COMMAND.COM 2>/dev/null | tr -d '\r')
if echo "$fcout" | grep -q '^fc-hello$'; then
    printf "  %-12s PASS\n" "FREECOM"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (out=%q)\n" "FREECOM" "$fcout"
    fail=$((fail + 1))
fi

# FreeCOM spawning an external program and returning to the REPL.
# Regression gate for two related fixes:
#   (a) top-level MCB-at-PSP-1  (FreeCOM's XMS_Swap build reads
#       mcb->mcb_size to derive SwapTransientSize; without a real
#       MCB, it reads 0 and AH=48 BX=0 post-spawn short-circuits).
#   (b) cpu.code.big + IDT restore on AH=4B exit when a PM child
#       (DJGPP) was spawned from an RM parent (FreeCOM).  Without
#       these, the callback trampoline's `iret` decodes as 32-bit
#       IRETD, or RM INT dispatch reads from the child's stale PM
#       IDT, and the REPL crashes into garbage.
# Exercises both an RM child (HELLO.COM) and a PM child (DJGPP
# DJ_WRITE.EXE) within a single FreeCOM session, then a post-spawn
# builtin + exit to verify the REPL is still alive.
# DJ_DJE hardcodes the path C:\TESTS\DJ_WRITE.EXE, so put DJ_WRITE.EXE
# under a TESTS\ subdir as well as at root (where FreeCOM can find it
# directly for the DJ_WRITE.exe command).
fcs_dir=$(mktemp -d)
mkdir -p "$fcs_dir/TESTS"
cp build/dosiz tests/COMMAND.COM tests/HELLO.COM tests/DJ_WRITE.exe \
   tests/SEQ.EXE tests/DJ_DJE.exe "$fcs_dir/"
cp tests/DJ_WRITE.exe "$fcs_dir/TESTS/"
fcsout=$(printf 'HELLO.COM\r\nDJ_WRITE.exe\r\nSEQ.EXE 1 3\r\nDJ_DJE.exe\r\necho post-spawn-ok\r\nexit\r\n' \
    | (cd "$fcs_dir" && timeout 15 ./dosiz COMMAND.COM 2>/dev/null) | tr -d '\r')
rm -rf "$fcs_dir"
# FC_SPAWN now asserts:
#   RM child (HELLO.COM)
#   minimal PM child (DJ_WRITE.EXE)
#   2000-era libc PM child (SEQ.EXE 1 3 -- prints "1" and "3")
#   3-level nested chain FreeCOM -> DJ_DJE -> DJ_WRITE
#   post-spawn builtin still works
if echo "$fcsout" | grep -q 'post-spawn-ok' \
    && echo "$fcsout" | grep -q 'dosiz-hello-ok' \
    && echo "$fcsout" | grep -q 'dj-write=ok' \
    && echo "$fcsout" | grep -q 'dj-dj-exec=ok' \
    && echo "$fcsout" | grep -Eq '^1$' \
    && echo "$fcsout" | grep -Eq '^3$'; then
    printf "  %-12s PASS\n" "FC_SPAWN"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (out=%q)\n" "FC_SPAWN" "$fcsout"
    fail=$((fail + 1))
fi

# Extra assertion: DJ_ARGV called with a quoted multi-word arg must
# deliver it as a single argv entry (regression gate for the PSP
# cmd-tail quoting fix).
qout=$(./build/dosiz tests/DJ_ARGV.exe "a b c" simple 2>/dev/null | tr -d '\r')
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
# to dosiz's DPMI host breaks a real program, this catches it.
bout=$(./build/dosiz tests/BANNER.EXE HI 2>/dev/null | tr -d '\r')
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
# Both dosiz and grep.exe are copied into a tmpdir so cwd-based
# drive mount lands there and grep's self-open via argv[0] finds
# grep.exe as C:\GREP.EXE.
gdir=$(mktemp -d)
cp build/dosiz "$gdir/"
cp tests/GREP.EXE "$gdir/"
printf "alpha\nbeta\nalphabet\ngamma\n" > "$gdir/input.txt"
(cd "$gdir" && ./dosiz GREP.EXE alpha input.txt 2>/dev/null) > "$gdir/out" && rc=$? || rc=$?
got=$(tr -d '\r' < "$gdir/out")
(cd "$gdir" && ./dosiz GREP.EXE nomatch input.txt 2>/dev/null) > /dev/null && rc2=$? || rc2=$?
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
cp build/dosiz "$ddir/"
cp tests/DIFF.EXE "$ddir/"
printf "line1\nline2\nline3\n" > "$ddir/a.txt"
printf "line1\nCHANGED\nline3\n" > "$ddir/b.txt"
(cd "$ddir" && ./dosiz DIFF.EXE a.txt b.txt 2>/dev/null) > "$ddir/out" && drc=$? || drc=$?
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
cp build/dosiz tests/CAT.EXE "$cdir/"
printf "banana\napple\ncherry\n" > "$cdir/in.txt"
(cd "$cdir" && ./dosiz CAT.EXE in.txt 2>/dev/null) > "$cdir/out" && crc=$? || crc=$?
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
cp build/dosiz tests/SED.EXE "$sdir/"
printf "hello world\nfoo bar\nhello again\n" > "$sdir/in.txt"
(cd "$sdir" && ./dosiz SED.EXE 's/hello/HOWDY/' in.txt 2>/dev/null) > "$sdir/out" && src=$? || src=$?
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
cp build/dosiz tests/SORT.EXE "$odir/"
printf "banana\napple\ncherry\nbanana\n" > "$odir/in.txt"
(cd "$odir" && ./dosiz SORT.EXE in.txt 2>/dev/null) > "$odir/out" && orc=$? || orc=$?
ogot=$(tr -d '\r' < "$odir/out")
rm -rf "$odir"
if [[ "$orc" == "0" && "$ogot" == "apple"$'\n'"banana"$'\n'"banana"$'\n'"cherry" ]]; then
    printf "  %-12s PASS\n" "SORT"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s out=%q)\n" "SORT" "$orc" "$ogot"
    fail=$((fail + 1))
fi

# GNU wc (coreutils / txt20b).  `wc -l input.txt` on a 4-line file.
wdir=$(mktemp -d)
cp build/dosiz tests/WC.EXE "$wdir/"
printf "one\ntwo\nthree\nfour\n" > "$wdir/in.txt"
(cd "$wdir" && ./dosiz WC.EXE -l in.txt 2>/dev/null) > "$wdir/out" && wrc=$? || wrc=$?
wgot=$(tr -d '\r' < "$wdir/out" | tr -s ' ')
rm -rf "$wdir"
# wc output: "       4 in.txt" (leading whitespace compressed by tr -s)
if [[ "$wrc" == "0" && "$wgot" == *"4 in.txt"* ]]; then
    printf "  %-12s PASS\n" "WC"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s out=%q)\n" "WC" "$wrc" "$wgot"
    fail=$((fail + 1))
fi

# GNU gawk 5.0 (gwk500b).  Prints column 2 of a whitespace-
# separated input -- exercises the regex engine + field split.
adir=$(mktemp -d)
cp build/dosiz tests/GAWK.EXE "$adir/"
printf "1 2 3\n4 5 6\n" > "$adir/in.txt"
(cd "$adir" && ./dosiz GAWK.EXE '{print $2}' in.txt 2>/dev/null) > "$adir/out" && arc=$? || arc=$?
agot=$(tr -d '\r' < "$adir/out")
rm -rf "$adir"
if [[ "$arc" == "0" && "$agot" == "2"$'\n'"5" ]]; then
    printf "  %-12s PASS\n" "GAWK"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s out=%q)\n" "GAWK" "$arc" "$agot"
    fail=$((fail + 1))
fi

# GNU gzip 1.10 (gzip110b).  Compress then decompress the same
# data and verify round-trip -- gate for binary-safe stdio and
# that the compressor's deterministic state machine survives
# the AH=3F/AH=40 paths without spurious CR/LF insertion.  Also
# verify host gunzip (if available) can decompress our gzip's
# output -- catches TZ/format-compat bugs that would round-trip
# within dosiz but produce non-standard files.
zdir=$(mktemp -d)
cp build/dosiz tests/GZIP.EXE "$zdir/"
printf "alpha beta gamma delta epsilon alpha beta gamma delta epsilon\n" > "$zdir/in.txt"
orig=$(tr -d '\r' < "$zdir/in.txt")
(cd "$zdir" && ./dosiz GZIP.EXE -c in.txt 2>/dev/null) > "$zdir/in.gz" && zrc=$? || zrc=$?
(cd "$zdir" && ./dosiz GZIP.EXE -dc in.gz 2>/dev/null) > "$zdir/out" && zrc2=$? || zrc2=$?
roundtrip=$(tr -d '\r' < "$zdir/out")
host_unzip_ok=1
if command -v gunzip >/dev/null; then
    host_out=$(gunzip -kc "$zdir/in.gz" 2>/dev/null | tr -d '\r' || true)
    [[ "$host_out" == "$orig" ]] || host_unzip_ok=0
fi
rm -rf "$zdir"
if [[ "$zrc" == "0" && "$zrc2" == "0" && "$roundtrip" == "$orig" && "$host_unzip_ok" == 1 ]]; then
    printf "  %-12s PASS\n" "GZIP"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (zip-rc=%s unzip-rc=%s host-ok=%s out=%q)\n" \
        "GZIP" "$zrc" "$zrc2" "$host_unzip_ok" "$roundtrip"
    fail=$((fail + 1))
fi

# GNU ls (fileutils fil41b).  Exercises AH=4E/4F findfirst/findnext
# directory enumeration -- the full dance including ".", "..", and
# attribute-byte for directory detection.  Regression gate for the
# token-based find-state + DOS-semantic glob fixes.
ldir=$(mktemp -d)
cp build/dosiz tests/LS.EXE "$ldir/"
echo "content-a" > "$ldir/a.txt"
echo "content-b" > "$ldir/b.txt"
mkdir "$ldir/subdir"
(cd "$ldir" && ./dosiz LS.EXE 2>/dev/null) > "$ldir/out" && lrc=$? || lrc=$?
lgot=$(tr -d '\r' < "$ldir/out")
rm -rf "$ldir"
# ls should list a.txt, b.txt, subdir, dosiz, ls.exe (case may vary).
if [[ "$lrc" == "0" \
    && "$lgot" == *"a.txt"* \
    && "$lgot" == *"b.txt"* \
    && "$lgot" == *"subdir"* ]]; then
    printf "  %-12s PASS\n" "LS"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s out=%q)\n" "LS" "$lrc" "$lgot"
    fail=$((fail + 1))
fi

# GNU find (fnd4233b).  Full recursive descent is DJGPP-port quirky
# so we test a single-file stat -- that exercises the findfirst
# on an exact filename (no-wildcard pattern), which is the usual
# POSIX stat() path on DOS.
fdir=$(mktemp -d)
cp build/dosiz tests/FIND.EXE "$fdir/"
echo hi > "$fdir/target.txt"
(cd "$fdir" && ./dosiz FIND.EXE target.txt 2>/dev/null) > "$fdir/out" && frc=$? || frc=$?
fgot=$(tr -d '\r' < "$fdir/out")
rm -rf "$fdir"
if [[ "$frc" == "0" && "$fgot" == *"target.txt"* ]]; then
    printf "  %-12s PASS\n" "FIND"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s out=%q)\n" "FIND" "$frc" "$fgot"
    fail=$((fail + 1))
fi

# GNU patch (pat275b).  Applies a unified diff -- pairs with DIFF
# to exercise the same text-file read/write path in both directions.
pdir=$(mktemp -d)
cp build/dosiz tests/PATCH.EXE "$pdir/"
printf "hello\nworld\n" > "$pdir/a.txt"
printf "2c2\n< world\n---\n> WORLD\n" > "$pdir/p.diff"
(cd "$pdir" && ./dosiz PATCH.EXE a.txt p.diff 2>/dev/null) >/dev/null && prc=$? || prc=$?
pfinal=$(tr -d '\r' < "$pdir/a.txt")
rm -rf "$pdir"
if [[ "$prc" == "0" && "$pfinal" == "hello"$'\n'"WORLD" ]]; then
    printf "  %-12s PASS\n" "PATCH"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s final=%q)\n" "PATCH" "$prc" "$pfinal"
    fail=$((fail + 1))
fi

# GNU tar (tar112ab).  Full round-trip: create archive with two
# files, extract into a fresh dir, verify content matches.  Tests
# multi-file I/O and the archive format on AH=3F/40.
tdir=$(mktemp -d)
cp build/dosiz tests/TAR.EXE "$tdir/"
echo "file1-content" > "$tdir/a.txt"
echo "file2-content" > "$tdir/b.txt"
(cd "$tdir" && ./dosiz TAR.EXE cf out.tar a.txt b.txt 2>/dev/null) >/dev/null && trc1=$? || trc1=$?
mkdir "$tdir/ex"
cp build/dosiz tests/TAR.EXE "$tdir/out.tar" "$tdir/ex/"
(cd "$tdir/ex" && ./dosiz TAR.EXE xf out.tar 2>/dev/null) >/dev/null && trc2=$? || trc2=$?
tgot1=$(tr -d '\r' < "$tdir/ex/a.txt" 2>/dev/null)
tgot2=$(tr -d '\r' < "$tdir/ex/b.txt" 2>/dev/null)
rm -rf "$tdir"
if [[ "$trc1" == "0" && "$trc2" == "0" \
    && "$tgot1" == "file1-content" && "$tgot2" == "file2-content" ]]; then
    printf "  %-12s PASS\n" "TAR"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (cf-rc=%s xf-rc=%s a=%q b=%q)\n" \
        "TAR" "$trc1" "$trc2" "$tgot1" "$tgot2"
    fail=$((fail + 1))
fi

# GNU bc (bc1071b).  Feed expressions on stdin -- tests interactive-
# style piped stdin reads to a program that does its own line
# parsing, not just stdio buffered reads.
bcdir=$(mktemp -d)
cp build/dosiz tests/BC.EXE "$bcdir/"
bcout=$(printf "2+2\n7*6\nquit\n" | (cd "$bcdir" && ./dosiz BC.EXE 2>/dev/null))
bcrc=$?
rm -rf "$bcdir"
bcout=$(echo "$bcout" | tr -d '\r')
if [[ "$bcrc" == "0" && "$bcout" == "4"$'\n'"42" ]]; then
    printf "  %-12s PASS\n" "BC"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s out=%q)\n" "BC" "$bcrc" "$bcout"
    fail=$((fail + 1))
fi

# GNU m4 1.4.19 (m4-1419b).  Macro processor -- expands a simple
# define.  Different dispatch from sed (which uses regex); m4 does
# parser-level text substitution with its own symbol table.
mdir=$(mktemp -d)
cp build/dosiz tests/M4.EXE "$mdir/"
printf "define(\`GREET', \`Hi, \$1!')GREET(\`world')\n" > "$mdir/in.m4"
(cd "$mdir" && ./dosiz M4.EXE in.m4 2>/dev/null) > "$mdir/out" && mrc=$? || mrc=$?
mgot=$(tr -d '\r' < "$mdir/out")
rm -rf "$mdir"
if [[ "$mrc" == "0" && "$mgot" == "Hi, world!" ]]; then
    printf "  %-12s PASS\n" "M4"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s out=%q)\n" "M4" "$mrc" "$mgot"
    fail=$((fail + 1))
fi

# GNU flex 2.5.4 (flx254b).  Generates a lexical scanner -- exercises
# AH=46h (force-dup, a.k.a. dup2) because flex dup2's its output
# file onto stdout before writing the generated C.  Regression gate
# for the dup/dup2 fix.
xdir=$(mktemp -d)
cp build/dosiz tests/FLEX.EXE "$xdir/"
printf '%%%%\n[a-z]+ printf("word: %%s\\n", yytext);\n' > "$xdir/in.l"
(cd "$xdir" && ./dosiz FLEX.EXE in.l 2>/dev/null) >/dev/null && xrc=$? || xrc=$?
xlines=$(wc -l < "$xdir/lexyy.c" 2>/dev/null || echo 0)
xhas_yylex=$(grep -c yylex "$xdir/lexyy.c" 2>/dev/null || echo 0)
rm -rf "$xdir"
if [[ "$xrc" == "0" && "$xlines" -gt 100 && "$xhas_yylex" -gt 0 ]]; then
    printf "  %-12s PASS\n" "FLEX"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (rc=%s lines=%s yylex=%s)\n" \
        "FLEX" "$xrc" "$xlines" "$xhas_yylex"
    fail=$((fail + 1))
fi

# GNU seq (shellutils).  Generates integer sequence to stdout --
# tests argv-as-numbers + AH=40 stdout writes.
seqout=$(./build/dosiz tests/SEQ.EXE 1 5 2>/dev/null | tr -d '\r' | tr '\n' ',')
if [[ "$seqout" == "1,2,3,4,5," ]]; then
    printf "  %-12s PASS\n" "SEQ"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (out=%q)\n" "SEQ" "$seqout"
    fail=$((fail + 1))
fi

# GNU factor (shellutils).  Prime factorization of 42 = 2*3*7.
# Exercises argv parsing, simple arithmetic, and formatted output.
fout=$(./build/dosiz tests/FACTOR.EXE 42 2>/dev/null | tr -d '\r')
if [[ "$fout" == "42: 2 3 7" ]]; then
    printf "  %-12s PASS\n" "FACTOR"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (out=%q)\n" "FACTOR" "$fout"
    fail=$((fail + 1))
fi

# GNU basename (shellutils).  Strips directory from a DOS path.
bnout=$(./build/dosiz tests/BASENAME.EXE "C:\\TESTS\\DJ_WRITE.EXE" 2>/dev/null | tr -d '\r')
if [[ "$bnout" == "DJ_WRITE.EXE" ]]; then
    printf "  %-12s PASS\n" "BASENAME"
    pass=$((pass + 1))
else
    printf "  %-12s FAIL (out=%q)\n" "BASENAME" "$bnout"
    fail=$((fail + 1))
fi

# Open Watcom compile-link-run pipeline (optional -- requires a
# complete OW install at ~/ow with patches/watcom-wlink.lnk dropped
# into ~/ow/binw/wlink.lnk).  See docs/watcom-setup.md.  When the
# install isn't present this test is skipped silently.
if [[ -f ~/ow/binw/wcc386.exe && -f ~/ow/binw/wlink.exe \
      && -f ~/ow/binw/wstub.exe && -f ~/ow/binw/dos4gw.exe ]]; then
    # Make sure wlink.lnk exists (drop ours if the install doesn't have one)
    if [[ ! -f ~/ow/binw/wlink.lnk ]]; then
        cp patches/watcom-wlink.lnk ~/ow/binw/wlink.lnk
    fi
    watdir=$(mktemp -d)
    cp -R ~/ow/binw ~/ow/h ~/ow/lib386 ~/ow/lib286 "$watdir/" 2>/dev/null
    cp build/dosiz "$watdir/"
    cat > "$watdir/hello.c" <<'HELLOCEOF'
#include <stdio.h>
int main(void) { printf("wat-compile-link-run-ok\n"); return 0; }
HELLOCEOF

    # 32-bit DOS/4G pipeline
    cat > "$watdir/link32.cmd" <<'LINK32EOF'
system dos4g
file hello
name hello.exe
LINK32EOF
    watout=$(cd "$watdir" && \
        WATCOM='C:\' INCLUDE='C:\H' DOSIZ_PATH='C:\BINW' \
        ./dosiz binw/wcc386.exe hello.c 2>/dev/null && \
        WATCOM='C:\' DOSIZ_PATH='C:\BINW' \
        ./dosiz binw/wlink.exe @link32.cmd 2>/dev/null && \
        DOSIZ_PATH='C:\BINW' ./dosiz hello.exe 2>/dev/null | tr -d '\r')
    if echo "$watout" | grep -q 'wat-compile-link-run-ok'; then
        printf "  %-12s PASS\n" "WATCOM32"
        pass=$((pass + 1))
    else
        printf "  %-12s FAIL (out=%q)\n" "WATCOM32" "$watout"
        fail=$((fail + 1))
    fi

    # 16-bit DOS pipeline (if lib286 is installed)
    if [[ -d "$watdir/lib286" ]]; then
        rm -f "$watdir/hello.obj" "$watdir/hello.exe"
        cat > "$watdir/link16.cmd" <<'LINK16EOF'
system dos
file hello
name hello.exe
LINK16EOF
        wat16out=$(cd "$watdir" && \
            WATCOM='C:\' INCLUDE='C:\H' DOSIZ_PATH='C:\BINW' \
            ./dosiz binw/wcc.exe -ml hello.c 2>/dev/null && \
            WATCOM='C:\' DOSIZ_PATH='C:\BINW' \
            ./dosiz binw/wlink.exe @link16.cmd 2>/dev/null && \
            ./dosiz hello.exe 2>/dev/null | tr -d '\r')
        if echo "$wat16out" | grep -q 'wat-compile-link-run-ok'; then
            printf "  %-12s PASS\n" "WATCOM16"
            pass=$((pass + 1))
        else
            printf "  %-12s FAIL (out=%q)\n" "WATCOM16" "$wat16out"
            fail=$((fail + 1))
        fi
    fi
    rm -rf "$watdir"
fi

echo ""
echo "  ${pass} passed, ${fail} failed"
exit "$fail"

#!/usr/bin/env bash
#
# tests/run_tests.sh — smoke tests for dosemu
#
# Intentionally minimal: CLI parsing, config file loading, dosbox.conf
# generation. Tests that actually run DOS programs require dosbox-staging
# to have been built (ninja -C dosbox-staging/build) and live in `live_*`.
#

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
DOSEMU="${DOSEMU:-$ROOT/build/dosemu}"

if [[ ! -x "$DOSEMU" ]]; then
  echo "FAIL: $DOSEMU not found — build it first (make, or cmake --build build)" >&2
  exit 1
fi

pass=0
fail=0
note() { printf '  %s\n' "$*"; }
pass() { printf '  \033[32mPASS\033[0m  %s\n' "$*"; pass=$((pass+1)); }
fail() { printf '  \033[31mFAIL\033[0m  %s\n' "$*"; fail=$((fail+1)); }

printf '=== CLI smoke tests ===\n'

if "$DOSEMU" --version | grep -q 'dosemu '; then
  pass '--version prints dosemu banner'
else
  fail '--version'
fi

if "$DOSEMU" --help 2>&1 | grep -q 'config.cfg'; then
  pass '--help mentions config.cfg'
else
  fail '--help'
fi

if { "$DOSEMU" --no-such-flag 2>&1 || true; } | grep -q 'unknown option'; then
  pass 'unknown option is rejected'
else
  fail 'unknown option'
fi

printf '\n=== Conf generation tests ===\n'

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"

# Stubbed dosbox: just prints the conf path and exits.  dosemu --keep-conf
# preserves the conf in /tmp so we can inspect it.
#
# Use /bin/false so the stubbed "dosbox" returns non-zero, which exercises
# the exit-code propagation path. --keep-conf keeps /tmp/dosemu-*.conf.
rm -f /tmp/dosemu-*.conf
"$DOSEMU" --keep-conf --dosbox=/bin/false HELLO.EXE foo bar 2>/dev/null || true
conf=$(ls -t /tmp/dosemu-*.conf 2>/dev/null | head -1)

if [[ -f "$conf" ]] && grep -q "HELLO.EXE foo bar" "$conf"; then
  pass 'autoexec runs HELLO.EXE with args'
else
  fail 'autoexec program line'
fi

if [[ -f "$conf" ]] && grep -q "MOUNT C \"$WORK\"" "$conf"; then
  pass 'CWD mounted as C:'
else
  fail 'CWD mount'
fi

if [[ -f "$conf" ]] && grep -q '^memsize = 16' "$conf"; then
  pass 'default memsize=16'
else
  fail 'memsize'
fi

rm -f /tmp/dosemu-*.conf

# .cfg file loading
cat > prog.cfg <<EOF
program = FOO.EXE
args = /q /v
drive_C = $WORK
drive_D = /tmp
memsize = 32
cputype = 486
EOF

"$DOSEMU" --keep-conf --dosbox=/bin/false prog.cfg 2>/dev/null || true
conf=$(ls -t /tmp/dosemu-*.conf 2>/dev/null | head -1)

if [[ -f "$conf" ]] && grep -q "FOO.EXE /q /v" "$conf"; then
  pass '.cfg program line with args'
else
  fail '.cfg program line'
fi

if [[ -f "$conf" ]] && grep -q "MOUNT D \"/tmp\"" "$conf"; then
  pass '.cfg mounts D:'
else
  fail '.cfg drive_D'
fi

if [[ -f "$conf" ]] && grep -q '^memsize = 32' "$conf"; then
  pass '.cfg overrides memsize'
else
  fail '.cfg memsize'
fi

if [[ -f "$conf" ]] && grep -q '^cputype = 486' "$conf"; then
  pass '.cfg overrides cputype'
else
  fail '.cfg cputype'
fi

rm -f /tmp/dosemu-*.conf

# Env var expansion in .cfg
cat > env.cfg <<EOF
program = TEST.EXE
drive_C = \${HOME}/nonexistent
EOF

"$DOSEMU" --keep-conf --dosbox=/bin/false env.cfg 2>/dev/null || true
conf=$(ls -t /tmp/dosemu-*.conf 2>/dev/null | head -1)

if [[ -f "$conf" ]] && grep -q "MOUNT C \"$HOME/nonexistent\"" "$conf"; then
  pass 'env var expansion in cfg values'
else
  fail 'env var expansion'
fi

rm -f /tmp/dosemu-*.conf

printf '\n=== Integration tests (require dosbox-staging built) ===\n'

DOSBOX="${DOSBOX:-$ROOT/dosbox-staging/build/dosbox}"
if [[ ! -x "$DOSBOX" ]]; then
  note "SKIP  dosbox-staging not built ($DOSBOX)"
else
  # Run ECHO (a COMMAND.COM builtin) and capture its stdout.
  mkdir -p "$WORK/dosrun"
  cd "$WORK/dosrun"
  cat > go.cfg <<EOF
program = ECHO
args = dosemu-live-test-ok
drive_C = $WORK/dosrun
EOF

  out=$("$DOSEMU" --dosbox="$DOSBOX" go.cfg 2>&1 || true)
  if grep -q 'dosemu-live-test-ok' <<< "$out"; then
    pass 'dosbox runs ECHO and returns output'
  else
    fail "dosbox ECHO test (got: $out)"
  fi
fi

printf '\n=== Results ===\n'
printf '  %d passed, %d failed\n' "$pass" "$fail"
exit $((fail > 0))

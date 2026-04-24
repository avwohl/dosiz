# Why does `cpp.exe` crash under dosiz?

A write-up of the failure mode, root cause, and what can be done about
it.  Moved from inline WIP notes so the conclusions are easy to find
later.

## TL;DR

**It's a DJGPP libc bug, not a dosiz bug.**  Any program linked
against DJGPP libc 2.05 has a stack-buffer overflow in its startup
code.  Whether the overflow is *visible* depends on the program's
startup call chain.  `delorie/gcc*b.zip/bin/cpp.exe` happens to have
exactly the wrong call chain and crashes; our own DJGPP-built
binaries (up to and including the 2.3 MB `tests/BIGTEST.EXE` with
STL + regex + global constructors) survive the same overflow and run
clean.

The patch that fixes it is in
[`../patches/djgpp-libc-c1loadef-stack-smash.patch`](../patches/djgpp-libc-c1loadef-stack-smash.patch).
It can't be PR'd because DJGPP libc hasn't been actively maintained
since October 2015 -- see [*Distribution options*](#distribution-options)
below.

## Symptom

Running `delorie`'s `cpp.exe --version` with the standard 123-line
`djgpp.env` from `djdev205.zip`:

```
$ DJGPP='C:/DJGPP/DJGPP.ENV' ./dosiz CPP.EXE --version
Exiting due to signal SIGILL
Exiting due to signal SIGFPE
Exiting due to signal SIGFPE
Exiting due to signal SIGFPE
Exiting due to signal SIGFPE
dosiz: PM exception dispatcher in recursive-fault loop
        (vec=16 cs:eip=0037:001e713d err=0x0) -- terminating
```

Same failure on both delorie-built gcc 9.3 cpp.exe and gcc 12.2
cpp.exe.  Trimming `djgpp.env` below **2683 bytes** dodges it; above
that size, any content fails.  With a minimal two-line `djgpp.env`
(just `DJDIR=...`), `cpp.exe --version` prints its banner cleanly.

## Root cause

DJGPP libc source file `src/libc/crt0/c1loadef.c`, function
`__crt0_load_environment_file`:

```c
if (this_prog)
{
  char *buf = alloca(fsize);        /* <<< stack buffer sized to file size */
  char *tb2 = buf;
  char *sp = tb, *dp = tb2;
  ...
  /* %VAR% expansion loop: */
  e = getenv(sp + 1);
  if (e)
    while (*e)
      *dp++ = *e++;                 /* <<< no bound on dp */
  ...
}
```

The stack buffer is sized to the **raw** djgpp.env file size.  Every
`%VAR%` reference then expands into that buffer with no upper bound.
Standard `djgpp.env` lines recursively reference already-expanded
variables, e.g.:

```
C_INCLUDE_PATH=%/>;C_INCLUDE_PATH%%DJDIR%/include
```

Each pass through the loop grows `C_INCLUDE_PATH` longer.  After a
few lines the expanded output walks past `buf + fsize` and scribbles
over whatever's above `buf` in the caller's stack frame.

Under the normal delorie `cpp.exe` startup call chain, "whatever's
above `buf`" happens to include a pushed return address.  On return,
the CPU pops four bytes of ASCII djgpp.env content (`;%DJ` = bytes
`3B 25 44 4A`, the `;%DJ` prefix of `;%DJDIR%`) into EIP, jumps to
`0x4a44253b` -- a linear address well past the CS segment limit --
and traps #UD.  DJGPP's SIGILL handler is then re-entered in a
state where our dispatcher sees it re-faulting at the same EIP and
bails after the recursion counter trips.

## Evidence this is not dosiz

| What                                        | Runs with full djgpp.env? |
|---------------------------------------------|---------------------------|
| `tests/HELLO.EXE` (DJGPP, 150 KB)           | ✓                         |
| `tests/DJ_WRITE.exe` + 9 other DJ_* (DJGPP) | ✓ (32/33 suite)           |
| `tests/BIGTEST.EXE` (DJGPP C++, 2.3 MB)     | ✓ (STL/regex/globals)     |
| Our envtest renamed to `CPP.EXE`            | ✓                         |
| `delorie/gcc930b/bin/cpp.exe` (2.2 MB)      | ✗ (same failure)          |
| `delorie/gcc122b/bin/cpp.exe` (2.2 MB)      | ✗ (same failure)          |

Both delorie cpp.exe binaries fail at the *exact same* 2683-byte
threshold, with *byte-identical* bogus EIP content.  Different GCC
versions, same startup code (DJGPP libc), same bug.  Our own
DJGPP-built programs use the same libc and overflow the same buffer
-- but their call chains at startup don't put a return address at
the offset the overflow reaches.  See `patches/README.md` for the
bisected EIP evidence.

## The fix

`patches/djgpp-libc-c1loadef-stack-smash.patch` replaces the unbounded
`alloca(fsize)` with a `malloc`'d buffer that realloc-grows on demand.
Every write goes through an `ENSURE(1)` gate that doubles the buffer
when the next write would overrun:

```c
-    char *buf = alloca(fsize);
-    char *tb2 = buf;
-    char *sp=tb, *dp=tb2;
-    while (*sp != '=' && *sp != '\n' && *sp)
-      *dp++ = *sp++;
+    size_t buf_sz = (size_t)fsize + 32;
+    char *buf = (char *)malloc(buf_sz);
+    if (!buf) continue;
+    size_t dp_off = 0, dirend_off = 0, tb2_off = 0;
+    char *sp = tb;
+
+#define C1LOADEF_ENSURE(need) do { \
+      if (dp_off + (need) > buf_sz) { \
+        size_t ns = buf_sz * 2; \
+        while (ns < dp_off + (need) + 16) ns *= 2; \
+        char *nb = (char *)realloc(buf, ns); \
+        if (!nb) { free(buf); goto c1loadef_bail; } \
+        buf = nb; buf_sz = ns; \
+      } \
+    } while (0)
+#define C1LOADEF_EMIT(c) do { \
+      C1LOADEF_ENSURE(1); \
+      buf[dp_off++] = (char)(c); \
+    } while (0)
+
+    while (*sp != '=' && *sp != '\n' && *sp)
+      C1LOADEF_EMIT(*sp++);
     ...
+    putenv(buf + tb2_off);
+    free(buf);
```

No magic size multiplier, no "generous headroom" guess -- the buffer
grows by doubling only when expansion actually overruns it.

Heap allocation is fine because DJGPP's `putenv`
(`libc/compat/stdlib/putenv.c`) copies the value rather than retaining
the caller's pointer, so freeing the buffer after `putenv()` is safe.
This is DJGPP-specific non-POSIX behavior but it's how DJGPP has
always worked.

### Verified

`patches/verify-c1loadef-patch.sh` pulls `djlsr205.zip`, compiles a
test harness that pre-populates `C_INCLUDE_PATH` with 40 KB of data
and invokes `__crt0_load_environment_file`.  On un-patched 2.05:
SIGBUS (rc=138).  Patched: emits the expected 40 977-byte expansion
cleanly (rc=0).

`patch --dry-run -p1` applies cleanly to `djlsr205.zip`'s source
tree.

## Distribution options

DJGPP libc hasn't seen a release since `djlsr205.zip` was posted to
delorie.com on 2015-10-18.  There's no git repository, no GitHub
mirror, no issue tracker, and no PR queue.  The traditional
delorie.com mail archive shows "Updated Jul 2019", so even the
mailing list is near-dormant.

Realistic channels to get the patch into circulation:

### 1. `djgpp@delorie.com` mailing list

Subscribe via <https://www.delorie.com/djgpp/>.  DJ Delorie still
nominally watches it.  Traditional and "correct" but given the
decade of dormancy, expected throughput is low.

Message would need:
- A clear subject (e.g. `[PATCH] libc: fix __crt0_load_environment_file stack overflow`)
- A reproducer (djgpp.env excerpt + "cpp.exe --version" under DOSBox suffices; dosemu2 is a fine second)
- The patch inline (not an attachment)

### 2. Community fork on GitHub

Precedent: [`PC-98/djlsr`](https://github.com/PC-98/djlsr) forked
djlsr in 2020 for PC-98-specific hacking.  Nothing stops us doing
the same.  A clean `djlsr205-community/` or `djgpp-libc-fixes/`
repo with this one patch applied, versioned as `2.05.1`, would
give downstream distributors (andrewwutw/build-djgpp, FreeDOS
packagers, etc.) a target to pull from.

Pros: visible, discoverable, git-native.
Cons: risks fragmenting an already-thin ecosystem.  Better to pitch
it to DJ Delorie first and only fork if that goes silent for N weeks.

### 3. Inline in downstream DJGPP distributions

[`andrewwutw/build-djgpp`](https://github.com/andrewwutw/build-djgpp)
is actively maintained (last push 2024-08).  It could carry the
patch in its `patch/` directory and apply during build.  Same story
for any FreeDOS packager shipping a DJGPP set.  This is probably the
fastest path to "users actually get the fix" even if upstream never
accepts it.

### 4. Leave as-is in `patches/` here

What we're doing now.  The patch, the reproducer, and this write-up
are all in-tree so anyone debugging a similar crash in another DPMI
host finds the root cause via a search.  Not a distribution channel
for the fix itself -- just a bread-crumb trail.

## Recommended order

1. Post to `djgpp@delorie.com` with the patch and reproducer.  If DJ
   Delorie responds with a release cadence or merge plan, stop there.
2. If no response in ~4 weeks, open a PR on
   `andrewwutw/build-djgpp` adding the patch to its patch set.  That
   immediately benefits everyone building DJGPP from that repo.
3. Optionally (or if Andrew Wu declines for out-of-scope reasons)
   publish a `djgpp-libc-patches` fork on GitHub with this + any
   other accumulated fixes, and link it from the dosiz README.

## References

- [`../WIP.md`](../WIP.md) -- investigation log (search for
  "The delorie cpp.exe SIGFPE")
- [`../patches/djgpp-libc-c1loadef-stack-smash.patch`](../patches/djgpp-libc-c1loadef-stack-smash.patch) -- the patch itself
- [`../patches/README.md`](../patches/README.md) -- patch archive
  index
- [`../tests/djgpp/bigtest.cpp`](../tests/djgpp/bigtest.cpp) +
  [`../tests/BIGTEST.exe`](../tests/BIGTEST.exe) -- 2.3 MB DJGPP C++
  regression gate that proves dosiz isn't the problem
- `src/libc/crt0/c1loadef.c` in `djlsr205.zip` (download from
  <https://www.delorie.com/pub/djgpp/current/v2/djlsr205.zip>) --
  the buggy source

/* dj_dje.c -- DJGPP parent spawns a DJGPP child (nested DPMI).
 *
 * Harder than dj_exec.c: the child enters protected mode, installs
 * its own PM exception handlers, wipes the LDT, switches CR0.PE,
 * allocates DOS memory, then exits.  The bridge must snapshot and
 * restore enough parent state (LDT, exception handlers, CR0,
 * segment selectors, FS/GS) that the parent can resume in its own
 * libc post-spawn.
 *
 * Prints "dj-dj-exec=ok" on success.
 */
#include <stdio.h>
#include <stdlib.h>
#include <process.h>
#include <errno.h>

int main(void) {
    int rc = spawnlp(P_WAIT, "C:\\TESTS\\DJ_WRITE.EXE",
                     "DJ_WRITE.EXE", (char *)NULL);
    if (rc != 0) {
        printf("spawn returned %d (errno=%d)\n", rc, errno);
        return 1;
    }
    printf("dj-dj-exec=ok\n");
    return 0;
}

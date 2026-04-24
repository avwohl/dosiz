/* dj_nest3.c -- three-level nested DJGPP exec chain.
 *
 * DJ_NEST3 spawns DJ_DJE, which spawns DJ_WRITE.  So the total
 * AH=4B depth under dosiz is:
 *
 *   dosiz -> DJ_NEST3.exe                        (depth 1 PM entry)
 *            |-> DJ_DJE.exe                       (depth 2)
 *                  |-> DJ_WRITE.exe               (depth 3)
 *
 * Each child is a DJGPP binary that enters PM, installs its own
 * PM exception handlers, reconfigures LDT, then cleanly exits.
 * Every return path has to restore the calling level's state
 * without clobbering the deeper chain state snapshot.
 *
 * Prints "dj-nest3=ok" on success.
 */
#include <stdio.h>
#include <stdlib.h>
#include <process.h>
#include <errno.h>

int main(void) {
    int rc = spawnlp(P_WAIT, "C:\\TESTS\\DJ_DJE.exe",
                     "DJ_DJE.exe", (char *)NULL);
    if (rc != 0) {
        printf("spawn returned %d (errno=%d)\n", rc, errno);
        return 1;
    }
    printf("dj-nest3=ok\n");
    return 0;
}

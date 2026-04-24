/* dj_env.c -- smoke test: getenv for values the DOS PSP provides.
 *
 * dosiz populates COMSPEC=C:\COMMAND.COM and PATH=C:\ in the env
 * block regardless of host env.  Check both, then print marker.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(void) {
    const char *comspec = getenv("COMSPEC");
    const char *path    = getenv("PATH");
    if (!comspec || strcmp(comspec, "C:\\COMMAND.COM") != 0) {
        printf("FAIL: COMSPEC=%s\n", comspec ? comspec : "(null)");
        return 1;
    }
    if (!path || strcmp(path, "C:\\") != 0) {
        printf("FAIL: PATH=%s\n", path ? path : "(null)");
        return 2;
    }
    printf("COMSPEC=%s\n", comspec);
    printf("PATH=%s\n", path);
    printf("dj-env=ok\n");
    return 0;
}

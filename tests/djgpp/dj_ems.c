/* dj_ems.c -- probe EMS (INT 67h) availability.
 *
 * Smoke-test whether an EMS provider is installed:
 *   1. INT 67h vector is nonzero
 *   2. EMMXXXX0 device can be opened (DOS AH=3D)
 *   3. AX=4000 (installation status) returns AH=0
 *   4. AX=4600 returns EMS version (AL != 0)
 *   5. AX=4200 returns page count (BX total, DX free)
 *
 * Prints one line per step.  Final "dj-ems=ok" marker if all pass.
 */
#include <stdio.h>
#include <string.h>
#include <dpmi.h>
#include <go32.h>
#include <sys/farptr.h>

int main(void) {
    /* (1) INT 67h vector via DPMI get-real-mode-vector */
    __dpmi_raddr v;
    if (__dpmi_get_real_mode_interrupt_vector(0x67, &v) != 0) {
        printf("FAIL: couldn't read INT 67h vector\n");
        return 1;
    }
    printf("INT67=%04x:%04x\n", v.segment, v.offset16);
    if (v.segment == 0 && v.offset16 == 0) {
        printf("FAIL: INT 67h vector is 0:0 (no EMS)\n");
        return 2;
    }

    /* (2) EMMXXXX0 device probe via AH=3D */
    __dpmi_regs r;
    memset(&r, 0, sizeof r);
    /* transfer the name into the DOS transfer buffer */
    const char *name = "EMMXXXX0";
    dosmemput(name, 9, __tb);
    r.x.ax = 0x3D00;
    r.x.ds = __tb >> 4;
    r.x.dx = __tb & 0x0F;
    __dpmi_int(0x21, &r);
    if (r.x.flags & 1) {
        printf("FAIL: open EMMXXXX0 failed AX=%04x\n", r.x.ax);
        return 3;
    }
    uint16_t fh = r.x.ax;
    printf("EMMXXXX0 handle=%04x\n", fh);
    /* close it */
    memset(&r, 0, sizeof r);
    r.x.ax = 0x3E00;
    r.x.bx = fh;
    __dpmi_int(0x21, &r);

    /* (3) INT 67h AH=40: installation status */
    memset(&r, 0, sizeof r);
    r.x.ax = 0x4000;
    __dpmi_int(0x67, &r);
    if ((r.h.ah) != 0) {
        printf("FAIL: AH=40 returned AH=%02x\n", r.h.ah);
        return 4;
    }
    printf("AH=40 install OK\n");

    /* (4) AH=46 get EMM version */
    memset(&r, 0, sizeof r);
    r.x.ax = 0x4600;
    __dpmi_int(0x67, &r);
    if (r.h.ah != 0 || r.h.al == 0) {
        printf("FAIL: AH=46 AX=%04x\n", r.x.ax);
        return 5;
    }
    printf("EMS ver=%x.%x\n", (r.h.al >> 4) & 0xF, r.h.al & 0xF);

    /* (5) AH=42 get page counts */
    memset(&r, 0, sizeof r);
    r.x.ax = 0x4200;
    __dpmi_int(0x67, &r);
    if (r.h.ah != 0) {
        printf("FAIL: AH=42 returned AH=%02x\n", r.h.ah);
        return 6;
    }
    printf("EMS pages total=%u free=%u (16KB each)\n", r.x.dx, r.x.bx);

    /* (6) AH=41 get page frame */
    memset(&r, 0, sizeof r);
    r.x.ax = 0x4100;
    __dpmi_int(0x67, &r);
    if (r.h.ah != 0) {
        printf("FAIL: AH=41 returned AH=%02x\n", r.h.ah);
        return 7;
    }
    printf("EMS page frame=%04x:0\n", r.x.bx);

    printf("dj-ems=ok\n");
    return 0;
}

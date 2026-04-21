; dpmi_extra.asm — covers the "last few" DPMI services:
;   AX=0102  Resize DOS Memory Block
;   AX=0305  Get State Save/Restore Addresses
;   AX=0306  Get Raw Mode Switch Addresses
;   AX=0801  Free Physical Address Mapping
;   AX=0B00  Set Debug Watchpoint (stub accept)
;
; Assemble:  nasm -f bin dpmi_extra.asm -o DPMI_EXTRA.COM

    org 100h

BITS 16
    mov ax, 1687h
    int 2Fh
    test ax, ax
    jnz  not_present
    mov [entry_off], di
    mov [entry_seg], es

    cli
    xor ax, ax
    call far [entry_off]
    test ax, ax
    jnz  switch_failed

    ; AX=0100 alloc 16 paragraphs + selector
    mov ax, 0100h
    mov bx, 16
    int 31h
    jc  fail
    mov [sel], dx

    ; AX=0102 resize to 32 paragraphs
    mov ax, 0102h
    mov bx, [sel]
    mov dx, 32
    int 31h
    jc  fail_resize

    ; AX=0006 verify limit updated via the new byte count
    ; (limit = 32*16-1 = 511 = 0x1FF).  Read back via 000B raw read
    ; and check bytes 0-1.
    mov ax, ds
    mov es, ax
    mov ax, 000Bh
    mov bx, [sel]
    mov di, rawbuf
    int 31h
    jc  fail
    cmp byte [rawbuf + 0], 0FFh
    jne fail_limit
    cmp byte [rawbuf + 1], 01h
    jne fail_limit

    ; AX=0101 free
    mov ax, 0101h
    mov dx, [sel]
    int 31h
    jc  fail

    ; AX=0305 get save/restore -- just expect CF=0
    mov ax, 0305h
    int 31h
    jc  fail

    ; AX=0306 get raw mode switch -- expect CF=0
    mov ax, 0306h
    int 31h
    jc  fail

    ; AX=0801 free phys mapping (stub)
    mov ax, 0801h
    int 31h
    jc  fail

    ; AX=0B00 debug watchpoint stub
    mov ax, 0B00h
    int 31h
    jc  fail

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail:
    mov dx, fail_msg
    jmp p1
fail_resize:
    mov dx, fail_resize_msg
    jmp p1
fail_limit:
    mov dx, fail_limit_msg
    jmp p1

p1:
    mov ah, 9
    int 21h
    mov ax, 4C01h
    int 21h

not_present:
    mov dx, absent_msg
    mov ah, 9
    int 21h
    mov ax, 4C02h
    int 21h

switch_failed:
    mov dx, failed_msg
    mov ah, 9
    int 21h
    mov ax, 4C01h
    int 21h

entry_off         dw 0
entry_seg         dw 0
sel               dw 0
rawbuf            times 8 db 0

ok_msg            db 'dpmi-extra=ok', 13, 10, '$'
fail_msg          db 'dpmi-extra=fail', 13, 10, '$'
fail_resize_msg   db 'dpmi-extra=fail-resize', 13, 10, '$'
fail_limit_msg    db 'dpmi-extra=fail-limit', 13, 10, '$'
absent_msg        db 'dpmi-extra=absent', 13, 10, '$'
failed_msg        db 'dpmi-extra=switch-failed', 13, 10, '$'

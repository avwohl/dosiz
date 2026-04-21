; dpmi_stage4c.asm — exercises INT 31h service calls real DPMI clients
; make during init but which don't have their own dedicated fixture:
;
;   AX=0204  Get real-mode interrupt vector     (BL=vec -> CX:DX = seg:off)
;   AX=0205  Set real-mode interrupt vector     (BL, CX:DX)
;   AX=0500  Get free memory information        (0x30-byte buffer at ES:DI)
;
; Flow (16-bit PM):
;   1. AX=0204 BL=21h -> read the IVT entry for INT 21h.  It must be
;      non-zero (dosbox + our handler registered a vector).  Save it.
;   2. AX=0205 BL=DEh with a distinctive seg:off, then AX=0204 BL=DEh
;      and verify the round-trip.  DE is unused in DOS so we won't
;      break anything else.
;   3. AX=0500 with ES:DI pointing at a local 0x30 buffer; verify the
;      returned "largest free block" DWORD is non-zero.
;   4. Print dpmi-stage4c=ok, exit 0.

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

    ; -- AX=0204 read INT 21h vector ------------------------------
    mov ax, 0200h
    mov bl, 21h
    int 31h
    jc  fail_get21
    mov ax, cx
    or  ax, dx
    jz  fail_get21

    ; -- AX=0205 set INT DEh to CAFE:BABEh --------------------------
    mov ax, 0201h
    mov bl, 0DEh
    mov cx, 0CAFEh
    mov dx, 0BABEh
    int 31h
    jc  fail_set

    ; -- AX=0204 read it back --------------------------------------
    mov ax, 0200h
    mov bl, 0DEh
    int 31h
    jc  fail_getde
    cmp cx, 0CAFEh
    jne fail_getde
    cmp dx, 0BABEh
    jne fail_getde

    ; -- AX=0500 get free memory info ------------------------------
    ; Our PM DS is selector 0x10 with base = original DS * 16 = PSP*16.
    ; ES already equals DS (same PSP base).  Point DI at our local buf.
    mov ax, ds
    mov es, ax
    mov ax, 0500h
    mov di, freebuf
    int 31h
    jc  fail_info
    ; First dword = largest-available-block (bytes).  Must be > 0.
    mov ax, [freebuf + 0]
    mov bx, [freebuf + 2]
    or  ax, bx
    jz  fail_info

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_get21:
    mov dx, fail_get21_msg
    jmp print_and_exit_1
fail_set:
    mov dx, fail_set_msg
    jmp print_and_exit_1
fail_getde:
    mov dx, fail_getde_msg
    jmp print_and_exit_1
fail_info:
    mov dx, fail_info_msg
    jmp print_and_exit_1

print_and_exit_1:
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

entry_off          dw 0
entry_seg          dw 0
freebuf            times 48 db 0    ; 0x30 bytes

ok_msg             db 'dpmi-stage4c=ok', 13, 10, '$'
fail_get21_msg     db 'dpmi-stage4c=fail-get21', 13, 10, '$'
fail_set_msg       db 'dpmi-stage4c=fail-set', 13, 10, '$'
fail_getde_msg     db 'dpmi-stage4c=fail-getde', 13, 10, '$'
fail_info_msg      db 'dpmi-stage4c=fail-info', 13, 10, '$'
absent_msg         db 'dpmi-stage4c=absent', 13, 10, '$'
failed_msg         db 'dpmi-stage4c=switch-failed', 13, 10, '$'

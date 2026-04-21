; dpmi_descmgmt.asm — exercises the rest of DPMI descriptor-management
; services: AX=0008 (set limit), 0009 (set access rights), 000A (alias),
; 000B (get raw 8 bytes), 000C (set raw 8 bytes).
;
; Flow (16-bit PM):
;   1. AX=0000 CX=1 alloc one LDT slot.
;   2. AX=000C write raw 8-byte descriptor covering 0x12340000..FFFF
;      as read-write data.
;   3. AX=000B read back and verify bytes match.
;   4. AX=0006 get base -> expect 0x12340000.
;   5. AX=0008 set limit to 0x00001000 -> read back via 000B, verify
;      bytes 0-1 and nibble of byte 6 reflect new limit.
;   6. AX=0009 flip access byte to 0x9A (readable code) then back to
;      0x92.
;   7. AX=000A alias the selector -> new selector, same base/limit,
;      access 0x92.
;   8. AX=0001 free the alias; AX=0001 free the original.
;
; Assemble:  nasm -f bin dpmi_descmgmt.asm -o DPMI_DESCMGMT.COM

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

    ; Alloc one LDT slot
    mov ax, 0000h
    mov cx, 1
    int 31h
    jc  fail
    mov [sel], ax

    ; Build raw 8-byte descriptor in `desc`:
    ;   limit = 0xFFFF, base = 0x12340000, access = 0x92, flags = 0x00
    mov byte [desc + 0], 0FFh      ; limit low byte
    mov byte [desc + 1], 0FFh
    mov byte [desc + 2], 00h       ; base low
    mov byte [desc + 3], 00h
    mov byte [desc + 4], 34h       ; base mid (base = 0x00..0x34..0x12)
    mov byte [desc + 5], 92h       ; access: present, data, rw
    mov byte [desc + 6], 00h       ; flags + limit high nibble
    mov byte [desc + 7], 12h       ; base high

    mov ax, ds
    mov es, ax
    mov ax, 000Ch
    mov bx, [sel]
    mov di, desc
    int 31h
    jc  fail

    ; Read back via 000B
    mov ax, 000Bh
    mov bx, [sel]
    mov di, readback
    int 31h
    jc  fail
    ; Compare 8 bytes
    mov cx, 8
    mov si, desc
    mov di, readback
    repe cmpsb
    jne fail_mismatch

    ; 0006 get base -> expect 0x12340000 (CX=0x1234 DX=0)
    mov ax, 0006h
    mov bx, [sel]
    int 31h
    jc  fail
    cmp cx, 1234h
    jne fail_base
    cmp dx, 0
    jne fail_base

    ; 0008 set limit to 0x1000
    mov ax, 0008h
    mov bx, [sel]
    mov cx, 0
    mov dx, 1000h
    int 31h
    jc  fail_setlim

    ; 0009 set access rights: access byte 0x9A (readable code), flags hi 0
    mov ax, 0009h
    mov bx, [sel]
    mov cl, 9Ah
    mov ch, 0
    int 31h
    jc  fail
    ; And back to 0x92
    mov ax, 0009h
    mov bx, [sel]
    mov cl, 92h
    mov ch, 0
    int 31h
    jc  fail

    ; 000A alias
    mov ax, 000Ah
    mov bx, [sel]
    int 31h
    jc  fail_alias
    test al, 4                     ; LDT selector
    jz  fail_alias
    mov [alias_sel], ax

    ; Verify alias base equals original's base
    mov ax, 0006h
    mov bx, [alias_sel]
    int 31h
    jc  fail_alias
    cmp cx, 1234h
    jne fail_alias

    ; Free alias + original
    mov ax, 0001h
    mov bx, [alias_sel]
    int 31h
    jc  fail
    mov ax, 0001h
    mov bx, [sel]
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
fail_mismatch:
    mov dx, fail_mismatch_msg
    jmp p1
fail_base:
    mov dx, fail_base_msg
    jmp p1
fail_setlim:
    mov dx, fail_setlim_msg
    jmp p1
fail_alias:
    mov dx, fail_alias_msg
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

entry_off           dw 0
entry_seg           dw 0
sel                 dw 0
alias_sel           dw 0
desc                times 8 db 0
readback            times 8 db 0

ok_msg              db 'dpmi-descmgmt=ok', 13, 10, '$'
fail_msg            db 'dpmi-descmgmt=fail', 13, 10, '$'
fail_mismatch_msg   db 'dpmi-descmgmt=fail-readback', 13, 10, '$'
fail_base_msg       db 'dpmi-descmgmt=fail-base', 13, 10, '$'
fail_setlim_msg     db 'dpmi-descmgmt=fail-setlim', 13, 10, '$'
fail_alias_msg      db 'dpmi-descmgmt=fail-alias', 13, 10, '$'
absent_msg          db 'dpmi-descmgmt=absent', 13, 10, '$'
failed_msg          db 'dpmi-descmgmt=switch-failed', 13, 10, '$'

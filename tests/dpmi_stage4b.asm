; dpmi_stage4b.asm — exercises LDT descriptor alloc/free/convert
; (INT 31h AX=0000/0001/0002/0003).  Complements dpmi_stage4.asm which
; covered AX=0400/0006/0007 on the statically-built GDT.
;
; Flow (16-bit PM):
;   1. AX=0003  -> expect AX=8 (selector increment)
;   2. AX=0000 CX=3  -> allocate 3 consecutive LDT descriptors;
;      expect CF=0 and the returned selector has TI=1 (bit 2 set).
;   3. AX=0007  -> set base of descriptor 0 to DS*16 (alias our data).
;      AX=0007 must accept LDT selectors just as well as GDT ones.
;   4. mov es, <alloc[0]>  -> load the LDT selector, descriptor cache
;      refreshes.  Read the PSP INT 20h word (0x20CD at offset 0) via
;      ES to confirm the alias works.
;   5. AX=0002 BX=DS  -> segment-to-descriptor.  Repeat: second call
;      must return the same selector (DPMI spec).
;   6. AX=0001 <alloc[1]>  -> free middle descriptor; CF=0.
;   7. Print dpmi-stage4b=ok, exit 0.
;
; Assemble:  nasm -f bin dpmi_stage4b.asm -o DPMI_STAGE4B.COM

    org 100h

BITS 16
    mov ax, 1687h
    int 2Fh
    test ax, ax
    jnz  not_present

    mov [entry_off], di
    mov [entry_seg], es

    ; Save DS for later (INT 2Fh leaves DS alone, but after the switch
    ; DS is the PM selector 0x10 whose base = original DS * 16 -- we
    ; want the real-mode segment value for AX=0002).
    mov ax, ds
    mov [saved_ds], ax

    cli
    xor ax, ax
    call far [entry_off]

    test ax, ax
    jnz  switch_failed

    ; -- AX=0003 Get Selector Increment ---------------------------
    mov ax, 0003h
    int 31h
    jc  fail_inc
    cmp ax, 8
    jne fail_inc

    ; -- AX=0000 Alloc 3 descriptors -----------------------------
    mov ax, 0000h
    mov cx, 3
    int 31h
    jc  fail_alloc
    test al, 4                      ; TI bit should be set (LDT)
    jz  fail_alloc
    mov [slot0], ax
    add ax, 8
    mov [slot1], ax
    add ax, 8
    mov [slot2], ax

    ; -- AX=0007 Set base of slot0 to saved_ds * 16 --------------
    ; CX:DX = linear base = (saved_ds << 4) & 0xFFFFF; high 16 of a
    ; 20-bit paragraph address can be non-zero (bits 4..19 mapped),
    ; compute it exactly: high = saved_ds >> 12, low = saved_ds << 4.
    mov ax, [saved_ds]
    mov dx, ax
    shl dx, 4                       ; low 16 of base
    mov cx, ax
    shr cx, 12                      ; high 16 of base (really only bits 16..19 occupied)
    mov ax, 0007h
    mov bx, [slot0]
    int 31h
    jc  fail_setbase

    ; -- Load slot0 into ES, read PSP INT 20h marker via that alias
    mov ax, [slot0]
    mov es, ax
    mov ax, [es:0]                  ; PSP's first word = 0x20CD (INT 20)
    cmp ax, 20CDh
    jne fail_alias

    ; -- AX=0002 Segment-to-descriptor on our DS ------------------
    mov ax, 0002h
    mov bx, [saved_ds]
    int 31h
    jc  fail_s2d
    test al, 4
    jz  fail_s2d
    mov [s2d_a], ax

    ; Second call with same BX must return the same selector.
    mov ax, 0002h
    mov bx, [saved_ds]
    int 31h
    jc  fail_s2d
    cmp ax, [s2d_a]
    jne fail_s2d

    ; -- AX=0001 Free slot1 ---------------------------------------
    mov ax, 0001h
    mov bx, [slot1]
    int 31h
    jc  fail_free

    ; -- Sanity: freeing the same slot again should error ---------
    mov ax, 0001h
    mov bx, [slot1]
    int 31h
    jnc fail_double_free
    cmp ax, 8022h
    jne fail_double_free

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_inc:
    mov dx, fail_inc_msg
    jmp print_and_exit_1
fail_alloc:
    mov dx, fail_alloc_msg
    jmp print_and_exit_1
fail_setbase:
    mov dx, fail_setbase_msg
    jmp print_and_exit_1
fail_alias:
    mov dx, fail_alias_msg
    jmp print_and_exit_1
fail_s2d:
    mov dx, fail_s2d_msg
    jmp print_and_exit_1
fail_free:
    mov dx, fail_free_msg
    jmp print_and_exit_1
fail_double_free:
    mov dx, fail_double_free_msg
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

entry_off             dw 0
entry_seg             dw 0
saved_ds              dw 0
slot0                 dw 0
slot1                 dw 0
slot2                 dw 0
s2d_a                 dw 0

ok_msg                db 'dpmi-stage4b=ok', 13, 10, '$'
fail_inc_msg          db 'dpmi-stage4b=fail-inc', 13, 10, '$'
fail_alloc_msg        db 'dpmi-stage4b=fail-alloc', 13, 10, '$'
fail_setbase_msg      db 'dpmi-stage4b=fail-setbase', 13, 10, '$'
fail_alias_msg        db 'dpmi-stage4b=fail-alias', 13, 10, '$'
fail_s2d_msg          db 'dpmi-stage4b=fail-s2d', 13, 10, '$'
fail_free_msg         db 'dpmi-stage4b=fail-free', 13, 10, '$'
fail_double_free_msg  db 'dpmi-stage4b=fail-double-free', 13, 10, '$'
absent_msg            db 'dpmi-stage4b=absent', 13, 10, '$'
failed_msg            db 'dpmi-stage4b=switch-failed', 13, 10, '$'

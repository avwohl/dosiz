; dpmi_pmalloc.asm -- exercises the pm_arena fallback tier of AX=0501.
;
; MCB arena caps out at 0xFFFF paragraphs (~1MB).  Asking for >1MB
; forces dosiz to fall through to pm_alloc in the extended-memory
; pool above 1MB.  For pm_arena blocks the handle encoding is
; SI:DI = high:low of the host linear base (SI >= 0x0010 because
; base >= 0x100000).
;
; Flow (16-bit PM):
;   1. INT 31h AX=0501 BX:CX = 0x00100100 (just over 1MB).
;   2. Verify SI >= 0x10 and SI == BX and DI == CX.
;   3. AX=0007 repoint ES (sel 0x20) at the block.
;   4. Write/read a word at es:0 to prove the memory is mapped.
;   5. AX=0503 shrink in place to 4KB; verify SI:DI unchanged.
;   6. AX=0502 free via SI:DI.
;   7. Print dpmi-pmalloc=ok.
;
; Assemble:  nasm -f bin dpmi_pmalloc.asm -o DPMI_PMALLOC.COM

    org 100h

BITS 16
    mov ax, 1687h
    int 2Fh
    test ax, ax
    jnz  not_present
    mov [entry_off], di
    mov [entry_seg], es

    cli
    xor ax, ax                     ; 16-bit PM
    call far [entry_off]
    test ax, ax
    jnz  switch_failed

    ; -- AX=0501 Allocate 0x00100100 bytes --------------------------
    mov ax, 0501h
    mov bx, 0010h
    mov cx, 0100h
    int 31h
    jc  fail_alloc

    mov [base_hi], bx
    mov [base_lo], cx
    mov [handle_hi], si
    mov [handle_lo], di

    ; SI must be >= 0x10 (linear >= 1MB).
    cmp si, 10h
    jb  fail_tier

    ; For pm_arena blocks SI:DI == BX:CX.
    cmp si, bx
    jne fail_handle
    cmp di, cx
    jne fail_handle

    ; -- AX=0007 Point ES (sel 0x20) at the block -------------------
    mov ax, 0007h
    mov bx, 0020h
    mov cx, [base_hi]
    mov dx, [base_lo]
    int 31h
    jc  fail_setbase
    mov ax, 0020h
    mov es, ax

    ; -- Write + read back pattern at offset 0 ----------------------
    mov word [es:0], 0BEEFh
    mov ax, [es:0]
    cmp ax, 0BEEFh
    jne fail_readback

    ; -- AX=0503 Shrink to 4KB --------------------------------------
    mov ax, 0503h
    mov si, [handle_hi]
    mov di, [handle_lo]
    mov bx, 0
    mov cx, 1000h
    int 31h
    jc  fail_resize
    cmp bx, [base_hi]
    jne fail_resize
    cmp cx, [base_lo]
    jne fail_resize

    ; -- AX=0502 Free -----------------------------------------------
    mov ax, 0502h
    mov si, [handle_hi]
    mov di, [handle_lo]
    int 31h
    jc  fail_free

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_alloc:    mov dx, fail_alloc_msg
               jmp print_and_exit_1
fail_tier:     mov dx, fail_tier_msg
               jmp print_and_exit_1
fail_handle:   mov dx, fail_handle_msg
               jmp print_and_exit_1
fail_setbase:  mov dx, fail_setbase_msg
               jmp print_and_exit_1
fail_readback: mov dx, fail_readback_msg
               jmp print_and_exit_1
fail_resize:   mov dx, fail_resize_msg
               jmp print_and_exit_1
fail_free:     mov dx, fail_free_msg
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

entry_off         dw 0
entry_seg         dw 0
base_hi           dw 0
base_lo           dw 0
handle_hi         dw 0
handle_lo         dw 0

ok_msg            db 'dpmi-pmalloc=ok', 13, 10, '$'
fail_alloc_msg    db 'dpmi-pmalloc=fail-alloc', 13, 10, '$'
fail_tier_msg     db 'dpmi-pmalloc=fail-tier', 13, 10, '$'
fail_handle_msg   db 'dpmi-pmalloc=fail-handle', 13, 10, '$'
fail_setbase_msg  db 'dpmi-pmalloc=fail-setbase', 13, 10, '$'
fail_readback_msg db 'dpmi-pmalloc=fail-readback', 13, 10, '$'
fail_resize_msg   db 'dpmi-pmalloc=fail-resize', 13, 10, '$'
fail_free_msg     db 'dpmi-pmalloc=fail-free', 13, 10, '$'
absent_msg        db 'dpmi-pmalloc=absent', 13, 10, '$'
failed_msg        db 'dpmi-pmalloc=switch-failed', 13, 10, '$'

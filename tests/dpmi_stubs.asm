; dpmi_stubs.asm — round-trip tests for the "stub" DPMI services
; clients probe during init but which dosiz implements as no-ops
; (no paging, no exception dispatch):
;
;   AX=0202/0203  Get/Set PM Exception Handler
;   AX=0600/0601  Lock/Unlock Linear Region
;   AX=0604       Get Page Size
;   AX=0800       Physical Address Mapping
;
; Assemble:  nasm -f bin dpmi_stubs.asm -o DPMI_STUBS.COM

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

    ; -- AX=0203 set PM exception handler for #GP (vector 13) --------
    mov ax, 0203h
    mov bl, 13
    mov cx, 0CAFEh
    mov dx, 0BABEh
    int 31h
    jc  fail_set_exc

    ; -- AX=0202 read it back ---------------------------------------
    mov ax, 0202h
    mov bl, 13
    int 31h
    jc  fail_get_exc
    cmp cx, 0CAFEh
    jne fail_get_exc
    cmp dx, 0BABEh
    jne fail_get_exc

    ; -- AX=0203 with bad vector -> expect CF=1 ---------------------
    mov ax, 0203h
    mov bl, 40
    int 31h
    jnc fail_exc_range

    ; -- AX=0604 get page size -> expect BX:CX = 0:4096 -------------
    mov ax, 0604h
    int 31h
    jc  fail_pagesize
    test bx, bx
    jnz fail_pagesize
    cmp cx, 4096
    jne fail_pagesize

    ; -- AX=0600 lock linear region -> expect CF=0 ------------------
    mov ax, 0600h
    mov bx, 0
    mov cx, 0
    mov si, 0
    mov di, 4096
    int 31h
    jc  fail_lock

    ; -- AX=0601 unlock -> expect CF=0 ------------------------------
    mov ax, 0601h
    int 31h
    jc  fail_lock

    ; -- AX=0800 physical address mapping: echoes BX:CX -------------
    ; Pass 0x000B:0x8000 (VGA text buffer) + 4KB size.  Output linear
    ; should equal input (we don't remap; physical == linear).
    mov ax, 0800h
    mov bx, 000Bh
    mov cx, 8000h
    mov si, 0
    mov di, 4096
    int 31h
    jc  fail_map
    cmp bx, 000Bh
    jne fail_map
    cmp cx, 8000h
    jne fail_map

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_set_exc:
    mov dx, fail_set_exc_msg
    jmp print_and_exit_1
fail_get_exc:
    mov dx, fail_get_exc_msg
    jmp print_and_exit_1
fail_exc_range:
    mov dx, fail_exc_range_msg
    jmp print_and_exit_1
fail_pagesize:
    mov dx, fail_pagesize_msg
    jmp print_and_exit_1
fail_lock:
    mov dx, fail_lock_msg
    jmp print_and_exit_1
fail_map:
    mov dx, fail_map_msg
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

entry_off            dw 0
entry_seg            dw 0

ok_msg               db 'dpmi-stubs=ok', 13, 10, '$'
fail_set_exc_msg     db 'dpmi-stubs=fail-set-exc', 13, 10, '$'
fail_get_exc_msg     db 'dpmi-stubs=fail-get-exc', 13, 10, '$'
fail_exc_range_msg   db 'dpmi-stubs=fail-exc-range', 13, 10, '$'
fail_pagesize_msg    db 'dpmi-stubs=fail-pagesize', 13, 10, '$'
fail_lock_msg        db 'dpmi-stubs=fail-lock', 13, 10, '$'
fail_map_msg         db 'dpmi-stubs=fail-map', 13, 10, '$'
absent_msg           db 'dpmi-stubs=absent', 13, 10, '$'
failed_msg           db 'dpmi-stubs=switch-failed', 13, 10, '$'

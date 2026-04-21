; dpmi_pmidt.asm — INT 31h AX=0210/0212 (get/set PM interrupt vector)
; and AX=0900/0901/0902 (virtual interrupt state).
;
; Flow (16-bit PM):
;   1. AX=0210 BL=21h -> read existing INT 21h PM gate (stash
;      selector:offset).
;   2. AX=0212 BL=70h CX:EDX=<stashed> -> install the same pointer as
;      vector 70h (sanity: same descriptor should work for a different
;      slot).
;   3. AX=0210 BL=70h -> verify round-trip.
;   4. AX=0902 -> expect AL=1 (default virtual IF).
;   5. AX=0900 -> expect AL=1 + now disabled.
;   6. AX=0902 -> expect AL=0.
;   7. AX=0901 -> expect AL=0 + now enabled.
;   8. AX=0902 -> expect AL=1.
;
; Assemble:  nasm -f bin dpmi_pmidt.asm -o DPMI_PMIDT.COM

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

    ; -- AX=0210 get INT 21h PM vector -------------------------------
    mov ax, 0204h
    mov bl, 21h
    int 31h
    jc  fail_get21
    mov [saved_sel], cx
    mov [saved_off], dx

    ; -- AX=0212 install same pointer at vector 70h -----------------
    mov ax, 0205h
    mov bl, 70h
    mov cx, [saved_sel]
    mov dx, [saved_off]
    int 31h
    jc  fail_set70

    ; -- AX=0210 BL=70h to read it back -----------------------------
    mov ax, 0204h
    mov bl, 70h
    int 31h
    jc  fail_get70
    cmp cx, [saved_sel]
    jne fail_get70
    cmp dx, [saved_off]
    jne fail_get70

    ; -- AX=0902 query virtual IF; expect 1 -------------------------
    mov ax, 0902h
    int 31h
    jc  fail_vif
    cmp al, 1
    jne fail_vif

    ; -- AX=0900 disable -> expect previous=1 -----------------------
    mov ax, 0900h
    int 31h
    jc  fail_vif
    cmp al, 1
    jne fail_vif

    ; -- AX=0902 query -> expect 0 ---------------------------------
    mov ax, 0902h
    int 31h
    cmp al, 0
    jne fail_vif

    ; -- AX=0901 enable -> expect previous=0 -----------------------
    mov ax, 0901h
    int 31h
    cmp al, 0
    jne fail_vif

    ; -- AX=0902 query -> expect 1 ---------------------------------
    mov ax, 0902h
    int 31h
    cmp al, 1
    jne fail_vif

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_get21:
    mov dx, fail_get21_msg
    jmp print_and_exit_1
fail_set70:
    mov dx, fail_set70_msg
    jmp print_and_exit_1
fail_get70:
    mov dx, fail_get70_msg
    jmp print_and_exit_1
fail_vif:
    mov dx, fail_vif_msg
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
saved_sel         dw 0
saved_off         dw 0

ok_msg            db 'dpmi-pmidt=ok', 13, 10, '$'
fail_get21_msg    db 'dpmi-pmidt=fail-get21', 13, 10, '$'
fail_set70_msg    db 'dpmi-pmidt=fail-set70', 13, 10, '$'
fail_get70_msg    db 'dpmi-pmidt=fail-get70', 13, 10, '$'
fail_vif_msg      db 'dpmi-pmidt=fail-vif', 13, 10, '$'
absent_msg        db 'dpmi-pmidt=absent', 13, 10, '$'
failed_msg        db 'dpmi-pmidt=switch-failed', 13, 10, '$'

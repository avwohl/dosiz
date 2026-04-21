; dpmi_callrmi.asm — INT 31h AX=0302 "Call Real Mode Procedure With
; IRET Frame".
;
; Companion to dpmi_callrm.asm (AX=0301, uses RETF).  The RM routine
; here ends in IRET instead of RETF, which pops FLAGS:CS:IP (three
; words) vs RETF's CS:IP (two words).  AX=0302 pushes the matching
; three-word frame pointing at our host's stop callback before
; jumping to the client's target, so the IRET unwinds cleanly.
;
; Flow:
;   1. Enter 16-bit PM.
;   2. Populate struct.CS:IP at the bundled rm_proc (mov ax,0x5678; iret).
;   3. AX=0302.
;   4. Verify struct.EAX = 0x5678.
;
; Assemble:  nasm -f bin dpmi_callrmi.asm -o DPMI_CALLRMI.COM

    org 100h

BITS 16
    mov ax, 1687h
    int 2Fh
    test ax, ax
    jnz  not_present

    mov [entry_off], di
    mov [entry_seg], es

    mov ax, ds
    mov [saved_ds_rm], ax

    cli
    xor ax, ax
    call far [entry_off]
    test ax, ax
    jnz  switch_failed

    mov cx, 50
    mov di, rmcs
    xor ax, ax
    rep stosb

    mov ax, [saved_ds_rm]
    mov [rmcs + 0x2C], ax
    mov ax, rm_proc
    mov [rmcs + 0x2A], ax

    mov ax, ds
    mov es, ax

    mov ax, 0302h
    mov bh, 0
    xor cx, cx
    mov di, rmcs
    int 31h
    jc  fail_callrmi

    mov ax, [rmcs + 0x1C]
    cmp ax, 5678h
    jne fail_eax

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_callrmi:
    mov dx, fail_callrmi_msg
    jmp print_and_exit_1
fail_eax:
    mov dx, fail_eax_msg
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

rm_proc:
    mov ax, 5678h
    iret

entry_off            dw 0
entry_seg            dw 0
saved_ds_rm          dw 0

rmcs                 times 50 db 0

ok_msg               db 'dpmi-callrmi=ok', 13, 10, '$'
fail_callrmi_msg     db 'dpmi-callrmi=fail-callrmi', 13, 10, '$'
fail_eax_msg         db 'dpmi-callrmi=fail-eax', 13, 10, '$'
absent_msg           db 'dpmi-callrmi=absent', 13, 10, '$'
failed_msg           db 'dpmi-callrmi=switch-failed', 13, 10, '$'

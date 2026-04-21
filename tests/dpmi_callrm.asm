; dpmi_callrm.asm — INT 31h AX=0301 "Call Real Mode Procedure
; With Far Return Frame".
;
; The fixture bundles its own 2-byte RM procedure `mov ax, 0x1234;
; retf` inside the .COM.  From 16-bit PM it populates a
; RealModeCallStructure with CS:IP pointing at that procedure,
; calls AX=0301, and verifies EAX in the struct came back as 0x1234.
;
; Exercises the same mode-switch round-trip AX=0300 uses, but with
; CALLBACK_RunRealFar as the RM dispatcher instead of
; CALLBACK_RunRealInt -- confirming the shared prologue/epilogue
; (IDTR swap, CR0 toggle, PM segment + CS cache refresh) is general
; enough.
;
; Assemble:  nasm -f bin dpmi_callrm.asm -o DPMI_CALLRM.COM

    org 100h

BITS 16
    mov ax, 1687h
    int 2Fh
    test ax, ax
    jnz  not_present

    mov [entry_off], di
    mov [entry_seg], es

    ; Capture the real-mode CS of this .COM before the switch.  In a
    ; .COM the loader sets CS=DS=SS=PSP, so DS holds the RM seg we
    ; want.  Stash into the RM-call struct's CS field later.
    mov ax, ds
    mov [saved_ds_rm], ax

    cli
    xor ax, ax                     ; 16-bit PM
    call far [entry_off]
    test ax, ax
    jnz  switch_failed

    ; Zero the 50-byte struct.
    mov cx, 50
    mov di, rmcs
    xor ax, ax
    rep stosb

    ; Populate struct.CS = saved_ds_rm, struct.IP = offset of rm_proc.
    mov ax, [saved_ds_rm]
    mov [rmcs + 0x2C], ax          ; CS
    mov ax, rm_proc
    mov [rmcs + 0x2A], ax          ; IP

    ; Point ES at our data (same PSP alias as DS post-switch).
    mov ax, ds
    mov es, ax

    mov ax, 0301h
    mov bh, 0                      ; flags reserved
    xor cx, cx                     ; no PM-stack copy
    mov di, rmcs
    int 31h
    jc  fail_callrm

    ; Verify struct EAX = 0x1234 -- what rm_proc wrote.
    mov ax, [rmcs + 0x1C]          ; EAX low word
    cmp ax, 1234h
    jne fail_eax

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_callrm:
    mov dx, fail_callrm_msg
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

; Real-mode procedure to be called by AX=0301.  Lives inside the .COM
; image at PSP:<offset of rm_proc>.  Must end in RETF -- AX=0301 pushes
; a far-return address pointing at its "stop" callback before
; transferring control.
rm_proc:
    mov ax, 1234h
    retf

entry_off            dw 0
entry_seg            dw 0
saved_ds_rm          dw 0

rmcs                 times 50 db 0

ok_msg               db 'dpmi-callrm=ok', 13, 10, '$'
fail_callrm_msg      db 'dpmi-callrm=fail-callrm', 13, 10, '$'
fail_eax_msg         db 'dpmi-callrm=fail-eax', 13, 10, '$'
absent_msg           db 'dpmi-callrm=absent', 13, 10, '$'
failed_msg           db 'dpmi-callrm=switch-failed', 13, 10, '$'

; dpmi_stage2.asm — verifies DPMI stage 2 contract.
;
; DPMI stage 2 says:
;   1. INT 2Fh AX=1687h reports host present (AX returns 0).
;   2. ES:DI on return points at a valid real-mode switch entry.
;   3. FAR CALL to that entry returns AX != 0 because the mode switch
;      isn't implemented yet.  Clients that can fall back to real-mode
;      operation survive this; the full DJGPP/Watcom path needs stage 3.
;
; Prints one of:
;   dpmi-stage2=ok              -- all three contract points held
;   dpmi-stage2=not-present     -- detection said "absent"
;   dpmi-stage2=switch-ok       -- entry point returned success (?!)
;
; Assemble:  nasm -f bin dpmi_stage2.asm -o DPMI_STAGE2.COM

    org 100h

    mov ax, 1687h
    int 2Fh
    test ax, ax
    jnz  not_present

    ; Save the real-mode entry pointer.
    mov [entry_off], di
    mov [entry_seg], es

    ; AX=0 -> request 16-bit protected mode.  Then FAR CALL the entry.
    xor ax, ax
    call far [entry_off]

    ; On return: AX=0 means mode switch succeeded (we shouldn't see this
    ; in stage 2).  Anything else means the switch failed -- the stage 2
    ; contract.
    test ax, ax
    jz   unexpected_success

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

not_present:
    mov dx, absent_msg
    mov ah, 9
    int 21h
    mov ax, 4C02h
    int 21h

unexpected_success:
    mov dx, unexpected_msg
    mov ah, 9
    int 21h
    mov ax, 4C01h
    int 21h

entry_off        dw 0
entry_seg        dw 0

ok_msg          db 'dpmi-stage2=ok', 13, 10, '$'
absent_msg      db 'dpmi-stage2=not-present', 13, 10, '$'
unexpected_msg  db 'dpmi-stage2=switch-ok', 13, 10, '$'

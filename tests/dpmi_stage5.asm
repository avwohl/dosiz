; dpmi_stage5.asm — verifies INT 21h from PM reaches the host's real-mode
; handler via our IDT interrupt gate and mode-agnostic SegPhys addressing.
;
; Test path:
;   1. Probe DPMI (expect present).
;   2. CLI (mask IRQs -- no timer-interrupt handler in the PM IDT).
;   3. Call the DPMI entry; expect AX=0 (switched to 16-bit PM).
;   4. In PM: do INT 21h AH=09 with a message pointer.  Our host-C++
;      handler reads the string via SegPhys(ds) + reg_dx, writes it to
;      stdout, returns via IRET.
;   5. In PM: do INT 21h AH=4C AL=0 to exit cleanly.
;
; Expected output: the message, then exit code 0.  A regression in the
; mode switch or reflection path would either fault inside dosbox or
; exit non-zero.
;
; Assemble:  nasm -f bin dpmi_stage5.asm -o DPMI_STAGE5.COM

    org 100h

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

    ; Now in PM.  The IDT routes INT 21h to our handler.
    mov ah, 9
    mov dx, pm_msg
    int 21h

    mov ax, 4C00h
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

entry_off   dw 0
entry_seg   dw 0

pm_msg      db 'dpmi-stage5=hello-from-pm', 13, 10, '$'
absent_msg  db 'dpmi-stage5=absent', 13, 10, '$'
failed_msg  db 'dpmi-stage5=switch-failed', 13, 10, '$'

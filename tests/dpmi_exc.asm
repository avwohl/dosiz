; dpmi_exc.asm — verifies PM exception dispatch via AX=0203.
;
; Flow (16-bit PM):
;   1. Install a handler for vector 6 (#UD = invalid opcode) via
;      AX=0203.  Handler's CS = our PM code selector, offset =
;      pm_exc_handler.
;   2. Execute UD2 (0F 0B) -- CPU raises #UD.
;   3. dosbox's CPU_Interrupt finds the gate we installed and dispatches
;      to pm_exc_handler.
;   4. Handler sets fault_flag and rewinds the stacked IP by +2 (the
;      length of UD2) so IRET skips past the faulting instruction.
;   5. After handler IRETs, execution continues at the instruction
;      after UD2; fixture checks fault_flag.
;
; Assemble:  nasm -f bin dpmi_exc.asm -o DPMI_EXC.COM

    org 100h

BITS 16
    mov ax, 1687h
    int 2Fh
    test ax, ax
    jnz  not_present
    mov [entry_off], di
    mov [entry_seg], es

    cli
    xor ax, ax                      ; 16-bit PM
    call far [entry_off]
    test ax, ax
    jnz  switch_failed

    ; Install #UD handler.  CS = our client PM code selector, off =
    ; pm_exc_handler label.  AX=0205 would wire a plain interrupt
    ; vector; AX=0203 both records it in the exception table and
    ; writes the IDT gate.
    mov ax, 0203h
    mov bl, 6                       ; vector 6 = #UD
    mov cx, cs                      ; handler CS = client PM CS
    mov dx, pm_exc_handler
    int 31h
    jc  fail_install

    ; Trigger #UD.  Handler will fire, set fault_flag=1, skip past the
    ; UD2 via pushed-IP rewind.  Execution resumes right after.
    db 0Fh, 0Bh                     ; UD2

    cmp byte [fault_flag], 1
    jne fail_not_handled

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

; --- PM exception handler ---
; On entry (16-bit interrupt gate, no error code for #UD):
;   [SP+0]=IP of UD2, [SP+2]=CS, [SP+4]=FLAGS
; Rewind IP by +2 so IRET returns to the instruction after UD2.
pm_exc_handler:
    push bp
    mov bp, sp
    add word [bp + 2], 2            ; skip UD2 (2 bytes)
    pop bp
    mov byte [fault_flag], 1
    iret

fail_install:
    mov dx, fail_install_msg
    jmp p1
fail_not_handled:
    mov dx, fail_not_handled_msg
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

entry_off              dw 0
entry_seg              dw 0
fault_flag             db 0

ok_msg                 db 'dpmi-exc=ok', 13, 10, '$'
fail_install_msg       db 'dpmi-exc=fail-install', 13, 10, '$'
fail_not_handled_msg   db 'dpmi-exc=fail-not-handled', 13, 10, '$'
absent_msg             db 'dpmi-exc=absent', 13, 10, '$'
failed_msg             db 'dpmi-exc=switch-failed', 13, 10, '$'

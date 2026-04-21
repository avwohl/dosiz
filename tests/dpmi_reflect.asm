; dpmi_reflect.asm — verifies 16-bit PM IDT reflection to real-mode
; BIOS handlers.  Without reflection, any INT from PM other than 21h
; or 31h traps into an empty IDT slot and the emulator aborts with
; "INT:Gate Selector points to illegal descriptor with type 0x0".
;
; Flow:
;   1. In real mode, call INT 10h AH=0F (get current video mode); save
;      the returned AL (mode) and BH (page) as the oracle.
;   2. Enter 16-bit PM.
;   3. Repeat INT 10h AH=0F in PM -- must route through our new IDT
;      gate to the real-mode handler and return the same AL/BH.
;   4. Print dpmi-reflect=ok + exit.
;
; A regression in the reflection path either hangs (empty IDT slot)
; or returns wrong registers; we check both.
;
; Assemble:  nasm -f bin dpmi_reflect.asm -o DPMI_REFLECT.COM

    org 100h

BITS 16
    ; Oracle call in real mode.
    mov ah, 0Fh
    int 10h
    mov [oracle_al], al
    mov [oracle_bh], bh

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

    ; PM-side INT 10h.  Our IDT gate reflects to the real-mode BIOS
    ; handler; the native C++ callback there runs in PM, reads/writes
    ; regs, and returns via 16-bit IRET.
    mov ah, 0Fh
    int 10h
    cmp al, [oracle_al]
    jne fail_mode
    cmp bh, [oracle_bh]
    jne fail_page

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_mode:
    mov dx, fail_mode_msg
    jmp print_and_exit_1
fail_page:
    mov dx, fail_page_msg
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

entry_off      dw 0
entry_seg      dw 0
oracle_al      db 0
oracle_bh      db 0

ok_msg         db 'dpmi-reflect=ok', 13, 10, '$'
fail_mode_msg  db 'dpmi-reflect=fail-mode', 13, 10, '$'
fail_page_msg  db 'dpmi-reflect=fail-page', 13, 10, '$'
absent_msg     db 'dpmi-reflect=absent', 13, 10, '$'
failed_msg     db 'dpmi-reflect=switch-failed', 13, 10, '$'

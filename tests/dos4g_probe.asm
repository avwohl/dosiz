; dos4g_probe.asm -- verifies dosiz answers Watcom's DOS/4GW
; detection probe (INT 21h AH=FFh DH=00h DL=78h) with the exact
; EAX value real DOS/4GW returns (0x4734FFFF; high word 0x4734 is
; byte-swapped "4G" ASCII).  Without this, a Watcom-compiled
; 32-bit LE binary's C runtime assumes no extender is present
; and GP-faults shortly after entry.
;
; Reference: Ralf Brown's Interrupt List, INT 21h AH=FFh DH=00h.
;
; Assemble:  nasm -f bin dos4g_probe.asm -o DOS4G_PROBE.COM

    org 100h

BITS 16
    ; Preset EBX high word to "PH" (ASCII) matching what Open
    ; Watcom's runtime sends; verify our handler ignores the
    ; signature and responds based on DX alone.
    mov ebx, 0x50480000
    mov ecx, 0x1606
    mov edx, 0x0078               ; DH=00, DL=78
    mov ax, 0xFF00
    int 21h
    jc  fail_cf

    ; Expect EAX = 0x4734FFFF.
    cmp eax, 0x4734FFFF
    jne fail_eax

    ; Verify high word of EAX is 0x4734 = "4G" byte-swapped.
    mov eax, eax                  ; truncate to 32-bit
    shr eax, 16
    cmp ax, 0x4734
    jne fail_magic

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_cf:      mov dx, fail_cf_msg
              jmp print_and_exit_1
fail_eax:     mov dx, fail_eax_msg
              jmp print_and_exit_1
fail_magic:   mov dx, fail_magic_msg
              jmp print_and_exit_1

print_and_exit_1:
    mov ah, 9
    int 21h
    mov ax, 4C01h
    int 21h

ok_msg           db 'dos4g-probe=ok', 13, 10, '$'
fail_cf_msg      db 'dos4g-probe=fail-cf', 13, 10, '$'
fail_eax_msg     db 'dos4g-probe=fail-eax', 13, 10, '$'
fail_magic_msg   db 'dos4g-probe=fail-magic', 13, 10, '$'

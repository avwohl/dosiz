; dpmi_dosmem.asm — INT 31h AX=0100/0101 "DOS Memory Alloc/Free".
;
; Flow (16-bit PM):
;   1. AX=0100 BX=16 (256 bytes = 16 paragraphs) -> expect CF=0,
;      AX = RM segment, DX = LDT selector aliasing it.
;   2. Load DX into FS, write a 32-bit pattern at fs:0 and fs:254.
;   3. Using the RM segment in AX, translate via AX=0002
;      (segment-to-descriptor) and expect it returns the SAME selector
;      we got from AX=0100 -- verifies the s_seg2desc_cache wiring.
;   4. AX=0101 DX = selector -> free.
;
; Assemble:  nasm -f bin dpmi_dosmem.asm -o DPMI_DOSMEM.COM

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

    ; -- AX=0100 alloc 16 paragraphs (256 bytes) ---------------------
    mov ax, 0100h
    mov bx, 16
    int 31h
    jc  fail_alloc
    mov [rm_seg], ax
    mov [ldt_sel], dx
    test dl, 4                     ; TI bit set -> LDT selector
    jz  fail_alloc

    ; -- Load selector into FS, round-trip 32-bit pattern ------------
    mov ax, [ldt_sel]
    mov fs, ax
    mov dword [fs:0],   0DEADBEEFh
    mov dword [fs:252], 0CAFEBABEh
    mov eax, [fs:0]
    cmp eax, 0DEADBEEFh
    jne fail_pattern
    mov eax, [fs:252]
    cmp eax, 0CAFEBABEh
    jne fail_pattern

    ; -- AX=0002 seg-to-descriptor should return the cached selector -
    mov ax, 0002h
    mov bx, [rm_seg]
    int 31h
    jc  fail_s2d
    cmp ax, [ldt_sel]
    jne fail_s2d

    ; -- AX=0101 free -----------------------------------------------
    mov ax, 0101h
    mov dx, [ldt_sel]
    int 31h
    jc  fail_free

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_alloc:
    mov dx, fail_alloc_msg
    jmp print_and_exit_1
fail_pattern:
    mov dx, fail_pattern_msg
    jmp print_and_exit_1
fail_s2d:
    mov dx, fail_s2d_msg
    jmp print_and_exit_1
fail_free:
    mov dx, fail_free_msg
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

entry_off          dw 0
entry_seg          dw 0
rm_seg             dw 0
ldt_sel            dw 0

ok_msg             db 'dpmi-dosmem=ok', 13, 10, '$'
fail_alloc_msg     db 'dpmi-dosmem=fail-alloc', 13, 10, '$'
fail_pattern_msg   db 'dpmi-dosmem=fail-pattern', 13, 10, '$'
fail_s2d_msg       db 'dpmi-dosmem=fail-s2d', 13, 10, '$'
fail_free_msg      db 'dpmi-dosmem=fail-free', 13, 10, '$'
absent_msg         db 'dpmi-dosmem=absent', 13, 10, '$'
failed_msg         db 'dpmi-dosmem=switch-failed', 13, 10, '$'

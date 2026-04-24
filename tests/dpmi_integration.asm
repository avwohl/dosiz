; dpmi_integration.asm — end-to-end smoke test exercising most of the
; DPMI host in one run, like an actual client's init sequence would.
;
;   1. INT 2Fh/1687h detect DPMI (must report 32-bit capable).
;   2. Enter 32-bit PM (AX=1).
;   3. INT 31h AX=0400 (get version, expect CL=3 = 386 CPU).
;   4. INT 31h AX=0003 (selector increment, expect 8).
;   5. INT 31h AX=0000 CX=1 (alloc one LDT descriptor).
;   6. INT 31h AX=0501 alloc 1024 bytes.
;   7. INT 31h AX=0007 point the LDT descriptor at the alloc's base.
;   8. Load the descriptor into FS and round-trip a 32-bit pattern at
;      offset 0 and offset 1020 (end-of-block).
;   9. INT 10h AH=0F video mode query via 32-bit reflection (shim path).
;  10. INT 31h AX=0503 resize alloc to 2048 bytes (grow in-place).
;  11. INT 21h AH=09 print success via stage-5-32 IDT gate.
;  12. INT 31h AX=0502 free alloc.
;  13. INT 31h AX=0001 free LDT descriptor.
;  14. INT 21h AH=4C exit 0.
;
; A fail at any stage prints a distinct tag before exiting non-zero.
;
; Assemble:  nasm -f bin dpmi_integration.asm -o DPMI_INTEGRATION.COM

    org 100h

BITS 16
    mov ax, 1687h
    int 2Fh
    test ax, ax
    jnz  not_present
    test bl, 1                     ; bit 0 of BL = 32-bit DPMI supported
    jz   fail_no32

    mov [entry_off], di
    mov [entry_seg], es

    cli
    xor eax, eax
    mov ax, 1                      ; 32-bit PM
    call far [entry_off]
    test ax, ax
    jnz  switch_failed

    ; CWSDPMI (and dosiz by default) hands the client a 16-bit CS
    ; regardless of the AX=1 "32-bit PM" entry (control.c:469
    ; hardcodes D=0).  Stubs that want 32-bit code alloc their own
    ; descriptor and far-jump to it.  Do exactly that before running
    ; the 32-bit test body.

    ; 2a. AX=0000 alloc one LDT descriptor.
    mov ax, 0000h
    mov cx, 1
    int 31h
    jc  switch_failed
    mov bx, ax                     ; bx = new selector (save for later writes)

    ; 2b. AX=0006 get base of current 16-bit CS -> CX:DX.
    push bx
    mov ax, 0006h
    mov bx, cs
    int 31h
    mov di, cx                     ; save segment base hi
    mov si, dx                     ; save segment base lo
    pop bx

    ; 2c. AX=0007 set new sel's base to the same.
    push bx
    mov ax, 0007h
    mov cx, di
    mov dx, si
    int 31h
    pop bx

    ; 2d. AX=0008 set limit to 0xFFFF (1:1 byte granularity is fine).
    push bx
    mov ax, 0008h
    mov cx, 0
    mov dx, 0FFFFh
    int 31h
    pop bx

    ; 2e. AX=0009 set access rights: CL=0xFA (P=1, DPL=3, code r/x),
    ;     CH=0x40 (D=1 so this is a 32-bit code segment).
    mov ax, 0009h
    mov cx, 40FAh
    int 31h

    ; 2f. Patch the selector into the 48-bit far pointer and jump.
    ;     Assembler can't know bx at build time; runtime-patch it.
    mov word [fptr_sel], bx

    o32 jmp far [fptr_off]

BITS 32
pm32_entry:
    ; -- 3. AX=0400 version ---------------------------------------
    mov eax, 0400h
    int 31h
    jc  fail_ver
    cmp cl, 3
    jne fail_ver

    ; -- 4. AX=0003 selector increment ----------------------------
    mov eax, 0003h
    int 31h
    jc  fail_inc
    cmp ax, 8
    jne fail_inc

    ; -- 5. AX=0000 alloc 1 LDT descriptor ------------------------
    mov eax, 0000h
    mov ecx, 1
    int 31h
    jc  fail_ldt
    test al, 4                     ; TI=1
    jz  fail_ldt
    mov [ldt_sel], ax

    ; -- 6. AX=0501 alloc 1024 bytes ------------------------------
    mov eax, 0501h
    mov ebx, 0
    mov ecx, 1024
    int 31h
    jc  fail_alloc
    mov [base_hi], bx
    mov [base_lo], cx
    mov [h_si], si
    mov [h_di], di

    ; -- 7. AX=0007 point LDT sel at alloc's base -----------------
    mov eax, 0007h
    mov bx, [ldt_sel]
    mov cx, [base_hi]
    mov dx, [base_lo]
    int 31h
    jc  fail_setbase

    ; -- 8. Load LDT sel into FS; round-trip 32-bit pattern -------
    ; Zero-extend the 16-bit selector into eax before mov fs, ax.
    movzx eax, word [ldt_sel]
    mov fs, ax

    mov dword [fs:0],    0DEADBEEFh
    mov dword [fs:1020], 0CAFEBABEh
    mov eax, [fs:0]
    cmp eax, 0DEADBEEFh
    jne fail_pattern
    mov eax, [fs:1020]
    cmp eax, 0CAFEBABEh
    jne fail_pattern

    ; -- 9. INT 10h AH=0F reflects via 32-bit shim to BIOS --------
    mov ah, 0Fh
    int 10h
    ; AL should be a mode number (any value; the fixture just needs the
    ; call not to fault).  A faulting reflection would abort the
    ; emulator here with "IRET:Illegal descriptor type 0x0".

    ; -- 10. AX=0503 grow to 2048 bytes ---------------------------
    mov eax, 0503h
    mov si, [h_si]
    mov di, [h_di]
    mov ebx, 0
    mov ecx, 2048
    int 31h
    jc  fail_grow

    ; -- 11. INT 21h AH=09 print via 32-bit IDT gate --------------
    mov ah, 9
    mov edx, ok_msg
    int 21h

    ; -- 12. AX=0502 free alloc ----------------------------------
    mov eax, 0502h
    mov si, [h_si]
    mov di, [h_di]
    int 31h
    jc  fail_free

    ; -- 13. AX=0001 free LDT ------------------------------------
    mov eax, 0001h
    mov bx, [ldt_sel]
    int 31h
    jc  fail_ldt_free

    ; -- 14. exit 0 ---------------------------------------------
    mov eax, 4C00h
    int 21h

; ---- Failure paths (all in 32-bit PM; use 32-bit INT 21h) ----
fail_ver:
    mov edx, fail_ver_msg
    jmp die32
fail_inc:
    mov edx, fail_inc_msg
    jmp die32
fail_ldt:
    mov edx, fail_ldt_msg
    jmp die32
fail_alloc:
    mov edx, fail_alloc_msg
    jmp die32
fail_setbase:
    mov edx, fail_setbase_msg
    jmp die32
fail_pattern:
    mov edx, fail_pattern_msg
    jmp die32
fail_grow:
    mov edx, fail_grow_msg
    jmp die32
fail_free:
    mov edx, fail_free_msg
    jmp die32
fail_ldt_free:
    mov edx, fail_ldt_free_msg
    jmp die32

die32:
    mov ah, 9
    int 21h
    mov eax, 4C01h
    int 21h

BITS 16
not_present:
    mov dx, absent_msg
    mov ah, 9
    int 21h
    mov ax, 4C02h
    int 21h

fail_no32:
    mov dx, fail_no32_msg
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
ldt_sel           dw 0
base_hi           dw 0
base_lo           dw 0
h_si              dw 0
h_di              dw 0

; Far pointer for the 32-bit `o32 jmp far [fptr_off]` out of the
; DPMI-entry 16-bit CS into our self-allocated 32-bit code selector.
; 4-byte offset + 2-byte selector (= m16:32).  The selector field is
; patched at runtime after AX=0000 gives us the slot.
fptr_off          dd pm32_entry
fptr_sel          dw 0

ok_msg              db 'dpmi-integration=ok', 13, 10, '$'
fail_ver_msg        db 'dpmi-integration=fail-ver', 13, 10, '$'
fail_inc_msg        db 'dpmi-integration=fail-inc', 13, 10, '$'
fail_ldt_msg        db 'dpmi-integration=fail-ldt', 13, 10, '$'
fail_alloc_msg      db 'dpmi-integration=fail-alloc', 13, 10, '$'
fail_setbase_msg    db 'dpmi-integration=fail-setbase', 13, 10, '$'
fail_pattern_msg    db 'dpmi-integration=fail-pattern', 13, 10, '$'
fail_grow_msg       db 'dpmi-integration=fail-grow', 13, 10, '$'
fail_free_msg       db 'dpmi-integration=fail-free', 13, 10, '$'
fail_ldt_free_msg   db 'dpmi-integration=fail-ldt-free', 13, 10, '$'
absent_msg          db 'dpmi-integration=absent', 13, 10, '$'
fail_no32_msg       db 'dpmi-integration=no32', 13, 10, '$'
failed_msg          db 'dpmi-integration=switch-failed', 13, 10, '$'

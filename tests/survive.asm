; survive.asm — calls an intentionally-unimplemented INT 21h AH and
; verifies execution continues past it, proving dosiz soft-fails unknown
; syscalls instead of killing the program.
;
; Assemble:  nasm -f bin survive.asm -o SURVIVE.COM

    org 100h

    mov ah, 0FDh               ; AH=FDh is reserved / unimplemented
    int 21h

    ; If we got here, the syscall returned (CF=1, AX=1) and the CPU kept
    ; running.  Print success and exit 0.
    mov ah, 09h
    mov dx, ok_msg
    int 21h

    mov ax, 4C00h
    int 21h

ok_msg db 'survived-unimpl', 13, 10, '$'

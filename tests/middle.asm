; middle.asm — intermediate link in the AH=4B grandchild chain.
; GRAND.COM spawns MIDDLE.COM, MIDDLE spawns CHILD.COM, CHILD exits
; with code 0x42.  MIDDLE verifies via AH=4D that it got 0x42,
; prints "middle=ok", exits with 0x21 so GRAND can verify the
; propagation across two nesting levels.
;
; Assemble:  nasm -f bin middle.asm -o MIDDLE.COM

    org 100h

    mov ax, cs
    mov word [pblock + 0], 0
    mov word [pblock + 2], cmdtail
    mov word [pblock + 4], ax
    mov word [pblock + 6], fcb1
    mov word [pblock + 8], ax
    mov word [pblock + 10], fcb2
    mov word [pblock + 12], ax

    mov es, ax
    mov bx, pblock
    mov dx, childname
    mov ax, 4B00h
    int 21h
    jc  fail

    ; AH=4D: read child's exit code
    mov ah, 4Dh
    int 21h
    cmp al, 42h
    jne fail_code

    mov ah, 9
    mov dx, ok_msg
    int 21h
    mov ax, 4C21h              ; exit AL=0x21
    int 21h

fail:
    mov dx, fail_msg
    jmp print_exit_1
fail_code:
    mov dx, fail_code_msg
    jmp print_exit_1
print_exit_1:
    mov ah, 9
    int 21h
    mov ax, 4C01h
    int 21h

childname     db 'CHILD.COM', 0
cmdtail       db 0, 13
fcb1          times 16 db 0
fcb2          times 16 db 0
pblock        times 14 db 0

ok_msg        db 'middle=ok', 13, 10, '$'
fail_msg      db 'middle=fail', 13, 10, '$'
fail_code_msg db 'middle=fail-code', 13, 10, '$'

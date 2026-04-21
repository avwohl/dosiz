; grand.asm — outer link in the AH=4B grandchild chain.
; GRAND spawns MIDDLE.COM which spawns CHILD.COM.  GRAND verifies
; MIDDLE returned 0x21 via AH=4D and prints "grand=ok".  Tests that
; our s_process_stack + nested DOSBOX_RunMachine works beyond one
; nesting level.
;
; Assemble:  nasm -f bin grand.asm -o GRAND.COM

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
    mov dx, middle_name
    mov ax, 4B00h
    int 21h
    jc  fail

    ; AH=4D: child = MIDDLE.  Should have exited with 0x21.
    mov ah, 4Dh
    int 21h
    cmp al, 21h
    jne fail_code

    mov ah, 9
    mov dx, ok_msg
    int 21h
    mov ax, 4C00h
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

middle_name   db 'MIDDLE.COM', 0
cmdtail       db 0, 13
fcb1          times 16 db 0
fcb2          times 16 db 0
pblock        times 14 db 0

ok_msg        db 'grand=ok', 13, 10, '$'
fail_msg      db 'grand=fail', 13, 10, '$'
fail_code_msg db 'grand=fail-code', 13, 10, '$'

; write.asm — DOS .COM that exercises INT 21h handle-based file I/O.
;
; Creates WROTE.TXT, writes "dosiz-wrote-ok" + CRLF, reads the command
; tail from PSP:80 (one arg expected), appends it if any, closes, exits 0.
;
; Assemble with:  nasm -f bin tests/write.asm -o tests/WRITE.COM

    org 100h

    ; AH=3Ch create file
    mov ah, 3Ch
    xor cx, cx
    mov dx, fname
    int 21h
    jc  bail
    mov bx, ax                ; BX = handle

    ; AH=40h write the greeting
    mov ah, 40h
    mov cx, msg_len
    mov dx, msg
    int 21h
    jc  bail

    ; AH=40h write the command tail (if any) -- PSP:80 = len, PSP:81 = string
    xor cx, cx
    mov cl, [80h]             ; command-tail length
    cmp cl, 0
    je  close_it
    mov dx, 81h               ; command-tail start
    mov ah, 40h
    int 21h

close_it:
    ; AH=3Eh close
    mov ah, 3Eh
    int 21h

    ; AH=4Ch exit 0
    mov ax, 4C00h
    int 21h

bail:
    mov ax, 4C01h
    int 21h

fname   db 'WROTE.TXT', 0
msg     db 'dosiz-wrote-ok', 13, 10
msg_len equ $ - msg

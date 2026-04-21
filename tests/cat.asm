; cat.asm — reads the file named in the command tail, writes it to stdout.
;
; Exercises: PSP command-tail parsing, AH=3Dh open, AH=3Fh read in a loop
; (including the text-mode LF->CRLF expansion), AH=40h write to stdout,
; AH=3Eh close, AH=4Ch exit.
;
; Assemble:  nasm -f bin cat.asm -o CAT.COM

    org 100h

    ; Skip leading space in PSP command tail (starts at PSP:81h after
    ; the length byte).
    mov si, 81h
skip_space:
    mov al, [si]
    cmp al, ' '
    jne found_name
    inc si
    jmp skip_space

found_name:
    mov bx, si
    mov di, si
find_end:
    mov al, [di]
    cmp al, 0Dh              ; DOS command-tail terminator
    je  got_end
    cmp al, 0
    je  got_end
    inc di
    jmp find_end
got_end:
    mov byte [di], 0         ; NUL-terminate in place
    cmp bx, di
    je  bail                 ; no argument

    mov dx, bx               ; DS:DX = filename

    ; AH=3Dh open read-only
    mov ah, 3Dh
    xor al, al
    int 21h
    jc  bail
    mov bx, ax               ; BX = handle

read_loop:
    ; AH=3Fh read up to 256 bytes
    mov ah, 3Fh
    mov cx, 256
    mov dx, buf
    int 21h
    jc  close_and_bail
    test ax, ax
    jz  close_and_exit       ; EOF

    ; AH=40h write AX bytes to stdout (handle 1)
    mov cx, ax
    push bx
    mov ah, 40h
    mov bx, 1
    mov dx, buf
    int 21h
    pop bx
    jmp read_loop

close_and_exit:
    mov ah, 3Eh
    int 21h
    mov ax, 4C00h
    int 21h

close_and_bail:
    mov ah, 3Eh
    int 21h
bail:
    mov ax, 4C01h
    int 21h

buf times 256 db 0

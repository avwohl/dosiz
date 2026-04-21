; listdir.asm — exercises AH=4Eh/4Fh findfirst/findnext.
;
; Globs *.TST, prints each matched filename on its own line, then exits.
; Filenames live in the default DTA at PSP:0080h starting at offset 0x1E
; (NUL-terminated ASCIIZ).
;
; Assemble:  nasm -f bin listdir.asm -o LISTDIR.COM

    org 100h

    mov ah, 4Eh
    xor cx, cx              ; attribute mask (normal files)
    mov dx, pat
    int 21h
    jc  done                ; no files -> just exit

print_loop:
    mov si, 80h + 1Eh       ; DTA filename field
write_loop:
    mov dl, [si]
    cmp dl, 0
    je  eol
    mov ah, 2
    int 21h
    inc si
    jmp write_loop
eol:
    mov dl, 10
    mov ah, 2
    int 21h

    mov ah, 4Fh
    int 21h
    jnc print_loop

done:
    mov ax, 4C00h
    int 21h

pat db '*.TST', 0

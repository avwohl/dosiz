; syscalls.asm — exercises AH=30h (DOS version), AH=48h (alloc), AH=35h
; (get int vector).  Prints human-readable diagnostic to stdout.
;
; Expected output (exact):
;   dosver=6.22
;   alloc=0x2000
;   int21-set-ok
;
; Assemble:  nasm -f bin syscalls.asm -o SYSCALLS.COM

    org 100h

    ; --- DOS version via AH=30h -----------------------------------------
    mov ah, 30h
    int 21h
    ; AL=major, AH=minor.  Print "dosver=<maj>.<mm>\r\n".
    push ax
    mov  dx, ver_prefix
    mov  ah, 9
    int  21h
    pop  ax
    push ax
    mov  dl, al
    add  dl, '0'            ; major digit
    mov  ah, 2
    int  21h
    mov  dl, '.'
    mov  ah, 2
    int  21h
    pop  ax                 ; AH still has minor
    mov  al, ah
    aam                     ; AH = tens, AL = units
    add  ax, 3030h
    push ax
    mov  dl, ah
    mov  ah, 2
    int  21h
    pop  ax
    mov  dl, al
    mov  ah, 2
    int  21h
    mov  dx, crlf
    mov  ah, 9
    int  21h

    ; --- Allocate 100h paragraphs via AH=48h ----------------------------
    mov  ah, 48h
    mov  bx, 100h
    int  21h
    jc   alloc_fail
    push ax                 ; allocated segment
    mov  dx, alloc_prefix
    mov  ah, 9
    int  21h
    pop  ax
    call print_hex_word
    mov  dx, crlf
    mov  ah, 9
    int  21h

    ; --- Install a fake INT 0x60 vector, read it back via AH=35h --------
    mov  ah, 25h
    mov  al, 60h
    mov  dx, 0BEEFh
    push ds
    push cs
    pop  ds                 ; DS := CS so the handler "seg" reads back as CS
    int  21h
    pop  ds
    mov  ah, 35h
    mov  al, 60h
    int  21h
    ; BX should equal 0xBEEF.  ES:BX should be set; we only check BX.
    cmp  bx, 0BEEFh
    jne  vec_fail
    mov  dx, vec_ok
    mov  ah, 9
    int  21h

    mov  ax, 4C00h
    int  21h

alloc_fail:
    mov  dx, alloc_fail_msg
    mov  ah, 9
    int  21h
    mov  ax, 4C01h
    int  21h

vec_fail:
    mov  dx, vec_fail_msg
    mov  ah, 9
    int  21h
    mov  ax, 4C02h
    int  21h

; --- Helpers ---------------------------------------------------------------

; print AX as 0xHHHH using AH=02h.
print_hex_word:
    push ax
    mov  dl, '0'
    mov  ah, 2
    int  21h
    mov  dl, 'x'
    mov  ah, 2
    int  21h
    pop  ax
    push ax
    mov  cl, 12
    call print_hex_nibble
    pop  ax
    push ax
    mov  cl, 8
    call print_hex_nibble
    pop  ax
    push ax
    mov  cl, 4
    call print_hex_nibble
    pop  ax
    mov  cl, 0
    call print_hex_nibble
    ret

print_hex_nibble:
    push ax
    shr  ax, cl
    and  al, 0Fh
    cmp  al, 10
    jb   .digit
    add  al, 'A' - 10 - '0'
.digit:
    add  al, '0'
    mov  dl, al
    mov  ah, 2
    int  21h
    pop  ax
    ret

ver_prefix      db 'dosver=', '$'
alloc_prefix    db 'alloc=', '$'
vec_ok          db 'int21-set-ok', 13, 10, '$'
alloc_fail_msg  db 'alloc-fail', 13, 10, '$'
vec_fail_msg    db 'vec-fail', 13, 10, '$'
crlf            db 13, 10, '$'

; vcpi_probe.asm -- probe for a VCPI (Virtual Control Program Interface)
; provider.  VCPI is the ring-0/V86 protected-mode bridge older DOS
; extenders used before DPMI was standardised.
;
; Checks:
;   AX=DE00  VCPI installation -- AH=0 on success, BH=version (BCD)
;   AX=DE03  Get number of free 4K pages -- AH=0, EDX=count
;
; Emits:
;   VCPI VER=X.Y PAGES=NNNN
;   vcpi-ok
; or a FAIL: line.
;
; Assemble:  nasm -f bin vcpi_probe.asm -o VCPI_PROBE.COM

    org 100h

    ; dosbox only answers VCPI-install from v86 mode or with the JEMM
    ; probe convention (CX=0, DI=0x0012), so set those.
    xor cx, cx
    mov di, 0012h
    mov ax, 0DE00h
    int 67h
    cmp ah, 0
    jne fail_install

    push bx                    ; version in BH

    mov dx, s_ver
    mov ah, 9
    int 21h

    pop ax                     ; ah=version bcd
    push ax
    mov dl, ah
    shr dl, 4
    add dl, '0'
    mov ah, 2
    int 21h
    mov dl, '.'
    mov ah, 2
    int 21h
    pop ax
    and ah, 0Fh
    mov dl, ah
    add dl, '0'
    mov ah, 2
    int 21h

    ; AX=DE03 free pages (EDX:32-bit count)
    mov ax, 0DE03h
    int 67h
    cmp ah, 0
    jne fail_pages
    mov dx, s_pages
    mov ah, 9
    int 21h
    mov ax, dx                 ; low 16 of EDX
    call print_hex16
    call newline

    mov dx, s_ok
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_install:
    mov dx, m_install
    jmp short fail_common
fail_pages:
    mov dx, m_pages
fail_common:
    mov ah, 9
    int 21h
    mov ax, 4C01h
    int 21h

print_hex16:
    push ax
    push bx
    push cx
    mov cx, 4
.loop:
    rol ax, 4
    mov bx, ax
    and bx, 0Fh
    mov dl, [bx+hex_tbl]
    push ax
    mov ah, 2
    int 21h
    pop ax
    loop .loop
    pop cx
    pop bx
    pop ax
    ret

newline:
    mov dl, 13
    mov ah, 2
    int 21h
    mov dl, 10
    mov ah, 2
    int 21h
    ret

hex_tbl   db '0123456789ABCDEF'
s_ver     db 'VCPI VER=$'
s_pages   db ' PAGES=$'
s_ok      db 'vcpi-ok',13,10,'$'
m_install db 'FAIL: AX=DE00 VCPI install',13,10,'$'
m_pages   db 'FAIL: AX=DE03 VCPI free pages',13,10,'$'

; ems_probe.asm -- probe for an EMS (LIM 3.2/4.0) provider.
;
; Checks in order:
;   1. INT 67h vector is nonzero
;   2. Can open EMMXXXX0 character device via AH=3D
;   3. AH=40 installation status returns AH=00
;   4. AH=46 version returns AH=00, AL=version (BCD)
;   5. AH=41 page-frame segment
;   6. AH=42 total/free 16K pages
;
; On success, prints:
;   INT67=SSSS:OOOO
;   VER=X.X
;   FRAME=SSSS
;   PAGES T=NNNN F=NNNN
;   ems-ok
;
; On any failure, prints the failing step and exits nonzero.
;
; Assemble:  nasm -f bin ems_probe.asm -o EMS_PROBE.COM

    org 100h

    ; ---- (1) INT 67h vector at 0000:019Ch (= 67h*4) ----
    xor ax, ax
    mov es, ax
    mov ax, [es:67h*4]        ; offset
    mov bx, [es:67h*4 + 2]    ; segment
    or  ax, bx
    jz  fail_novec

    ; print "INT67="
    mov dx, s_int67
    mov ah, 9
    int 21h

    ; segment
    mov ax, [es:67h*4 + 2]
    call print_hex16
    mov dl, ':'
    mov ah, 2
    int 21h
    ; offset
    mov ax, [es:67h*4]
    call print_hex16
    call newline

    ; ---- (2) Open EMMXXXX0 via AH=3D (our INT 21 routes char-device
    ;     names to a /dev/null-backed pseudo-handle) ----
    mov dx, emm_name
    mov ax, 3D00h
    int 21h
    jc  fail_device
    mov bx, ax
    mov ah, 3Eh
    int 21h

    ; ---- (3) AH=40 install check ----
    mov ah, 40h
    int 67h
    cmp ah, 0
    jne fail_install

    ; ---- (4) AH=46 version ----
    mov ah, 46h
    int 67h
    cmp ah, 0
    jne fail_version
    ; print "VER=X.Y"
    push ax
    mov dx, s_ver
    mov ah, 9
    int 21h
    pop ax
    push ax
    mov dl, al
    shr dl, 4
    add dl, '0'
    mov ah, 2
    int 21h
    mov dl, '.'
    mov ah, 2
    int 21h
    pop ax
    and al, 0Fh
    mov dl, al
    add dl, '0'
    mov ah, 2
    int 21h
    call newline

    ; ---- (5) AH=41 page frame ----
    mov ah, 41h
    int 67h
    cmp ah, 0
    jne fail_frame
    push bx
    mov dx, s_frame
    mov ah, 9
    int 21h
    pop ax                     ; bx was the seg
    call print_hex16
    call newline

    ; ---- (6) AH=42 pages ----
    mov ah, 42h
    int 67h
    cmp ah, 0
    jne fail_pages
    push bx
    push dx
    mov dx, s_pages
    mov ah, 9
    int 21h
    pop ax                     ; total (dx)
    call print_hex16
    mov dx, s_f
    mov ah, 9
    int 21h
    pop ax                     ; free (bx)
    call print_hex16
    call newline

    ; success
    mov dx, s_ok
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

; ---- failure tails ----
fail_novec:
    mov dx, m_novec
    jmp fail_common
fail_device:
    mov dx, m_device
    jmp short fail_common
fail_install:
    mov dx, m_install
    jmp short fail_common
fail_version:
    mov dx, m_ver
    jmp short fail_common
fail_frame:
    mov dx, m_frame
    jmp short fail_common
fail_pages:
    mov dx, m_pages
fail_common:
    mov ah, 9
    int 21h
    mov ax, 4C01h
    int 21h

; ---- helpers ----
; print_hex16: AX -> stdout as 4 hex digits
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

; ---- data ----
hex_tbl   db '0123456789ABCDEF'
emm_name  db 'EMMXXXX0',0
s_int67   db 'INT67=$'
s_ver     db 'VER=$'
s_frame   db 'FRAME=$'
s_pages   db 'PAGES T=$'
s_f       db ' F=$'
s_ok      db 'ems-ok',13,10,'$'
m_novec   db 'FAIL: INT67 vec=0',13,10,'$'
m_device  db 'FAIL: EMMXXXX0 open',13,10,'$'
m_install db 'FAIL: AH=40 install',13,10,'$'
m_ver     db 'FAIL: AH=46 version',13,10,'$'
m_frame   db 'FAIL: AH=41 frame',13,10,'$'
m_pages   db 'FAIL: AH=42 pages',13,10,'$'

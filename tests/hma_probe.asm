; hma_probe.asm -- probe XMS HMA and A20 helpers.
;
; Steps:
;   INT 2F AX=4300  -- XMS install (expect AL=80h)
;   INT 2F AX=4310  -- get XMS entry point -> ES:BX
;   XMS AH=00       -- version; expect DX=1 (HMA exists)
;   XMS AH=07       -- A20 state; expect AX=1 (on)
;   XMS AH=01       -- Request HMA; expect AX=1
;   XMS AH=01 again -- expect AX=0, BL=92h (already in use)
;   XMS AH=02       -- Release HMA; expect AX=1
;   XMS AH=05       -- Local Enable A20; expect AX=1
;
; On success prints "hma-ok"; on any failure prints a FAIL: line.
;
; XMS entry is "CALL FAR" convention ending in RETF -- no flag push.
;
; Assemble:  nasm -f bin hma_probe.asm -o HMA_PROBE.COM

    org 100h

    ; XMS install check
    mov ax, 4300h
    int 2Fh
    cmp al, 80h
    jne fail_install

    ; Get driver entry -> ES:BX
    mov ax, 4310h
    int 2Fh
    mov [entry_off], bx
    mov [entry_seg], es

    ; XMS AH=00 -- expect DX = 1
    mov ah, 00h
    call far [entry_off]
    cmp dx, 1
    jne fail_nohma

    ; AH=03 -- Global Enable A20
    mov ah, 03h
    call far [entry_off]
    cmp ax, 1
    jne fail_a20en

    ; AH=07 -- A20 state after enable, expect AX = 1
    mov ah, 07h
    call far [entry_off]
    cmp ax, 1
    jne fail_a20

    ; AH=01 -- Request HMA
    mov ah, 01h
    mov dx, 0FFFFh
    call far [entry_off]
    cmp ax, 1
    jne fail_req

    ; AH=01 again -- expect AX=0, BL=92h
    mov ah, 01h
    mov dx, 0FFFFh
    call far [entry_off]
    cmp ax, 0
    jne fail_reqx
    cmp bl, 92h
    jne fail_reqx

    ; AH=02 -- Release
    mov ah, 02h
    call far [entry_off]
    cmp ax, 1
    jne fail_rel

    ; AH=05 -- Local Enable A20
    mov ah, 05h
    call far [entry_off]
    cmp ax, 1
    jne fail_a20en

    mov dx, s_ok
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_install:
    mov dx, m_install
    jmp fail_common
fail_nohma:
    mov dx, m_nohma
    jmp fail_common
fail_a20:
    mov dx, m_a20
    jmp fail_common
fail_req:
    mov dx, m_req
    jmp fail_common
fail_reqx:
    mov dx, m_reqx
    jmp fail_common
fail_rel:
    mov dx, m_rel
    jmp fail_common
fail_a20en:
    mov dx, m_a20en
fail_common:
    mov ah, 9
    int 21h
    mov ax, 4C01h
    int 21h

entry_off dw 0
entry_seg dw 0

s_ok       db 'hma-ok',13,10,'$'
m_install  db 'FAIL: XMS install',13,10,'$'
m_nohma    db 'FAIL: no HMA (DX!=1)',13,10,'$'
m_a20      db 'FAIL: A20 not on',13,10,'$'
m_req      db 'FAIL: request HMA',13,10,'$'
m_reqx     db 'FAIL: second request should fail 92h',13,10,'$'
m_rel      db 'FAIL: release HMA',13,10,'$'
m_a20en    db 'FAIL: local A20 enable',13,10,'$'

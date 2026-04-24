; dpmi_int31.asm — verifies the INT 31h default-deny path.
;
; Picks an unimplemented DPMI function (AX=FF00h, reserved/unknown)
; so the default case of dosiz_int31 still returns CF=1 / AX=8001h
; even after stage 4 landed real handlers for AX=0400/0006/0007.
; Without the installed stub the call would dispatch through an
; un-installed IVT entry and the CPU would fault.
;
; Prints "int31=denied" on CF=1/AX=8001h, "int31=handled" on anything
; else (which would surprise us: nothing should be handling AX=FF00).
;
; Assemble:  nasm -f bin dpmi_int31.asm -o DPMI_INT31.COM

    org 100h

    mov ax, 0FF00h           ; reserved / unknown DPMI function
    int 31h
    jnc  not_denied
    cmp ax, 8001h
    jne not_denied

    mov ah, 9
    mov dx, denied_msg
    int 21h
    mov ax, 4C00h
    int 21h

not_denied:
    mov ah, 9
    mov dx, handled_msg
    int 21h
    mov ax, 4C01h
    int 21h

denied_msg  db 'int31=denied', 13, 10, '$'
handled_msg db 'int31=handled', 13, 10, '$'

; dpmi_probe.asm — classifies current DPMI status.  Per the DPMI 0.9
; spec: INT 2Fh AX=1687h; on return, AX=0 means DPMI is present, AX!=0
; means absent.
;
; dosemu currently reports "absent" -- this fixture locks that in as a
; regression test.  When a DPMI host is added in a future session, this
; fixture will flip to reporting "present" and additional probes will
; check the returned entry point + flags.
;
; Assemble:  nasm -f bin dpmi_probe.asm -o DPMI_PROBE.COM

    org 100h

    mov ax, 1687h
    int 2Fh
    test ax, ax
    jz  present

    mov dx, absent_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

present:
    mov dx, present_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

absent_msg  db 'dpmi=absent', 13, 10, '$'
present_msg db 'dpmi=present', 13, 10, '$'

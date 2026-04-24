; hello.asm — minimal DOS .COM that prints a string and exits.
;
; Assemble with:  nasm -f bin hello.asm -o HELLO.COM
;
; Used by tests/run_tests.sh to verify dosiz can actually run a
; hand-written DOS binary end-to-end.

    org 100h
    mov ah, 9
    mov dx, msg
    int 21h
    mov ax, 4C00h
    int 21h

msg db 'dosiz-hello-ok', 13, 10, '$'

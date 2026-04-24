; hello_exe.asm — minimal MZ .EXE that prints a string and exits.
;
; Two NASM sections:
;   .mzhdr  — 32-byte MZ header at file offset 0
;   .image  — code+data; labels are relative to image start (vstart=0) so
;             that `mov dx, msg` resolves to the offset within LOAD_SEG
;             after the loader strips the header.
;
; Assemble:  nasm -f bin hello_exe.asm -o HELLO.EXE

bits 16

section .mzhdr vstart=0
    db 'MZ'
    dw 68           ; 0x02: bytes in last 512-page = total image size (mod 512)
    dw 1            ; 0x04: total 512-pages (ceil(total / 512))
    dw 0            ; 0x06: reloc count
    dw 2            ; 0x08: header paragraphs (2 * 16 = 32 bytes)
    dw 10h          ; 0x0A: min extra paragraphs
    dw 0FFFFh       ; 0x0C: max extra paragraphs
    dw 0            ; 0x0E: initial SS (relative)
    dw 0F000h       ; 0x10: initial SP
    dw 0            ; 0x12: checksum
    dw 0            ; 0x14: initial IP
    dw 0            ; 0x16: initial CS (relative)
    dw 1Ch          ; 0x18: reloc-table offset
    dw 0            ; 0x1A: overlay
    dw 0, 0         ; 0x1C-0x1F: pad to 32-byte header end

section .image vstart=0 follows=.mzhdr
    push cs
    pop  ds
    mov  ah, 09h
    mov  dx, msg
    int  21h
    mov  ax, 4C00h
    int  21h
msg db 'dosiz-hello-exe-ok', 13, 10, '$'

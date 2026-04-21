; spawn.asm — exercises INT 21h AH=4Bh "Load and Execute Program".
;
; Flow:
;   1. Build a parameter block in-memory (env = parent's env, empty
;      command tail, no FCBs).
;   2. AH=4Bh AL=0, DS:DX = program name "C:\CHILD.COM", ES:BX =
;      parameter block.
;   3. On return: check CF=0, verify AL = 0x42 (child's exit code
;      forwarded by our handler).
;   4. Print "spawn=ok", exit 0.
;
; Requires CHILD.COM reachable via the DOS drive C: -- CI wires a
; tmpdir where both parent and child live.
;
; Assemble:  nasm -f bin spawn.asm -o SPAWN.COM

    org 100h

    ; Shrink our own PSP block to 4KB via AH=4A so AH=4B below has
    ; memory available (DOS gives us the whole free arena; we need
    ; to release most of it).
    mov ah, 4Ah
    mov bx, 100h
    push es
    mov ax, cs
    mov es, ax
    int 21h
    pop es

    ; Parameter block at `pblock` below.  env_seg=0 means "inherit
    ; parent's".  Command tail is a 4-byte far pointer to cmdtail
    ; which must start with a length byte followed by the tail, then
    ; 0x0D.
    mov ax, cs
    mov word [pblock + 0], 0            ; env_seg (0 = inherit)
    mov word [pblock + 2], cmdtail      ; cmdtail offset
    mov word [pblock + 4], ax           ; cmdtail seg
    mov word [pblock + 6], fcb1         ; FCB1 offset (unused)
    mov word [pblock + 8], ax
    mov word [pblock + 10], fcb2
    mov word [pblock + 12], ax

    mov es, ax
    mov bx, pblock
    mov dx, childname
    mov ax, 4B00h                       ; AL=0: load + execute
    int 21h
    jc  fail_spawn

    ; Child's exit code in AL (our AH=4B handler stashes it there).
    cmp al, 42h
    jne fail_exit

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

fail_spawn:
    mov dx, fail_spawn_msg
    jmp print_and_exit_1
fail_exit:
    mov dx, fail_exit_msg
    jmp print_and_exit_1

print_and_exit_1:
    mov ah, 9
    int 21h
    mov ax, 4C01h
    int 21h

childname       db 'CHILD.COM', 0
cmdtail         db 0, 13                ; empty command tail
fcb1            times 16 db 0
fcb2            times 16 db 0
pblock          times 14 db 0           ; 14-byte parameter block

ok_msg          db 'spawn=ok', 13, 10, '$'
fail_spawn_msg  db 'spawn=fail-spawn', 13, 10, '$'
fail_exit_msg   db 'spawn=fail-exit', 13, 10, '$'

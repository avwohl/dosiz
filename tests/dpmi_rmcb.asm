; dpmi_rmcb.asm — AX=0303 "Allocate Real Mode Callback Address" +
; AX=0304 "Free Real Mode Callback Address".
;
; Full round-trip: PM client registers a PM callback, then invokes it
; by calling a RM trampoline (via AX=0301) whose body does a FAR CALL
; to the returned RM callback address.  When the FAR CALL fires, the
; host switches to PM, the PM callback writes 0xABCD into the struct's
; EAX field, RETFs back to the host's stop callback, and the host
; switches back to RM; rm_proc's own RETF returns to AX=0301's stop.
; AX=0301 completes and the fixture verifies struct.EAX = 0xABCD.
;
; Assemble:  nasm -f bin dpmi_rmcb.asm -o DPMI_RMCB.COM

    org 100h

BITS 16
    mov ax, 1687h
    int 2Fh
    test ax, ax
    jnz  not_present
    mov [entry_off], di
    mov [entry_seg], es

    ; Capture our real-mode PSP segment -- needed to populate struct's
    ; RM CS/DS/ES when AX=0301 enters RM to call rm_proc.
    mov ax, ds
    mov [saved_ds_rm], ax

    cli
    xor ax, ax
    call far [entry_off]
    test ax, ax
    jnz  switch_failed

    ; Zero the struct
    mov cx, 50
    mov di, rmcs
    xor ax, ax
    rep stosb

    ; AX=0303 register PM callback pm_cb, pointing at rmcs for the
    ; struct.  DS:SI = pm_cb, ES:DI = rmcs (both in our data segment,
    ; which in PM is selector 0x10 base = saved_ds_rm * 16).
    mov ax, ds
    mov es, ax
    mov si, pm_cb
    mov di, rmcs
    mov ax, 0303h
    int 31h
    jc  fail_alloc
    mov [rm_cb_off], dx
    mov [rm_cb_seg], cx

    ; Populate struct CS:IP = rm_proc in our real-mode segment; DS/ES
    ; = saved_ds_rm so rm_proc can read its own data (the indirect
    ; far-call target at rm_cb_off/seg).
    mov ax, [saved_ds_rm]
    mov [rmcs + 0x24], ax           ; DS
    mov [rmcs + 0x22], ax           ; ES
    mov [rmcs + 0x2C], ax           ; CS
    mov word [rmcs + 0x2A], rm_proc ; IP

    ; AX=0301: enter RM, run rm_proc (which FAR-CALLs our RM callback
    ; address), return.  The PM callback fires during rm_proc's FAR
    ; CALL and mutates the struct.
    mov ax, 0301h
    mov bh, 0
    xor cx, cx
    mov di, rmcs
    int 31h
    jc  fail_call

    ; Verify the PM callback mutated struct.EAX to 0xABCD
    mov ax, [rmcs + 0x1C]
    cmp ax, 0ABCDh
    jne fail_verify

    ; AX=0304 free the callback
    mov cx, [rm_cb_seg]
    mov dx, [rm_cb_off]
    mov ax, 0304h
    int 31h
    jc  fail_free

    mov dx, ok_msg
    mov ah, 9
    int 21h
    mov ax, 4C00h
    int 21h

; ---- PM callback procedure: writes 0xABCD to struct.EAX, RETFs ----
; Called in PM with ES:DI pointing at the struct.  The host entered PM
; via manual CS cache refresh + CPU_SetSegGeneral; all regs come from
; our 0303 handler.
pm_cb:
    mov word [es:di + 0x1C], 0ABCDh
    mov word [es:di + 0x1E], 0
    retf

; ---- RM trampoline: called in RM by AX=0301.  FAR-CALLs the
; 0303-returned RM callback address.  The callback's own RETF returns
; control here; our RETF then returns to the stop callback AX=0301
; pushed.
rm_proc:
    call far [rm_cb_off]
    retf

fail_alloc:
    mov dx, fail_alloc_msg
    jmp p1
fail_call:
    mov dx, fail_call_msg
    jmp p1
fail_verify:
    mov dx, fail_verify_msg
    jmp p1
fail_free:
    mov dx, fail_free_msg
    jmp p1

p1:
    mov ah, 9
    int 21h
    mov ax, 4C01h
    int 21h

not_present:
    mov dx, absent_msg
    mov ah, 9
    int 21h
    mov ax, 4C02h
    int 21h

switch_failed:
    mov dx, failed_msg
    mov ah, 9
    int 21h
    mov ax, 4C01h
    int 21h

entry_off        dw 0
entry_seg        dw 0
saved_ds_rm      dw 0
rm_cb_off        dw 0
rm_cb_seg        dw 0

rmcs             times 50 db 0

ok_msg           db 'dpmi-rmcb=ok', 13, 10, '$'
fail_alloc_msg   db 'dpmi-rmcb=fail-alloc', 13, 10, '$'
fail_call_msg    db 'dpmi-rmcb=fail-call', 13, 10, '$'
fail_verify_msg  db 'dpmi-rmcb=fail-verify', 13, 10, '$'
fail_free_msg    db 'dpmi-rmcb=fail-free', 13, 10, '$'
absent_msg       db 'dpmi-rmcb=absent', 13, 10, '$'
failed_msg       db 'dpmi-rmcb=switch-failed', 13, 10, '$'

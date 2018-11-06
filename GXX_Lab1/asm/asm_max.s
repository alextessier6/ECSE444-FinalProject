	AREA test, CODE, READONLY
	
	export asm_max                  ; label must be exported if it is to be used as a function in C
asm_max

	PUSH{R4, R5}                    ; saving context according to calling convention
	
	VLDR.f32 S0, [R0]               ; max = f[0] (register S0 used for max)
	MOV R4, #0                      ; max_index = 0 (register R4 used for max_idx)
loop
	SUBS R1, R1, #1                 ; loop counter (N = N-1)
	BLT done                        ; loop has finished?
	ADD R5, R0, R1, LSL #2          ; creating base address for the next element in the array
	VLDR.f32 S1, [R5]               ; load next element into S1
	VCMP.f32 S0, S1                 ; compare element with current max
	VMRS APSR_nzcv, FPSCR           ; need to move the FPU status register to achive floating point conditional execution
	BGT continue                    ; if the current max is greater or equal to new element, no need to update, continue
	VMOV.f32 S0, S1                 ; if not, then update current max
	MOV R4, R1                      ; also update max index
	
continue
	B loop                          ; loop
	
done
	VSTR.f32 S0, [R2]               ; store the max value in the pointer (float *max) that was provided
	STR R4, [R3]                    ; store the index of the max value in the pointer (int *max_idx) that was provided
	
	POP{R4, R5}                     ; restore context
	BX LR                           ; return
	
	END

	AREA test, CODE, READONLY
	
	export asm_var
asm_var
	
	VPUSH 			{S3-S6}
	PUSH				{R0-R1}
	
LOOP_AVG
	VLDR				S3, [R0], #4
	VADD.f32		S4, S4, S3
	SUBS 				R1, #1
	BLE					LOOP_AVG_END
	B						LOOP_AVG

LOOP_AVG_END
	POP   			{R0-R1}
	VMOV				S1, R1
	VCVT.f32.s32 		S1,S1
	VDIV.f32		S4, S4, S1
	PUSH				{R0-R1}

LOOP_VAR
	VLDR				S3, [R0], #4				;	S3 = Ai
	VSUB.f32		S5, S3, S4					; S4 = average 
	VMUL.f32		S5, S5, S5		
	VADD.f32		S6, S6, S5
	SUBS 				R1, #1
	BLE					LOOP_VAR_END
	B						LOOP_VAR
	
LOOP_VAR_END
	POP   			{R0-R1}
	VMOV.f32		S5, #1
	VSUB.f32		S1, S5
	VDIV.f32		S6, S6, S1	
	VSTR				S6, [R2]
	VPOP 				{S3-S6}
	BX 					LR

	END
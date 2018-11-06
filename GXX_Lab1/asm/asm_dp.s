	AREA test, CODE, READONLY
	
	export asm_dp
asm_dp
	
	VPUSH 	{S4-S6}
	
LOOP
	VLDR		S4, [R0], #4
	VLDR		S5, [R1], #4
	VMLA.f32 S6, S4, S5
	SUBS 		R2, #1
	BLE			LOOP_END
	B				LOOP

LOOP_END
	VSTR		S6, [R3]
	VPOP 		{S4-S6}
	BX 			LR

	END
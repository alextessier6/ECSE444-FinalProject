	AREA test, CODE, READONLY
	export add_asm
add_asm

	VADD.f32 S0, S1, S0
	BX LR
	
	END

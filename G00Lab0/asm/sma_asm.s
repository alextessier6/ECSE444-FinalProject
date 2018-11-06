	AREA test, CODE, READONLY
	export sma_asm

sma_asm
	MOV					R9, #4
	MUL			    R2, R2, R9 
  ADD		    	R0, R0, R2
	MOV					R6, #2
  MUL     		R6, R3, R6  
  SUB    			R0, R0, R6  
  MOV         R7, #0
	MOV					R8, #1
LOOP
	STR         R4,[R0]
  VADD.f32   	S5, S4, S5
	ADD					R0, R0, R9
  ADD    			R7, R8
  CMP         R7, R3
  BGE         DIV_LOOP
  B           LOOP
			
DIV_LOOP
	ASR					R3, #1
	ASR					R5, #1
	CMP					R3, #1
	BLE					DIV_LOOP_DONE
	B						DIV_LOOP

DIV_LOOP_DONE
	STR         R5,[R1]
	BX          LR 

	END
// S0 = POINTRS S1= RETURN POINTER S2 = N S3 = D,

// S4 = POINTED VALUE, S5 = SUM
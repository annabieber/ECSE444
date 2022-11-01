	AREA test, CODE, READONLY
	
	export asm_variance                  ; label must be exported if it is to be used as a function in C

asm_variance

	PUSH{R4, R5}                    ; saving context according to calling convention
	VLDR.f32 S0, [R0]	
	LDR R5, [R1]

meanLoop

	SUBS R1, R1, #1                 ; loop counter (N = N-1)
	BLT temp                       ; loop has finished?
	ADD R4, R0, R2, LSL #2          ; creating base address for the next element in the array
	VLDR.f32 S1, [R4]               ; load next element into S0
	VADD.f32 S0, S0, S1				;add the new multiplication to the sum
	B meanLoop							;loop again 


temp

	;VDIV.f32 S0, S0, R2
	VMOV.f32 S2, S0
	LDR R5, [R2]
	B varianceLoop
	
varianceLoop
	SUBS R1, R1, #1                 ; loop counter (N = N-1)
	BLT done                       ; loop has finished?
	ADD R4, R0, R2, LSL #2          ; creating base address for the next element in the array
	VLDR.f32 S1, [R4]               ; load next element into S0
	VADD.f32 S0, S0, S1				;add the new multiplication to the sum
	B varianceLoop							;loop again 




done
	
	;VDIV.f32 S0, S0, R2				;divide the 
	VSTR.f32 S3, [R3]				;store the result in the provided register
	POP{R4, R5}
	BX LR
	
	END

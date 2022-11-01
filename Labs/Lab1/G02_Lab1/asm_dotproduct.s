	AREA test, CODE, READONLY
	
	export asm_dotproduct                  ; label must be exported if it is to be used as a function in C
		
asm_dotproduct

	PUSH{R4, R5}                    ; saving context according to calling convention

loop

	SUBS R2, R2, #1
	BLT done
	ADD R4, R0, R2, LSL #2          ; creating base address for the next element in the array
	ADD R5, R1, R2, LSL #2          ; creating base address for the next element in the array
	VLDR.f32 S0, [R4]               ; load next element into S0
	VLDR.f32 S1, [R5]               ; load next element into S1
	VMLA.f32 S2, S0, S1				; multiply a and b
	B loop							;loop again 
		
done

	VSTR.f32 S2, [R3]				;store the result in the provided register
	POP{R4, R5}
	BX LR
	
	END

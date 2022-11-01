#include <stdio.h>
#include <add.h>

int main()
{
	float *originalSignal;
	float *resultSignal;	
	float samples;
	float filter;	
	
	add_c(originalSignal, resultSignal, samples, filter);
	add_asm(originalSignal, resultSignal, samples, filter);
	
//	printf("C subroutine sum = %f\n",);
//	printf("Assembly subroutine sum = %f\n", a);
	
	return 0;
}
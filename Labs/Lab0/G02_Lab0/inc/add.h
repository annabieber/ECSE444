#ifndef _ADD_H
#define _ADD_H

	extern void add_asm(float *originalSignal, float *resultSignal, float samples, float filter);
	void add_c(float *originalSignal, float *resultSignal, float samples, float filter);
	
#endif
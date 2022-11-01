#include <add.h>

void add_c(float *originalSignal, float *resultSignal, float samples, float filterDepth)
{
	float temp;
	int tempFilterDepth = filterDepth /2;
	originalSignal -= tempFilterDepth;
	
	for(int i = 0; i < filterDepth ; i++) 
	{
		if(originalSignal < 0 || originalSignal > samples) //check if address is negative or not
		{
			temp += 0;
		}
		else
		{
			temp += *originalSignal;
		}
		originalSignal++;
	}
	temp = temp/filterDepth;
	resultSignal = &temp;
}

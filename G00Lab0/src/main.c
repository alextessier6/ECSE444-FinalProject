#include <stdio.h>
#include <add.h>
#include <sma.h>

int main(){
	/*
	float n1 = 1.1;
	float n2 = 2.2;
	
	float c = add_c(n1,n2);
	float a = add_asm(n1, n2);
	
	printf("C subroutine sum = %f\n", c);
	printf("Assesmbly subroutine sum = %f\n", a);
	*/
	
	float samples[11] = {1.1,1.2,1.3,1.4,1.5,1.7,1.1,1.2,1.8,1.1,1.0};
	float* samp_p = samples;
	float* result;
	sma_c(samp_p, result, 5, 4);
	sma_asm(samp_p, result, 5, 4);
	printf("%f\n",*result);
	
}
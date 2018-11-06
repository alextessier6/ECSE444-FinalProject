#include <stdio.h>
#include <add.h>
#include <sma.h>

void sma_c(float* samples, float* result, int n, int depth){
	int i = 0;
	float temp;
	samples = samples + n - (int)(depth/2);
	for( i = 0; i < depth; i++){
		temp = add_c(temp,*(samples));
		samples += 1;
		printf("sample at %d: %f, temp: %f\n",samples,*samples, temp);
	}
	*result = temp/depth;
}

	
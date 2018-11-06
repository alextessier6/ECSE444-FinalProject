#ifndef _SMA_H
#define _SMA_H
	
	extern void sma_asm(float* samples, float* result, int n, int depth);
	void sma_c(float* samples, float* result, int n, int depth);
	
#endif
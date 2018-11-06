#include <stdio.h>
#include <arm_math.h>
#include <asm_max.h>

float f10_array[10] = {48.21, 79.48, 24.27, 28.82, 78.24, 88.49, 31.19, 5.52, 82.70, 77.73};
float A[10] = {48.21, 79.48, 24.27, 28.82, 78.24, 88.49, 31.19, 5.52, 82.70, 77.73};
flaot B[10] = {48.21, 79.48, 24.27, 28.82, 78.24, 88.49, 31.19, 5.52, 82.70, 77.73};

int main(){
	
//	float max;
//	int max_idx;
//	
//	float a_max;
//	int a_max_idx;
//	
//	float DSP_max;
//	unit32_t DSP_max_idx;
//	
//	int i;
//	
//	/* LOOP TO FIND MAX*/
//	max = f10_array[0];
//	max_idx = 0;
//	for(i = 0; i< 10; i++){
//		if(f10_array[i] > max){
//			max = f10_array[i];
//			max_idx = i;
//		}
//	}
//	/* Assembly max functions */
//	asm_max(f10_array, 10, &a_max, &a_max_idx);
//	
//	/* CMSIS-DSP max function */
//	arm_max_f32(f10_array, 10, &DSP_max, &DSP_max_idx);
//	
//	printf("Pure C	: Max element f[%d] = %f\n", max_idx, max);
//	printf("Assembly: Max element f[%d] = %f\n", a_max_idx, a_max);
//	printf("C: DSP	: Max element f[%d] = %f\n", DSP_max_idx, DSP_max);
	
	/* -------------------DOT PRODUCT -------------------*/
	float DP = 0;
	int i;
	
	float DSP_dp;
	float DSP_product;
	
	/* C Implementation */
	for(i = 0; i < 10 ; i++){
		DP += A[i]*B[i];
	}	
	
	/* CMSIS-DSP max function */
	arm_mult_f32(	A, B, &DSP_product, 10);	
	for (i = 0; i < 10; i++){
		arm_add_f32	(	&DSP_dp, m&DSP_product[i], &DSP_dp, 1);
	}	
	
	printf("Pure C	: Dot Product = %f\n", DP);
	printf("Assembly: Dot Product = %f\n", DSP_dp);
	printf("C: DSP	: Dot Product = %f\n", DSP_dp);
	
	/*---------------------VARIANCE-----------------------*/
	float average = 0;
	float variance = 0;
	float sum = 0;
	int i;
	
	float DSP_var;
	
	/* C Implementation */
	for(i = 0; i < 10; i++){
		average+=f10_array[i];
	}
	average/=10.f;
	for(i = 0; i < 10; i++){
		sum+= (f10_array[i]-average)^2;
	}
	sum/=10.f;
	
	/* CMSIS-DSP max function */
	arm_var_f32 (f10_array, 10, &DSP_var);
	
	printf("Pure C	: Variance = %f\n", variance);
	printf("Assembly: Variance = %f\n", a_max_idx, a_max);
	printf("C: DSP	: Variance = %f\n", DSP_var);
	
	return 0;
}
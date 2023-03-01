/*
 * MatrixCalculate.c
 *
 *  Created on: 2021¦~3¤ë19¤é
 *      Author: ldsc8
 */
#include "MatrixCalculate.h"
#include <stdio.h>

void mattimes(float *matrix_a,int row_a, int col_a, float *matrix_b, int row_b, int col_b, float *matrix_ans){
	if(col_a != row_b){
		printf("matrix dimension error\r\n");
	}
	for(int uu = 0; uu < row_a*col_b; uu++){
		matrix_ans[uu] = 0;
	}

	int num = 0;
	for(int i=0 ; i<row_a ; i++){
		for(int j=0 ; j<col_b ; j++){
			for(int k=0 ; k<col_a ; k++){
				matrix_ans[num] = matrix_ans[num] + matrix_a[k+i*col_a]*matrix_b[k*col_b+j];
			}
			num++;
		}
	}
}

void transpose(float *mat, int row, int col, float *mat_ans){
//	for(int uu = 0; uu < row*col; uu++){
//		printf("%d\r\n",(int)(mat[uu]*10));
//	}
	for(int i=0; i<row ;i++){
		for(int j=0; j<col ;j++){
			mat_ans[j*row+i] = mat[i*col+j];
		}
	}
}

void matplus(float *mat_a, int row, int col, float *mat_b, float *mat_ans){
	for(int i=0; i<row ;i++){
		for(int j=0; j<col ;j++){
			mat_ans[i*col+j] = mat_a[i*col+j] + mat_b[i*col+j];
		}
	}
}

void inv3(float *S, float *mat_ans){	//just for 3*3 matrix
	float det1 = S[0]*S[4]*S[8]+S[1]*S[5]*S[6]+S[2]*S[3]*S[7];
	float det2 = S[2]*S[4]*S[6]+S[0]*S[5]*S[7]+S[1]*S[3]*S[8];
	float det = det1 - det2;

	if(det == 0){
		printf("matrix is singular\r\n");
	}
	mat_ans[0] = (S[4]*S[8]-S[5]*S[7])/det;
	mat_ans[1] = -(S[1]*S[8]-S[2]*S[7])/det;
	mat_ans[2] = (S[1]*S[5]-S[2]*S[4])/det;
	mat_ans[3] = -(S[3]*S[8]-S[5]*S[6])/det;
	mat_ans[4] = (S[0]*S[8]-S[2]*S[6])/det;
	mat_ans[5] = -(S[0]*S[5]-S[2]*S[3])/det;
	mat_ans[6] = (S[3]*S[7]-S[4]*S[6])/det;
	mat_ans[7] = -(S[0]*S[7]-S[1]*S[6])/det;
	mat_ans[8] = (S[0]*S[4]-S[1]*S[3])/det;
}


/*
 * MatrixCalculate.h
 *
 *  Created on: 2021¦~3¤ë19¤é
 *      Author: ldsc8
 */

#ifndef INC_MATRIXCALCULATE_H_
#define INC_MATRIXCALCULATE_H_


void mattimes(float* matrix_a, int row_a, int col_a, float* matrix_b, int row_b, int col_b, float* matrix_ans);
void transpose(float* mat, int row, int col, float* mat_ans);
void matplus(float* mat_a, int row, int col, float* mat_b, float* mat_ans);
void inv3(float *S, float *mat_ans);
#endif /* INC_MATRIXCALCULATE_H_ */

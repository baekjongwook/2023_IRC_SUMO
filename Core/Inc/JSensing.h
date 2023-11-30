/*
 * JSensing.h
 *
 *  Created on: Oct 5, 2023
 *      Author: Jongwook Baek
 */

#ifndef INC_JSENSING_H_
#define INC_JSENSING_H_

#include <stdlib.h>
#include "main.h"

typedef struct _MAF
{
	int n_maf;

	int count;

	float *buffer;

	float output;
}MAF;

typedef struct _FIR
{
	int n_fir;

	double *b_fir;

	double *buffer;

	double output;
}FIR;

typedef struct _IIR
{
	int n_iir;

	double *a_iir;
	double *b_iir;

	double *buffer;

	double *outputBuffer;
	double output;
}IIR;

void MAF_Init(MAF* maf, int n_maf);
void MAF_Filter(MAF* maf, float rawdata);
void FIR_Init(FIR *fir, int n_fir, double* b_fir);
void FIR_Filter(FIR *fir, double rawdata);
void IIR_Init(IIR* iir, int n_iir, double* a_iir, double* b_iir);
void IIR_Filter(IIR* iir, double rawdata);
void tx_data(unsigned int rawdata, unsigned int filtereddata);

#endif /* INC_JSENSING_H_ */

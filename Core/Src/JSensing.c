/*
 * JSensing.c
 *
 *  Created on: Oct 5, 2023
 *      Author: Jongwook Baek
 */

#include "JSensing.h"

void MAF_Init(MAF* maf, int n_maf)
{
	maf->n_maf = n_maf;

	maf->buffer = (float*)malloc(sizeof(float)*(n_maf+1));
}

void MAF_Filter(MAF* maf, float rawdata)
{
	maf->buffer[maf->count] = rawdata;

	float sum = 0.0;
	for(int i = 0; i < maf->n_maf; i++)
	{
		sum += maf->buffer[i];
	}

	maf->output = sum / maf->n_maf;

	maf->count++;
	if(maf->count >= maf->n_maf)
	{
		maf->count = 0;
	}
}

void FIR_Init(FIR *fir, int n_fir, double* b_fir)
{
	fir->n_fir = n_fir;

	fir->b_fir = (double*)malloc(sizeof(double)*(n_fir+1));
	fir->buffer = (double*)malloc(sizeof(double)*(n_fir+1));

	fir->b_fir = b_fir;
}

void FIR_Filter(FIR *fir, double rawdata)
{
	fir->output = 0;

	for(int i = 0; i < fir->n_fir; i++)
	{
		fir->buffer[i] = fir->buffer[i+1];
	}
	fir->buffer[fir->n_fir] = rawdata;

	for(int k = 0; k < fir->n_fir+1; k++)
	{
		fir->output += fir->b_fir[k] * fir->buffer[fir->n_fir-k];
	}
}

void IIR_Init(IIR* iir, int n_iir, double* a_iir, double* b_iir)
{
	iir->n_iir = n_iir;

	iir->a_iir = (double*)malloc(sizeof(double)*(n_iir+1));
	iir->b_iir = (double*)malloc(sizeof(double)*(n_iir+1));
	iir->buffer = (double*)malloc(sizeof(double)*(n_iir+1));
	iir->outputBuffer = (double*)malloc(sizeof(double)*(n_iir+1));

	iir->a_iir = a_iir;
	iir->b_iir = b_iir;
}

void IIR_Filter(IIR* iir, double rawdata)
{
	iir->output = 0;

	for(int i = 0; i < iir->n_iir; i++)
	{
		iir->buffer[i] = iir->buffer[i+1];
		iir->outputBuffer[i] = iir->outputBuffer[i+1];
	}
	iir->buffer[iir->n_iir] = rawdata;

	for(int k = 0; k < iir->n_iir+1; k++)
	{
		iir->outputBuffer[iir->n_iir] += iir->b_iir[k] * iir->buffer[iir->n_iir-k] - iir->a_iir[k] * iir->outputBuffer[iir->n_iir-k];
	}

	iir->output = iir->outputBuffer[iir->n_iir];
}

void tx_data(unsigned int rawdata, unsigned int filtereddata)
{
	unsigned char data[10] = {0, };

	data[0] = (rawdata % 10000)/1000 + 48;
	data[1] = (rawdata % 1000)/100 + 48;
	data[2] = (rawdata % 100)/10 + 48;
	data[3] = (rawdata % 10) + 48;

	data[4] = 32;

	data[5] = (filtereddata % 10000)/1000 + 48;
	data[6] = (filtereddata % 1000)/100 + 48;
	data[7] = (filtereddata % 100)/10 + 48;
	data[8] = (filtereddata % 10) + 48;

	data[9] = 13;

	for(int i = 0; i < 10; i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART3));
		LL_USART_TransmitData8(USART3, data[i]);
	}
}

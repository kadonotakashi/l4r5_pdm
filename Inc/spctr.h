/*


 * fft_q15.h
 *
 *  Created on: 2017/12/21
 *      Author: 13539
 */

#ifndef SPCTR_H_
#define SPCTR_H_

#include "stm32l4xx_hal.h"
//#define ARM_MATH_CM4

#include <math.h>
#include "arm_math.h"
#include "arm_const_structs.h"

#define	ADCH_CNT	2
//#define	ADCH_CNT	1
#define	FFT_SIZE	1024
#define	AD_RANGE 0x1000	//12bit
#define	SAMPLE_PAGE 2	//
#define SAMPLE_FREQ	16000

typedef struct{
	int16_t wave[ADCH_CNT][FFT_SIZE];	//(FFT_SIZE * 2) x 2
}WAVE_Data;

typedef struct{
	uint16_t wave[SAMPLE_PAGE][FFT_SIZE][ADCH_CNT];	//(FFT_SIZE * 2) x 2
}AD_Data;

typedef struct{
	int16_t data[FFT_SIZE];	// FFT_SIZE;
}FFT_InData;					//1CH

typedef struct{
	int16_t data[FFT_SIZE][2];	// FFT_SIZE * ([real],[image]);
}FFT_OutData;

typedef struct{
	uint16_t data[ADCH_CNT+1][FFT_SIZE/2];	//ADCH_CNT x FFT_SIZE;
}SpctrPwr;

typedef struct{
	uint16_t count;
	uint16_t freq;
}SpctrBand;


typedef struct{
	int16_t coef[FFT_SIZE];
}WindFuncCoef;

#define	ADDR_FFT_POWER		(SpctrPwr *)0x20014000		//power data,FFT_RESULT�㔼��overwrite
#define	ADDR_WAVE_DATA		(WAVE_Data *)0x20016000		//top of WaveData
#define	ADDR_AD_DATA		(AD_Data *)0x20018000		//top of AD_DATA
#define	ADDR_FFT_Coef		(WindFuncCoef *)0x2001C000	//Window Function Table
#define	ADDR_FFT_InData		(FFT_InData *)0x2001D000	//FFT_DATA ( * coef)
#define	ADDR_FFT_OutData	(FFT_OutData *)0x2001E000	//FFT_RESULT


void  make_TestData(q15_t *src);
int calc_power(q15_t *FFTout,q15_t *pPOWER,int16_t *MaxPower,int *MaxIndex);
void fft_q15(q15_t *src,q15_t *rslt);

void MakeWindCoef(int16_t *pDATA);
void WindowFunction(int16_t *pIN,int16_t *pOUT,int16_t *pCOEF);
void MakeRawData(uint16_t *pIN,int16_t *pOUT);
void make_1_3_octTable( SpctrBand *tbl);
void Linear2octave(uint16_t *pIN,uint16_t *pOUT,SpctrBand *tbl);

#endif /* SPCTR_H_ */

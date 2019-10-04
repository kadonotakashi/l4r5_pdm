#include <math.h>
#include <spctr.h>



void  make_TestData(q15_t *src)
{
	int i;
	float value0,value1;

	for(i=0;i<FFT_SIZE;i++){
		value0 = sin(2*M_PI*i/(FFT_SIZE/8))*0.2;
		value0 = sin(2*M_PI*i/(FFT_SIZE/9))*0.2;
		value0 = sin(2*M_PI*i/(FFT_SIZE/10))*0.2;
		value0 = sin(2*M_PI*i/(FFT_SIZE/11))*0.2;
		value0 += sin(2*M_PI*i/(FFT_SIZE/78))+0.2;
		value0 += sin(2*M_PI*i/(FFT_SIZE/100))+0.2;
		value0 += sin(2*M_PI*i/(FFT_SIZE/280))+0.5;
		value1 = sin(2*M_PI*i/(FFT_SIZE/4));
		*src++ = (uint16_t)((value0+1)*512);
	}
}

int calc_power(q15_t *FFTout,q15_t *pPOWER,int16_t *MaxPower,int *MaxIndex){

	arm_cmplx_mag_q15((q15_t *)FFTout,(q15_t *)pPOWER,FFT_SIZE/2);	//���f�z��̃p���[�Z�o
	arm_max_q15((q15_t *)pPOWER,FFT_SIZE/2,(q15_t *)MaxPower,(uint32_t *)&MaxIndex);			//�ő�l�Z�o
	return *MaxPower;
}

void fft_q15(q15_t *src,q15_t *rslt)
{
	arm_rfft_instance_q15 S;
	arm_rfft_init_q15(&S, FFT_SIZE, 0, 1);
	arm_rfft_q15(&S,src, rslt);
}


void MakeWindCoef(int16_t *pDATA)
{
	int i;
	double pi_2x;
	double temp0,temp1,temp2;
	#define	CoefSize	30000.0

//	Blackman window
//  w(x)=0.42-0.5(cos (2*pi* x)) + 0.08(cos(4*pi*x))

	for(i=0;i<FFT_SIZE;i++,pDATA++){
		pi_2x  =  (double)( i *2);
		pi_2x /= (double)( FFT_SIZE );
		pi_2x *= M_PI;

		temp0 = cos(pi_2x)*0.50;
		temp1 = cos(2*pi_2x)*0.08;
		temp2 = 0.42 - temp0 + temp1;
		temp2 = temp2 * CoefSize;
		*pDATA = (uint16_t)temp2;
	}
};
void WindowFunction(int16_t *pIN,int16_t *pOUT,int16_t *pCOEF)
{
//	arm_mult_q15 ((q15_t *)pIN,(q15_t *)pCOEF,  (q15_t *)pOUT,FFT_SIZE);
	int i;

	for(i=0;i<FFT_SIZE;i++,pIN++,pOUT++,pCOEF++){
		*pOUT = ((*pIN) * (*pCOEF)) >> 11;
//		*pOUT = *pOUT >> 11;
	}
};

void MakeRawData(uint16_t *pIN,int16_t *pOUT)
{
	int16_t *pCH0,*pCH1;
	int i;

	for(i=0;i<FFT_SIZE;i++,pOUT++,pIN+=2){
		*pOUT = *pIN-0x800;
		*(pOUT+FFT_SIZE) = *(pIN+1)-0x800;
	}
};


//	���g���������j�A����1/3�I�N�^�[�u���Ƃɕϊ����邽�߂�
//	�e�[�u���쐬
//
//		typedef struct{
//			uint16_t count;	���j�A�X�P�[���ł̗v�f��
//			uint16_t freq;	���S���g��
//		}SpctrBand;
//
//
void make_1_3_octTable( SpctrBand *tbl){
	double coef;
	double border;
	int i;
	double freq;
	int band;
	double freq_step = SAMPLE_FREQ/FFT_SIZE;

	coef=pow(2.0,(1.0/6.0));	//	1/6�I�N�^�[�u�̌W��

	for(i=0;i<32;i++){
		(tbl+i)->count=-1;	(tbl+i)->freq=-1;
	}

	//band 0 �́ADC
	band=0;
	(tbl+band)->count=1;
	(tbl+band)->freq=0;

	//band 1 �́A���S���g��50Hz����J�n
	band=1;
	(tbl+band)->freq=50;
	(tbl+band)->count=0;
	border=50.0 * coef;	//����band�̏��

	for(i=1;i<FFT_SIZE/2;i++){
		freq = freq_step * i;
		if(freq>SAMPLE_FREQ/2)
			break;
		if(band>31)
			break;
		if(freq<border){
			(tbl+band)->count+=1;
		}
		else{
			band++;	//���̃o���h
			i--;	//
							//border�́A��band�̏���l=���̃o���h�̉����l
			border*= coef;	//����band�̒��S�ɂȂ���
 			(tbl+band)->freq=(int)border;	//�e�[�u���ɏ�������ł���
 			(tbl+band)->count=0;

			border*= coef;	//����band�̏���ɍX�V���đ��s
		}
	}

}



void Linear2octave(uint16_t *pIN,uint16_t *pOUT,SpctrBand *tbl)
{
	int index;
	int i,j;
	int count;
	int sum;
	int average;

	double LOG_DATA;

	index=0;
	for(i=0;i<32;i++){
		count = (tbl+i)->count;
		if (count==0xffff){
			break;
		}

#if 0 //average
		sum=0;
		for(j=0;j<count;j++){
			sum+=*(pIN+index);
			index++;
		}
		average=sum/count;
#endif

#if 1 //MAX
		average=0;
		for(j=0;j<count;j++){
			if(average<*(pIN+index)){
				average=*(pIN+index);
			}
			index++;
		}
#endif
		LOG_DATA = log10(average);
		average = (uint16_t)(LOG_DATA*17.5);

		if(average>63)
			average=63;


		*(pOUT+i)=average;
	}
}

/** \endlink */

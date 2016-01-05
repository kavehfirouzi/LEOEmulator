#include "algorithms.h"

#define HAMMDELAY 101 //101 = 76 + 25, hamming and low-pass filtering delays respectively
#define HAMMLENGTH 151
#define wSMOOTHHAMM 51

/* Hamming window values obtained from Matlab
 * b = fir1(150, 0.01); Matlab code
 */
const float HammingWin[HAMMLENGTH] = {0.00034186,0.000359,0.0003801,0.00040563,0.00043603,0.00047173,0.00051315,0.00056072,0.00061482,0.00067583,0.00074413,0.00082005,0.00090391,0.000996,0.0010966,0.0012059,0.0013242,0.0014516,0.0015883,0.0017344,0.00189,0.0020551,0.0022297,0.0024138,0.0026073,0.0028101,0.0030222,0.0032432,0.003473,0.0037114,0.0039579,0.0042124,0.0044743,0.0047434,0.0050192,0.0053012,0.0055889,0.0058818,0.0061792,0.0064807,0.0067856,0.0070932,0.007403,0.0077141,0.008026,0.0083378,0.0086489,0.0089586,0.009266,0.0095705,0.0098713,0.010168,0.010459,0.010744,0.011022,0.011293,0.011556,0.01181,0.012055,0.012289,0.012513,0.012725,0.012925,0.013113,0.013288,0.013449,0.013596,0.013729,0.013847,0.013951,0.014038,0.01411,0.014167,0.014207,0.014231,0.014239,0.014231,0.014207,0.014167,0.01411,0.014038,0.013951,0.013847,0.013729,0.013596,0.013449,0.013288,0.013113,0.012925,0.012725,0.012513,0.012289,0.012055,0.01181,0.011556,0.011293,0.011022,0.010744,0.010459,0.010168,0.0098713,0.0095705,0.009266,0.0089586,0.0086489,0.0083378,0.008026,0.0077141,0.007403,0.0070932,0.0067856,0.0064807,0.0061792,0.0058818,0.0055889,0.0053012,0.0050192,0.0047434,0.0044743,0.0042124,0.0039579,0.0037114,0.003473,0.0032432,0.0030222,0.0028101,0.0026073,0.0024138,0.0022297,0.0020551,0.00189,0.0017344,0.0015883,0.0014516,0.0013242,0.0012059,0.0010966,0.000996,0.00090391,0.00082005,0.00074413,0.00067583,0.00061482,0.00056072,0.00051315,0.00047173,0.00043603,0.00040563,0.0003801,0.000359,0.00034186
};

/**
 * @brief  low pass filtering RFVM RMS by means of a moving average hamming window followed by a normal moving average.
 * @param  RMS			: new rfvm RMS value
 * @param  Filtered : High-pass filter output
 * @parami delayedRMS : Input RMS delayed by the same amount it takes to calculate filtered RMS
 * @parami init  	  : if init = 0, initialize, else run the algorithm
 * @parami sel  	  : if sel = 0, run for rfvm, otherwise run for ham
 * @return nothing
 * @detatil         : instead of shifting the values inside the buffers to implement the sliding window effect, this function uses circulating pointers. The pointer is incremented
 *                  : everytime and is set to zero when it reaches the end of the buffer. Since segmentation is done on hamming filtered RMS but processed metrics work with RMS values,
					  RMS is delayed by the appropriate value by this function and passed to the following functions
 */
void HammmingSmooth_RFVM(float RMS, float *Filtered, float *delayedRMS, int init)
{
	static float HammingBuf[HAMMLENGTH];														//Buffer used for hamming filtering
	static float delay[HAMMDELAY];																//Buffer to hold RMS vales to delay it
	static float SmoothBuf[wSMOOTHHAMM];														//Buffer used for moving average smoothing
	static int ptrSmooth = 0;																	//index in which the latest data should be placed inside SmoothBuf
	static float SumSmooth = 0;																	//variable to hold sum of RMS values for smoothing
	static int ptr1 = 0;																		//index in which the latest data should be placed inside delay
	int ptr1_d;																					//indexed of the current delayed RMS value
	static int ptrHammingBuf = 0;																//index in which the latest data should be placed inside HammingBuf
	float *ptrRFVM;																				//pointer to the starting point inside HammingWin
	float sum;
	int i;
	if (init == 0)
	{
		/*New data is put inside the buffer and circular buffer is incremented*/
		HammingBuf[ptrHammingBuf] = RMS;
		ptrHammingBuf++;
		if (ptrHammingBuf == HAMMLENGTH)
			ptrHammingBuf = 0;

		/*First stage of Hamming filtering, RMS data from ptrHammingBuf position
		inside HammingBuf are multiplied and accumulated with hamming window values starting
		from the begging of the hamming window buffer*/
		sum = 0;
		ptrRFVM = (float*)&HammingBuf[ptrHammingBuf];
		sum = 0;
		for (i = 0; i < HAMMLENGTH-ptrHammingBuf; i++)
			sum += ptrRFVM[i]*HammingWin[i];

		/*Second stage of Hamming filtering, RMS data starting from the beginning of the buffer
		are multiplied and accumulated with hamming window values starting
		from HAMMLENGTH - ptrHammingBuf position of the hamming window buffer*/
		ptrRFVM = (float*)&HammingWin[HAMMLENGTH - ptrHammingBuf];
		for (i = 0; i<ptrHammingBuf; i++)
			sum += HammingBuf[i]*ptrRFVM[i];

		/*smoothing*/
		SumSmooth  = SumSmooth - SmoothBuf[ptrSmooth] + sum;
		SmoothBuf[ptrSmooth] = sum;
		ptrSmooth++;
		if (ptrSmooth == wSMOOTHHAMM)
			ptrSmooth = 0;
		*Filtered = SumSmooth / wSMOOTHHAMM;

		/*delaying the RMS*/
		delay[ptr1] = RMS;
		ptr1_d = ptr1+1;
		ptr1++;
		if (ptr1 == HAMMDELAY)
			ptr1 = 0;
		if (ptr1_d == HAMMDELAY)
			ptr1_d = 0;
		*delayedRMS = delay[ptr1_d];
	}
	else
	{
	}
}

/**
 * @brief  low pass filtering Hamm RMS by means of a moving average hamming window followed by a normal moving average.
 * @param  RMS			: new ham RMS value
 * @param  Filtered : High-pass filter output
 * @parami delayedRMS : Input RMS delayed by the same amount it takes to calculate filtered RMS
 * @parami init  	  : if init = 0, initialize, else run the algorithm
 * @parami sel  	  : if sel = 0, run for rfvm, otherwise run for ham
 * @return nothing
 * @detatil         : instead of shifting the values inside the buffers to implement the sliding window effect, this function uses circulating pointers. The pointer is incremented
 *                  : everytime and is set to zero when it reaches the end of the buffer. Since segmentation is done on hamming filtered RMS but processed metrics work with RMS values,
					  RMS is delayed by the appropriate value by this function and passed to the following functions
 */
void HammmingSmooth_Ham(float RMS, float *Filtered, float *delayedRMS, int init)
{
	static float HammingBuf[HAMMLENGTH];														//Buffer used for hamming filtering
	static float delay[HAMMDELAY];																//Buffer to hold RMS vales to delay it
	static float SmoothBuf[wSMOOTHHAMM];														//Buffer used for moving average smoothing
	static int ptrSmooth = 0;																	//index in which the latest data should be placed inside SmoothBuf
	static float SumSmooth = 0;																	//variable to hold sum of RMS values for smoothing
	static int ptr1 = 0;																		//index in which the latest data should be placed inside delay
	int ptr1_d;																					//indexed of the current delayed RMS value
	static int ptrHammingBuf = 0;																//index in which the latest data should be placed inside HammingBuf
	float *ptrHam;																				//pointer to the starting point inside HammingWin
	float sum;
	int i;
	if (init == 0)
	{
		/*New data is put inside the buffer and circular buffer is incremented*/
		HammingBuf[ptrHammingBuf] = RMS;
		ptrHammingBuf++;
		if (ptrHammingBuf == HAMMLENGTH)
			ptrHammingBuf = 0;

		/*First stage of Hamming filtering, RMS data from ptrHammingBuf position
		inside HammingBuf are multiplied and accumulated with hamming window values starting
		from the begging of the hamming window buffer*/
		sum = 0;
		ptrHam = (float*)&HammingBuf[ptrHammingBuf];
		sum = 0;
		for (i = 0; i < HAMMLENGTH-ptrHammingBuf; i++)
			sum += ptrHam[i]*HammingWin[i];

		/*Second stage of Hamming filtering, RMS data starting from the beginning of the buffer
		are multiplied and accumulated with hamming window values starting
		from HAMMLENGTH - ptrHammingBuf position of the hamming window buffer*/
		ptrHam = (float*)&HammingWin[HAMMLENGTH - ptrHammingBuf];
		for (i = 0; i<ptrHammingBuf; i++)
			sum += HammingBuf[i]*ptrHam[i];

		/*smoothing*/
		SumSmooth  = SumSmooth - SmoothBuf[ptrSmooth] + sum;
		SmoothBuf[ptrSmooth] = sum;
		ptrSmooth++;
		if (ptrSmooth == wSMOOTHHAMM)
			ptrSmooth = 0;
		*Filtered = SumSmooth / wSMOOTHHAMM;

		/*delaying the RMS*/
		delay[ptr1] = RMS;
		ptr1_d = ptr1+1;
		ptr1++;
		if (ptr1 == HAMMDELAY)
			ptr1 = 0;
		if (ptr1_d == HAMMDELAY)
			ptr1_d = 0;
		*delayedRMS = delay[ptr1_d];
	}
	else
	{
	}
}

void HammmingSmooth_VL(float RMS, float *Filtered, float *delayedRMS, int init)
{
	static float HammingBuf[HAMMLENGTH];														//Buffer used for hamming filtering
	static float delay[HAMMDELAY];																//Buffer to hold RMS vales to delay it
	static float SmoothBuf[wSMOOTHHAMM];														//Buffer used for moving average smoothing
	static int ptrSmooth = 0;																	//index in which the latest data should be placed inside SmoothBuf
	static float SumSmooth = 0;																	//variable to hold sum of RMS values for smoothing
	static int ptr1 = 0;																		//index in which the latest data should be placed inside delay
	int ptr1_d;																					//indexed of the current delayed RMS value
	static int ptrHammingBuf = 0;																//index in which the latest data should be placed inside HammingBuf
	float *ptrVL;																				//pointer to the starting point inside HammingWin
	float sum;
	int i;
	if (init == 0)
	{
		/*New data is put inside the buffer and circular buffer is incremented*/
		HammingBuf[ptrHammingBuf] = RMS;
		ptrHammingBuf++;
		if (ptrHammingBuf == HAMMLENGTH)
			ptrHammingBuf = 0;

		/*First stage of Hamming filtering, RMS data from ptrHammingBuf position
		inside HammingBuf are multiplied and accumulated with hamming window values starting
		from the begging of the hamming window buffer*/
		sum = 0;
		ptrVL = (float*)&HammingBuf[ptrHammingBuf];
		sum = 0;
		for (i = 0; i < HAMMLENGTH-ptrHammingBuf; i++)
			sum += ptrVL[i]*HammingWin[i];

		/*Second stage of Hamming filtering, RMS data starting from the beginning of the buffer
		are multiplied and accumulated with hamming window values starting
		from HAMMLENGTH - ptrHammingBuf position of the hamming window buffer*/
		ptrVL = (float*)&HammingWin[HAMMLENGTH - ptrHammingBuf];
		for (i = 0; i<ptrHammingBuf; i++)
			sum += HammingBuf[i]*ptrVL[i];

		/*smoothing*/
		SumSmooth  = SumSmooth - SmoothBuf[ptrSmooth] + sum;
		SmoothBuf[ptrSmooth] = sum;
		ptrSmooth++;
		if (ptrSmooth == wSMOOTHHAMM)
			ptrSmooth = 0;
		*Filtered = SumSmooth / wSMOOTHHAMM;

		/*delaying the RMS*/
		delay[ptr1] = RMS;
		ptr1_d = ptr1+1;
		ptr1++;
		if (ptr1 == HAMMDELAY)
			ptr1 = 0;
		if (ptr1_d == HAMMDELAY)
			ptr1_d = 0;
		*delayedRMS = delay[ptr1_d];
	}
	else
	{
	}
}

#include "algorithms.h"

/* Constant butterworth filter coefficients, obtained from MATLAB
 * Butterworth is an order 4 filter with band-stop range [58-32]
 * [b,a] = butter(2,[58/250 62/250],'stop'); Matlab code
 */
double Butter_b[5] = {0.965080986344736,  -2.814944037358806,   3.982816114046526,  -2.814944037358806, 0.965080986344736};
double Butter_a[4] = {-2.864980440387021,   3.981596404609088,  -2.764907634330585, 0.931381682126905};
float tempFiltered_RF;						//temporary

/* Butterworth notch-filters for rfvm and ham
 * Band reject notch butterworth notch filters to get rid of 60Hz noise
 * Butterworth requires double variables otherwise it could become unstable, with such
 * an small range double variabes are strictly required
 */
void ButterRF(double newData, double *Butterout);
void ButterH(double newData, double *Butterout);
void ButterVL(double newData, double *Butterout);

/**
 * @brief  preprocessing raw EMG
 * @param  rawdata	   : pointer to buffer holding raw EMG values. Highpass filtering, butterworth notchfilter to remove 60Hz and RMS.
 * @param  Results     : pointer to the bluetooth buffer
 * @parami init  	   : if init = 0, initialize buffers and variables, else run the algorithm
 * @parami LR   	   : left/right leg selection. The order of rfvm and ham or swaped inside the buffer for left and right leg.
 * @detail			   : For rfvm, absluote values of RF and VL are added together after highpass filtering
 * @return	nothing
 */

void PreProcessing(int *RawEMG, float* FilteredEMG, float* SmoothedRMS, int sel, int freeMode)
{
			float32_t filtered, RMS, butter;
			double temp;
			static int counter = 0;
			static int k = -1;
			if (freeMode == 0)
			{
				if (sel == 0)
				{
					EMGHighPass(RawEMG[0], &filtered, 0, 0);
					ButterRF(filtered, &temp);
					butter = temp;
				}
				if (sel == 1)
				{
					EMGHighPass(RawEMG[1], &filtered, 0, 1);
					ButterH(filtered, &temp);
					butter = temp;
				}
				if (sel == 2)
				{
					EMGHighPass(RawEMG[2], &filtered, 0, 2);
					ButterVL(filtered, &temp);
					butter = temp;
				}
				
				emgRMS(butter, &RMS, 0, sel, RMS_WIN_SIZE);
				*FilteredEMG = butter;
				*SmoothedRMS = RMS;
			}
			else
			{
				if (sel == 0)
				{
					EMGHighPass(RawEMG[0], &filtered, 0, 0);
					ButterRF(filtered, &temp);
					butter = abs(temp);
					EMGHighPass(RawEMG[2], &filtered, 0, 2);
					ButterVL(filtered, &temp);
					butter += abs(temp);
				}
				if (sel == 1)
				{
					EMGHighPass(RawEMG[1], &filtered, 0, 1);
					ButterH(filtered, &temp);
					butter = temp;
				}
				if (sel == 0)
				{
					*FilteredEMG = k*butter;
					counter++;
					if (counter == 4)
					{
						counter = 0;
						k *= -1;
					}
				}
				else
					*FilteredEMG = butter;
			}
}

/*
* Butterworth filter: Y[n] = b0*X[n] + b1*X[n-1] + ... B4*X[n-4] + a0*Y[n-1] + ... + a3*Y[n-3]
*/
void ButterRF(double newData, double *Butterout)
{
		static double DataBuff[5+1]; 												//Buffer to hold input data, One extra element for padding needed to avoid memory corruption
		static double OutBuff[4+1];													//Buffer to hold previous outputs
		double sum;
		int j;
		double Data = newData;
		for (j=0 ; j< 4; j++)																//Shifting input data inside the buffer
		{
			DataBuff[j] = DataBuff[j+1];
		}
		DataBuff[4] = Data;
		sum = 0;
		/*Finding b0*X[n] + b1*X[n-1] + ... B4*X[n-4]*/
		for (j = 0; j<5; j++)
		{
			sum += DataBuff[j] * Butter_b[j];
		}
		/*Finding a0*Y[n-1] + ... + a3*Y[n-3]*/
		for (j = 0; j<4; j++)
		{
			sum -= OutBuff[j] * Butter_a[j];
		}
		for (j = 4 ; j > 0; j--)														//Shifting outputs inside the buffer
		{
			OutBuff[j] = OutBuff[j-1];
		}
		OutBuff[0] = sum;
		*Butterout =  sum;
}

/*
* Butterworth filter: Y[n] = b0*X[n] + b1*X[n-1] + ... B4*X[n-4] + a0*Y[n-1] + ... + a3*Y[n-3]
*/
void ButterH(double newData, double *Butterout)
{
		static double DataBuff[5+1]; 												//Buffer to hold input data, One extra element for padding needed to avoid memory corruption
		static double OutBuff[4+1];													//Buffer to hold previous outputs
		double sum;
		int j;
		double Data = newData;
		for (j=0 ; j< 4; j++)																//Shifting input data inside the buffer
		{
			DataBuff[j] = DataBuff[j+1];
		}
		DataBuff[4] = Data;
		sum = 0;
		/*Finding b0*X[n] + b1*X[n-1] + ... B4*X[n-4]*/
		for (j = 0; j<5; j++)
		{
			sum += DataBuff[j] * Butter_b[j];
		}
		/*Finding a0*Y[n-1] + ... + a3*Y[n-3]*/
		for (j = 0; j<4; j++)
		{
			sum -= OutBuff[j] * Butter_a[j];
		}
		for (j = 4 ; j > 0; j--)														//Shifting outputs inside the buffer
		{
			OutBuff[j] = OutBuff[j-1];
		}
		OutBuff[0] = sum;
		*Butterout =  sum;
}

/*
* Butterworth filter: Y[n] = b0*X[n] + b1*X[n-1] + ... B4*X[n-4] + a0*Y[n-1] + ... + a3*Y[n-3]
*/
void ButterVL(double newData, double *Butterout)
{
		static double DataBuff[5+1]; 												//Buffer to hold input data, One extra element for padding needed to avoid memory corruption
		static double OutBuff[4+1];													//Buffer to hold previous outputs
		double sum;
		int j;
		double Data = newData;
		for (j=0 ; j< 4; j++)																//Shifting input data inside the buffer
		{
			DataBuff[j] = DataBuff[j+1];
		}
		DataBuff[4] = Data;
		sum = 0;
		/*Finding b0*X[n] + b1*X[n-1] + ... B4*X[n-4]*/
		for (j = 0; j<5; j++)
		{
			sum += DataBuff[j] * Butter_b[j];
		}
		/*Finding a0*Y[n-1] + ... + a3*Y[n-3]*/
		for (j = 0; j<4; j++)
		{
			sum -= OutBuff[j] * Butter_a[j];
		}
		for (j = 4 ; j > 0; j--)														//Shifting outputs inside the buffer
		{
			OutBuff[j] = OutBuff[j-1];
		}
		OutBuff[0] = sum;
		*Butterout =  sum;
}

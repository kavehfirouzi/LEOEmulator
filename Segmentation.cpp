#include "algorithms.h"

#define SEG_WIN_SIZE 1200																// Size of the classification window
#define DIV_FACTOR_EMG 7

extern uint8_t cadence_acc;
extern uint8_t cadence_gyro;															// Cadence to be used for calculating the dynamic range
extern uint32_t global_counter;

void UnsupervisedRFVM(uint16_t RMS, int *segment, int init);
void UnsupervisedHam(uint16_t RMS, int *segment, int init);
void UnsupervisedVL(uint16_t RMS, int *segment, int init);

/**
 * @brief	Segmenting the smoothed RMS
 * @param   RMS   	   : input smoothed RmS
 * @param	*segment   : segmentation result
 * @parami  sel		   : select between rfvm and ham
 * @parami  init  	   : if init = 0, initialize, else run the algorithm
 * @detail             : Segmentation is based on comparing RMS values with max and min values within a window before the RMS point in question.
 * 					   : The range of the window is dynamically calculated based on the cadence value.
 */
void Unsupervised(uint16_t RMS, int32_t *segment, int sel, int init)
{
	 if (init == 1)																	    // Initialize the algorithms
	 {
		 UnsupervisedRFVM(RMS, segment, 1);
		 UnsupervisedRFVM(RMS, segment, 1);
		 UnsupervisedVL(RMS, segment, 1);
	 }
	 else
	 {
		if (sel == 0)																	// Run the algorithms
			UnsupervisedRFVM(RMS, segment, 0);
		else if (sel == 1)
			UnsupervisedHam(RMS, segment, 0);
		else
			UnsupervisedVL(RMS, segment, 0);
	 }
}
void UnsupervisedRFVM(uint16_t RMS, int *segment, int init)
{  
	static uint16_t RMSBuff[SEG_WIN_SIZE];												// Buffer to hold smoothed RMS values
    static int pnt = 0;
	int max;
	int min;
	int j, W, start, finish_1, finish_2;
	if (init == 0)
	{
		global_counter++;
		RMSBuff[pnt] = RMS;
		if (cadence == 0)															// Calculating the dynamic range value
			W = 0;
		else
			W = (60*400/cadence);
		if (W > SEG_WIN_SIZE)
			W = SEG_WIN_SIZE;
		if (W < 100)
			W = 100;
		start = pnt - W;

		if (start < 0)																	// Finding the starting and stopping elements in the buffer to find Max and Min values within them
		{
			start += SEG_WIN_SIZE;
			finish_1 = SEG_WIN_SIZE;
			finish_2 = pnt;
		}
		else
		{
			finish_1 = pnt;
			finish_2 = 0;
		}
		pnt++;
		if (pnt == SEG_WIN_SIZE)
			pnt = 0;
		max = RMSBuff[start];
		min = RMSBuff[start];
		for (j=start; j < finish_1 ; j++)												// Finding the maximum and minimum
		{
			if (max < RMSBuff[j])
					max = RMSBuff[j];
			if (min > RMSBuff[j])
					min = RMSBuff[j];
		}
		for (j=0; j < finish_2 ; j++)
		{
			if (max < RMSBuff[j])
					max = RMSBuff[j];
			if (min > RMSBuff[j])
					min = RMSBuff[j];
		}
		if (RMS > (max-min)/DIV_FACTOR_EMG + min )										// Classifying based on max and min values
		{
			*segment = 1;
		}
		else
		{
			*segment = 0;
		}
	}
	else																				// Resetting the buffer
	{
		for (j = 0; j < SEG_WIN_SIZE; j++)
			RMSBuff[j] = 0;
	}
}

void UnsupervisedHam(uint16_t RMS, int *segment, int init)
{
	static uint16_t RMSBuff[SEG_WIN_SIZE];												// Buffer to hold smoothed RMS values
    static int pnt = 0;
	int max;
	int min;
	int j, W, start, finish_1, finish_2;
	if (init == 0)
	{
		RMSBuff[pnt] = RMS;
		if (cadence == 0)															// Calculating the dynamic range value
			W = 0;
		else
			W = (60*400/cadence);
		if (W > SEG_WIN_SIZE)
			W = SEG_WIN_SIZE;
		if (W < 100)
			W = 100;
		start = pnt - W;
		if (start < 0)																	// Finding the starting and stopping elements in the buffer to find Max and Min values within them
		{
			start += SEG_WIN_SIZE;
			finish_1 = SEG_WIN_SIZE;
			finish_2 = pnt;
		}
		else
		{
			finish_1 = pnt;
			finish_2 = 0;
		}
		pnt++;
		if (pnt == SEG_WIN_SIZE)
			pnt = 0;
		max = RMSBuff[start];
		min = RMSBuff[start];
		for (j=start; j < finish_1 ; j++)												// Finding the maximum and minimum
		{
			if (max < RMSBuff[j])
					max = RMSBuff[j];
			if (min > RMSBuff[j])
					min = RMSBuff[j];
		}
		for (j=0; j < finish_2 ; j++)
		{
			if (max < RMSBuff[j])
					max = RMSBuff[j];
			if (min > RMSBuff[j])
					min = RMSBuff[j];
		}
		if (RMS > (max-min)/DIV_FACTOR_EMG + min )										// Classifying based on max and min values
		{
			*segment = 1;
		}
		else
		{
			*segment = 0;
		}
	}
	else																				// Resetting the buffer
	{
		for (j = 0; j < SEG_WIN_SIZE; j++)
			RMSBuff[j] = 0;
	}
}

void UnsupervisedVL(uint16_t RMS, int *segment, int init)
{
	static uint16_t RMSBuff[SEG_WIN_SIZE];												// Buffer to hold smoothed RMS values
    static int pnt = 0;
	int max;
	int min;
	int j, W, start, finish_1, finish_2;
	if (init == 0)
	{
		RMSBuff[pnt] = RMS;
		if (cadence == 0)															// Calculating the dynamic range value
			W = 0;
		else
			W = (60*400/cadence);
		if (W > SEG_WIN_SIZE)
			W = SEG_WIN_SIZE;
		if (W < 100)
			W = 100;
		start = pnt - W;

		if (start < 0)																	// Finding the starting and stopping elements in the buffer to find Max and Min values within them
		{
			start += SEG_WIN_SIZE;
			finish_1 = SEG_WIN_SIZE;
			finish_2 = pnt;
		}
		else
		{
			finish_1 = pnt;
			finish_2 = 0;
		}
		pnt++;
		if (pnt == SEG_WIN_SIZE)
			pnt = 0;
		max = RMSBuff[start];
		min = RMSBuff[start];
		for (j=start; j < finish_1 ; j++)												// Finding the maximum and minimum
		{
			if (max < RMSBuff[j])
					max = RMSBuff[j];
			if (min > RMSBuff[j])
					min = RMSBuff[j];
		}
		for (j=0; j < finish_2 ; j++)
		{
			if (max < RMSBuff[j])
					max = RMSBuff[j];
			if (min > RMSBuff[j])
					min = RMSBuff[j];
		}
		if (RMS > (max-min)/DIV_FACTOR_EMG + min )										// Classifying based on max and min values
		{
			*segment = 1;
		}
		else
		{
			*segment = 0;
		}
	}
	else																				// Resetting the buffer
	{
		for (j = 0; j < SEG_WIN_SIZE; j++)
			RMSBuff[j] = 0;
	}
}

#include "algorithms.h"

/**
 * @brief	Finding the RMS values of the the values inside a sliding window. New data is fed to the algorithm one at a time for real-time purposes.
 * @param Data	    : new data
 * @param MAout     : RMS output
 * @parami init  	  : if init = 0, initialize, else run the algorithm
 * @parami sel  	  : if sel = 0, run for rfvm, otherwise run for ham
 * @return	nothing
 * @detatil         : instead of shifting the values inside the buffers to implement the sliding window effect, this function uses circulating pointers. The pointer is incremented
 *                  : everytime and is set to zero when it reaches the end of the buffer. Once the buffer is filled, pointer points to the oldest value inside the buffer and the new data
 *                  : would replace the old data.
 */	
extern int my_counter;

void emgRMS(float32_t Data, float32_t *RMSOut, uint32_t init, uint32_t sel, int size)
{
	int i;
	static float buffer_1[RMS_WIN_SIZE];																				// Buffer to hold rfvm values
	static float buffer_2[RMS_WIN_SIZE];																				// Buffer to hold ham values
	static float buffer_3[RMS_WIN_SIZE];																				// Buffer to hold ham values
	static float RMS_1 = 0;																										// Variable to hold the sum of squared values inside the sliding window
	static int pointer_1 = 0;																									// Pointer to the place inside the buffer that holds the odest value
	static float RMS_2 = 0;
	static int pointer_2 = 0;
	static float RMS_3 = 0;
	static int pointer_3 = 0;
	static int count1 = 0;																										// A counter that keeps the output at zero before the buffer is filled
	static int count2 = 0;
	static int count3 = 0;
	float *buffer, *RMS;
	int *pointer;
	float newData = Data;
	int *count;
	if (sel == 0)																															// If sel = 1, set the pointers to Ham buffer and variables
	{
		buffer = buffer_1;
		RMS = &RMS_1;
		pointer = &pointer_1;
		count = &count1;
	}
	else if (sel == 1)																																	// If sel = 0, set the pointers to Ham buffer and variables
	{
		buffer = buffer_2;
		RMS = &RMS_2;
		pointer = &pointer_2;
		count = &count2;
	}
	else
	{
		buffer = buffer_3;
		RMS = &RMS_3;
		pointer = &pointer_3;
		count = &count3;
	}
	if (init == 0)																														// If init = 0, Run the algorithms
	{
		*RMS = *RMS - (buffer[*pointer]) + newData*newData;											// subtract the oldest value in the buffer and add the square of the latest input data
		if (*RMS < 0)																														// Just a caution to avoid negative RMS values in the beggining, before fillin the buffer, most likely unnecessary
			(*RMS = 0);
		buffer[*pointer] = newData*newData;																			// Replace the oldest value with the square of the latest input data
		(*pointer)++;																														// Increment the pointer, set it to zero if it reaches the end
		if (*pointer>size-1)
			(*pointer) = 0;
		if ((*count) < MA_SIZE+size+1)                                          // Before the buffer is filled, set the output to zero
			*RMSOut = 0;
		else
			*RMSOut = sqrtf(*RMS/size);
		(*count)++;
	}
	else																																			// If init = 1, initialize the buffers and variables
	{
		for (i=0; i<RMS_WIN_SIZE; i++)
		{
			buffer_1[i] = 0;																												// Set the buffers to zero
			buffer_2[i] = 0;
			buffer_3[i] = 0;
		}
			count1 = 0;																														// Set the variabled to initial values
			count2 = 0;
			count3 = 0;
			RMS_1 = 0;
			pointer_1 = 0;
			RMS_2 = 0;
			pointer_2 = 0;
			RMS_3 = 0;
			pointer_3 = 0;
	}
}

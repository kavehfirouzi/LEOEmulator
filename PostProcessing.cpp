#include "algorithms.h"

#define Max_Postproc 	52																			// Maximum detrending size

#define posprocSeg 50
#define posprocGap 25																						// External variables that set the threshold for the algorithm


/**
 * @brief	Calling differnet post processing functions based and action and RFVM/Ham
 * @param SegIn	    : input segmentation data, which could be either 0 or 1
 * @param SegProcessed : processed input
 * @parami RMS1_in  : Input rfvm RMS value
 * @paramo RMS1_out : delayed rfvm RMS
 * @parami RMS2_in  : Input ham RMS value
 * @paramo RMS2_ou  : delayed hamd RMS
 * @parami sel  	  : if sel = 0, run for rfvm, otherwise run for ham
 * @parami action  	: if init = 0, initialize, else run the algorithm
 * @return	nothing
 */	
 
void PostProcessing(int32_t SegIn, int32_t *SegProcessed, float32_t RMS1_in, float32_t *RMS1_out, float32_t RMS2_in, float32_t *RMS2_out, float32_t RMS3_in, float32_t *RMS3_out, int sel)
{
	int32_t	 dummy1;
	float32_t RMS1, RMS2, RMS3;
	PostProcessSeg(SegIn, &dummy1, RMS1_in, &RMS1, RMS2_in, &RMS2, RMS3_in, &RMS3, sel, 0);       // First post-processing (getting rid of small segments)
	PostProcessGap(dummy1, SegProcessed, RMS1, RMS1_out, RMS2, RMS2_out, RMS3, RMS3_out, sel, 0);	// Second post-processing (getting rid of small gaps)
}
/**
 * @brief	Post processing operation to get rid of segments samller than a certain size, for real-time practice, input data is fed one at a time.
 *                  : the function also gets RMS values for rfvm and ham and adds a delay to them by means of shifting them inside a buffer.
 * @param in	      : input segmentation data, which could be either 0 or 1
 * @param out       : processed input
 * @parami RMS_rfvm : Input rfvm RMS value
 * @paramo RMSout_rfvm : delayed rfvm RMS
 * @parami RMS_rfvm : Input ham RMS value
 * @paramo RMSout_rfvm : delayed hamd RMS
 * @parami sel  	  : if sel = 0, run for rfvm, otherwise run for ham
 * @parami init  	  : if init = 0, initialize, else run the algorithm
 * @return	nothing
 * @detail         : The delay in RMS values is added to avoid shift between segmentation and RMS when it comes to find the peak values and intesity.
 *                  : The function works as a state machine. It has two states. Reset state and Running state. When it's at reset state and the input is 0, it does nothing.
 *                  : Otherwise if the input is 1, it means the beggining of a segmentation and it changes state to running state. In this state a counter increments everytime with each 
 *                  : input wuth value 1 to keep track of the length of the segment. If the input goes to zero in this state, it means the segment has ended. If the length of the segment is
 *                  : smaller than a threshold, this segment is eliminated and set to zero.
 */	
 
void PostProcessSeg(int in, int* out, float RMS_rfvm, float *RMSout_rfvm, float RMS_ham, float *RMSout_ham, float RMS_vl, float *RMSout_vl, int sel, int init)
{
	static int state_1 = 0;																								// state machine for rfvm
	static int counter_1 = 0;																							// counter that keeps track of the length of the segment
	static int Buffer_1[Max_Postproc];																		// Buffer to hold segmentation values
	static int Buffer_RMSrfvm[Max_Postproc];														// Buffer to hold rfvm RMS values
	static int state_2 = 0;																								// same thing as above, but for ham
	static int counter_2 = 0;
	static int Buffer_2[Max_Postproc];
	static int Buffer_RMSham[Max_Postproc];
	static int state_3 = 0;																								// state machine for rfvm
	static int counter_3 = 0;																							// counter that keeps track of the length of the segment
	static int Buffer_3[Max_Postproc];																		// Buffer to hold segmentation values
	static int Buffer_RMSvl[Max_Postproc];														// Buffer to hold rfvm RMS values
	int *state;
	int *counter;
	int *Buffer;
	int *Buffer_RMS;
	int i;
	if (init == 0)																												// If init = 0, run the algortihm
	{
			if (sel == 0)																											// If sel = 0, set the pointers to buffers and variable for rfvm
			{
				state = &state_1;
				counter = &counter_1;
				Buffer = Buffer_1;
				Buffer_RMS =  Buffer_RMSrfvm;
			}
			else if (sel == 1)																															// If sel = 0, set the pointers to buffers and variable for rfvm
			{
				state = &state_2;
				counter = &counter_2;
				Buffer = Buffer_2;
				Buffer_RMS =  Buffer_RMSham;
			}
			else
			{
				state = &state_3;
				counter = &counter_3;
				Buffer = Buffer_3;
				Buffer_RMS =  Buffer_RMSvl;
			}
			for (i = 0; i < Max_Postproc; i++)																				 // Shift the values inside the buffers 
			{
				Buffer[i] = Buffer[i+1];
				Buffer_RMS[i] = Buffer_RMS[i+1];
			}
			Buffer[Max_Postproc-1] = in;																								 // put the new data inside the buffer
			if (sel == 0)
				Buffer_RMS[Max_Postproc-1] = RMS_rfvm;
			else if (sel == 1)
				Buffer_RMS[Max_Postproc-1] = RMS_ham;
			else
				Buffer_RMS[Max_Postproc-1] = RMS_vl;
			switch(*state)																									 // State machine
			{
				case 0:																												 // Reset state
					*counter = 0;																								 // Reset counter
					if (in == 1)																								 // If input is one, the segment has started, start counting and go to running state
					{
						(*counter)++;
						*state = 1;
					}
					break;
				case 1:																												 // Running state
					if (in == 1)																								 // If input is one, keep counting
						(*counter)++;
					if (in == 0)																								 // If input = 0, segment has ended, check the length of the segment
					{
						if (*counter < posprocSeg)																		 // If the length of the segment is smaller than a threshold, get rid of the segment by making it zero
						{
							for (i = (Max_Postproc - 1 - *counter); i<Max_Postproc-1; i++)
								Buffer[i] = 0;
						}
						*state = 0;																								 // go to reset state
					}
					break;
			}
			*out = Buffer[0];																								// The outputs of the post processing algorithm and delayed RMS values
			if (sel == 0)
				*RMSout_rfvm = Buffer_RMS[0];
			else if (sel == 1)
				*RMSout_ham = Buffer_RMS[0];
			else
				*RMSout_vl = Buffer_RMS[0];
	}
	else																																// Initialize
	{
		state_1 = 0;																											// Set parameters to zero
		counter_1 = 0;
		state_2 = 0;
		counter_2 = 0;
		for (i = 0; i<Max_Postproc; i++)																					  // Set buffers to zero
		{	
			Buffer_1[i] = 0;
			Buffer_RMSrfvm[i] = 0;
			Buffer_2[i] = 0;
			Buffer_RMSham[i] = 0;
		}
		
	}
}

/**
 * @brief	Post processing operation to get rid of empty spaces between two segments that are samller than a certain size, for real-time practice, input data is fed one at a time.
 *                  : the function also gets RMS values for rfvm and ham and adds a delay to them by means of shifting them inside a buffer.
 * @param  in	      : input segmentation data, which could be either 0 or 1
 * @param  out      : processed input
 * @parami RMS_rfvm : Input rfvm RMS value
 * @paramo RMSout_rfvm : delayed rfvm RMS
 * @parami RMS_rfvm : Input ham RMS value
 * @paramo RMSout_rfvm : delayed hamd RMS
 * @parami sel  	  : if sel = 0, run for rfvm, otherwise run for ham
 * @parami init  	  : if init = 0, initialize, else run the algorithm
 * @return	nothing
 * @detail         : The delay in RMS values is added to avoid shift between segmentation and RMS when it comes to find the peak values and intesity.
 *                  : The function works as a state machine. It has two states. Reset state and Running state. When it's at reset state and the input is 0, it does nothing.
 *                  : Otherwise if the input is 1, it means the beggining of a segmentation and it changes state to running state. In this state a counter increments everytime with each 
 *                  : input wuth value 1 to keep track of the length of the segment. If the input goes to zero in this state, it means the segment has ended. If the length of the segment is
 *                  : smaller than a threshold, this segment is eliminated and set to zero.
 */	
 
void PostProcessGap(int in, int* out, float RMS_rfvm, float *RMSout_rfvm, float RMS_ham, float *RMSout_ham, float RMS_vl, float *RMSout_vl, int sel, int init)
{
	static int state_1 = 0;																								// state machine for rfvm
	static int counter_1 = 0;																							// counter that keeps track of the length of the segment
	static int Buffer_1[Max_Postproc];																		// Buffer to hold segmentation values
	static int Buffer_RMSrfvm[Max_Postproc];														// Buffer to hold rfvm RMS values
	static int state_2 = 0;																								// same thing as above, but for ham
	static int counter_2 = 0;
	static int Buffer_2[Max_Postproc];
	static int Buffer_RMSham[Max_Postproc];
	static int state_3 = 0;																								// same thing as above, but for ham
	static int counter_3 = 0;
	static int Buffer_3[Max_Postproc];
	static int Buffer_RMSvl[Max_Postproc];
	int *state;
	int *counter;
	int *Buffer;
	int *Buffer_RMS;
	int i;
	if (init == 0)																												// If init = 0, run the algortihm
	{
			if (sel == 0)																											// If sel = 0, set the pointers to buffers and variable for rfvm
			{
				state = &state_1;
				counter = &counter_1;
				Buffer = Buffer_1;
				Buffer_RMS =  Buffer_RMSrfvm;
			}
			else if (sel == 1)																															// If sel = 0, set the pointers to buffers and variable for rfvm
			{
				state = &state_2;
				counter = &counter_2;
				Buffer = Buffer_2;
				Buffer_RMS =  Buffer_RMSham;
			}
			else
			{
				state = &state_3;
				counter = &counter_3;
				Buffer = Buffer_3;
				Buffer_RMS =  Buffer_RMSvl;
			}
			for (i = 0; i < Max_Postproc; i++)																				 // Shift the values inside the buffers 
			{
				Buffer[i] = Buffer[i+1];
				Buffer_RMS[i] = Buffer_RMS[i+1];
			}
			Buffer[Max_Postproc-1] = in;																								 // put the new data inside the buffer
			if (sel == 0)
				Buffer_RMS[Max_Postproc-1] = RMS_rfvm;
			else if (sel == 1)
				Buffer_RMS[Max_Postproc-1] = RMS_ham;
			else
				Buffer_RMS[Max_Postproc-1] = RMS_vl;
			switch(*state)																									 // State machine
			{
				case 0:																												 // Reset state
					*counter = 0;																								 // Reset counter
					if (in == 0)																								 // If input is zero, the segment has started, start counting and go to running state
					{
						(*counter)++;
						*state = 1;
					}
					break;
				case 1:																												 // Running state
					if (in == 0)																								 // If input is zero, keep counting
						(*counter)++;
					if (in == 1)																								 // If input = 1, the gap between two segments has ended, check the length of the gap
					{
						if (*counter < posprocGap)																		 // If the length of the gap is smaller than a threshold, get rid of the gap by making it ones
						{
							for (i = (Max_Postproc - 1 - *counter); i<Max_Postproc-1; i++)
								Buffer[i] = 1;
						}
						*state = 0;																								 // go to reset state
					}
					break;
			}
			*out = Buffer[0];																								// The outputs of the post processing algorithm and delayed RMS values
			if (sel == 0)
				*RMSout_rfvm = Buffer_RMS[0];
			else if (sel == 1)
				*RMSout_ham = Buffer_RMS[0];
			else
				*RMSout_vl = Buffer_RMS[0];
	}
	else																																// Initialize
	{
		state_1 = 0;																											// Set parameters to zero
		counter_1 = 0;
		state_2 = 0;
		counter_2 = 0;
		for (i = 0; i<Max_Postproc; i++)																					  // Set buffers to zero
		{	
			Buffer_1[i] = 0;
			Buffer_RMSrfvm[i] = 0;
			Buffer_2[i] = 0;
			Buffer_RMSham[i] = 0;
		}
		
	}
}

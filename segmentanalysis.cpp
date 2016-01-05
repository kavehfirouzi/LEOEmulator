#include "algorithms.h"
#include <fstream>
extern std::ofstream f1, f2, f3, f4, f5, f6, f6_s, f6_e, f6_2, X, Y, f8, ei, f7_e, f7_s;

unsigned int PrevRFVMPeak;																// Holds the value of the previous rfvm peak
unsigned int CurrentRFVMPeak;															// Holds the value of the current rfvm peak
unsigned int CurrentHamPeak;
extern int RFVMMVC;
extern int HamMVC;
extern int VLMVC;
extern int DebugCounter;
struct hamSegments hs[5];
/* Start and stop of rfvm and ham segments are detected by segmentanalysis algorithms
 * and flags are set. To compensate for the shift between EMG and acc data and to take care
 * of the different sampling rates of emg and acc, following buffers and pointer inside those
 * buffers are used
 */
uint8_t rfvmStartBuff[EMGACC];
uint8_t rfvmStopBuff[EMGACC];
uint8_t hamStartBuff[EMGACC];
uint8_t hamStopBuff[EMGACC];
int rfvmBuffPtr = 0;
int rfvmBuffPtr_delayed;
int hamBuffPtr = 0;
int hamBuffPtr_delayed;
int GyroRFVMS;
int GyroRFVME;
int GyroHamS;
int GyroHamE;
int GyroVLS;
int GyroVLE;
uint32_t segStartRFVM = 0;
uint32_t segStopRFVM = 0;
uint32_t segStartH = 0;
uint32_t segStopH = 0;
uint32_t segStartVL = 0;
uint32_t segStopVL = 0;
unsigned char EIRMSBuf[750];
int EIsize = 0;
int EIseg = 0;

/**
 * @brief	processing the segments to find the peak in each segment, find the intensity (average RMS value in the segment). Find coordination and targetting values. 
 *        For real time purposes, data is fed to the algorithm one at a time.
 * @param  in	         : input segmentation data, which could be either 0 or 1
 * @parami RMS         : Input RMS value
 * @param  cad         : Cadence value
 * @paramo RMSout_rfvm : delayed hamd RMS
 * @parami intensity   : intensity value inside each segment
 * @parami init  	     : if init = 0, initialize, else run the algorithm
 * @return	nothing
 * @detail         
 */	
 
void SegmetnAnalysisRFVM(float in, float RMS, float *effortRFVM, float *rawEffort, float *ratio, uint32_t globalCounter, int init)
{
	static int state = 0;									// state machine
	static int maxRMS = 0;
	static int counter = 0;    								// The countet to keep track of the length of the segment
	static float averageRFVMEffortBuf[3];						// Buffers to hold the last three effort values
	static float averageRatioBuf[3];						// Variable to hold the sum of non-normalized RMS valiues inside the segment
	static float sumNormalizedRMS = 0;
	static float hamEffort = 0;
	static float sumRMSHam = 0;								// Variable to hold the sum of normalized RMS valiues inside the segment
	static float sumRMSRFVM = 0;
	float maxHam;
	int rfvmStart, rfvmEnd;									// start and stop of rfvm segment flags
	float temp;
	int i;

	if (init == 1)											// If init = 1, initialize the parameters
	{
		state = 0;																										
		averageRFVMEffortBuf[0] = 0;
		averageRFVMEffortBuf[1] = 0;
		averageRFVMEffortBuf[2] = 0;
		averageRatioBuf[0] = 0;
		averageRatioBuf[1] = 0;
		averageRatioBuf[2] = 0;
	}
	else													// start Running the algorithms
	{
			rfvmStart = 0;
			rfvmEnd = 0;
			GyroRFVMS = 0;
			GyroRFVME = 0;
			switch(state)									// State Machine
			{
				case 0:										// Reset state
					sumRMSRFVM = 0;							// Sum of RMS values is set to 0
					maxRMS = 0;
					counter = 0;
					if (in == 1)							// If input is 1, a segment has started, go to running state, set the rfvmDetected flag to 1 so ham intensity can get updated
					{
						maxRMS = RMS;
						state = 1;
						rfvmStart = 1;
						GyroRFVMS = 1;
						segStartRFVM = globalCounter;
						EIRMSBuf[counter] = (RMS/RFVMMVC)*100;
					}
					break;
				case 1:
					counter++;								// If the input is 1, keep incrementing the counter to keep track of the length of the segment
					if (counter < 750)
						EIRMSBuf[counter] = (RMS/RFVMMVC)*100;
					sumRMSRFVM += RMS;						// Some of non-normalized values
					if (RMS > maxRMS)                       // As new RMS comes in, compare it with previous max, update if it's bigger
					{
						maxRMS = RMS;
						CurrentRFVMPeak = globalCounter;	// Keeping track of the peak
					}
					if (in == 0)							// If input is zero, segment has ended, next state will be the reset state.
					{
							state = 0;
						    rfvmEnd = 1;
							*rawEffort = sumRMSRFVM / counter;
							//*LT2Warning = SetAnaerobicFlag(*rawEffort, 0);
						    sumNormalizedRMS = (sumRMSRFVM / RFVMMVC);
							maxHam = 0;
							GyroRFVME = 1;
							segStopRFVM = globalCounter;
							EIsize = counter;
							EIseg = sumNormalizedRMS*100;
	
							temp = (sumNormalizedRMS / counter);                // Average the sum of RMS values by dividing it by the length of the segment and scale it to 255
							averageRFVMEffortBuf[0] = averageRFVMEffortBuf[1];
							averageRFVMEffortBuf[1] = averageRFVMEffortBuf[2];
							averageRFVMEffortBuf[2] = temp;						// Put the latest intensity value inside the averaging buffer
							temp = (averageRFVMEffortBuf[0] + averageRFVMEffortBuf[1] + averageRFVMEffortBuf[2])/3;
							//if (temp >= 2.5)										// Making sure effort doesn't exceed 100%
							//	temp = (float)2.5;							// fixed-point can't have the value of 1
							*effortRFVM = temp;
							//////////////////////////////
							averageRatioBuf[0] = averageRatioBuf[1];
							averageRatioBuf[1] = averageRatioBuf[2];
							sumRMSRFVM /= counter;
							if ((effortVLnonNormalized + sumRMSRFVM) != 0  &&  effortHamnonNormalized != 0)
								averageRatioBuf[2] = (effortVLnonNormalized + sumRMSRFVM)/((effortVLnonNormalized + sumRMSRFVM) + effortHamnonNormalized);
							temp = (averageRatioBuf[0] + averageRatioBuf[1] + averageRatioBuf[2])/3;
							if (temp >= 1)										// Making sure effort doesn't exceed 100%
								temp = (float)0.999;							// fixed-point can't have the value of 1
							*ratio = temp;
							//f5 << *ratio << std::endl;
							PrevRFVMPeak = CurrentRFVMPeak;
					}
					break;
			}
	}
}

/**
 * @brief	 Calculating cadence based on segmentation data, and intensity based on RMS of rfvm and ham. for real time purposes data is fed one at a time to the algorithm
 * @param  in	         : segmentation data
 * @param	 RMS_rfvm    : rfvm RMS
 * @param	 RMS_rfvm    : ham RMS
 * @param  cad				 : Pointer to cadence variable
 * @param  intensity_rfvm		: Pointer to rfvm intensity variable
 * @param  intensity_ham		: Pointer to ham intensity variable
 * @detail             : Distance between to consecutive segments is calculated. (From start of segment to start of the next segment). Cadence is the average over
 *                     : two of such distances. The function works as a state machine. It has two states. state 0 and state 1. When it's at 0 state and no segment has been detected, nothing happens.
 *                     : Otherwise if the input is 1, it means the beggining of a segmentation and it changes state to state 1 and starts counting. In this state a counter increments everytime with each 
 *                     : input with value 1 to keep track of the distance. If the input goes to zero in this state, it means the segment has ended. We go back to state 0. Now, since a segment has been detected, 
 *                     : state 0 acts differenty. It keeps incrementing the counter until input is 1, which means the start of another segment. Value of the counter is stored in the buffer as the distance. Counter is reset
 *										 : and the process repeats by going to state 1. Intensity is calculated as the average of RMS between to segments. (Start of one segment to the start of next). Latest
 *										 : two intensities are averaged for final result
 */	

void SegmetnAnalysisHam(float in, float RMS, float *effortHam, float *rawEffort, uint32_t globalCounter, int init)
{
	static int state = 0;																				// state machine
	static float maxRMS = 0;
	static float sumNormalizedRMS = 0;													// Sum of non-normalized RMS values
	static int counter = 0;																			// Counter to keep track of the length of the segment
	static float sumRMS = 0;																		// Sum of normalized RMS values
	int HamStart, HamEnd;																				// Start and stop of the segment flags
	float temp;
	static float averageHEffortBuf[3];
	int i;
	if (init == 1)
	{
		state = 0;
		averageHEffortBuf[0] = 0;
		averageHEffortBuf[1] = 0;
		averageHEffortBuf[2] = 0;
	}
	else
	{
			HamStart = 0;
			HamEnd = 0;
			GyroHamS = 0;
			GyroHamE = 0;
			switch(state)																						// State Machine
			{
				case 0:																								// Reset state
					sumRMS = 0;																					// Reset all variable
					counter = 0;
					maxRMS = 0;
					if (in == 1)																				// Start of the segment
					{
						HamStart = 1;																			// Set start start of ham flag
						maxRMS = RMS;
						state = 1;
						GyroHamS = 1;
						segStartH = globalCounter;
					}
					break;
				case 1:
					counter++;
					sumRMS += RMS;
					if (RMS > maxRMS)																		// Finding the maximum inside the session
					{
						maxRMS = RMS;
						CurrentHamPeak = globalCounter;
					}
					if (in == 0)																				// End of segment
					{
							state = 0;
							HamEnd = 1;																			// Set the end of ham flag
						    *rawEffort = sumRMS / counter;
							sumNormalizedRMS = (sumRMS / HamMVC);
							GyroHamE = 1;
							segStopH = globalCounter;
							averageHEffortBuf[0] = averageHEffortBuf[1];
							averageHEffortBuf[1] = averageHEffortBuf[2];
							averageHEffortBuf[2] = (sumRMS / HamMVC)/counter;
							temp = (averageHEffortBuf[0] + averageHEffortBuf[1] + averageHEffortBuf[2])/3;
							*effortHam = temp;
					}
					break;
			}
	}
}

void SegmetnAnalysisVL(float in, float RMS, float *rawEffort, float *vlEffort, uint32_t *LT2Warning, uint32_t globalCounter, int init)
{
	static int state = 0;																				// state machine
	static float maxRMS = 0;
	static float sumNormalizedRMS = 0;													// Sum of non-normalized RMS values
	static int counter = 0;																			// Counter to keep track of the length of the segment
	static float sumRMS = 0;																		// Sum of normalized RMS values																		// Start and stop of the segment flags
	int i;
	static float averageVLEffortBuf[3];
	float temp;
	if (init == 1)
	{
		state = 0;
		averageVLEffortBuf[0] = 0;
		averageVLEffortBuf[1] = 0;
		averageVLEffortBuf[2] = 0;
	}
	else
	{
		GyroVLS = 0;
		GyroVLE = 0;
		switch(state)																						// State Machine
		{
			case 0:																								// Reset state
				sumRMS = 0;																					// Reset all variable
				counter = 0;
				if (in == 1)
				{// Start of the segment
					state = 1;
					GyroVLS = 1;
					segStartVL = globalCounter;
				}
				break;
			case 1:
				counter++;
				sumRMS += RMS;
				if (in == 0)																				// End of segment
				{
					state = 0;
					GyroVLE = 1;
					segStopVL = globalCounter;
					*rawEffort = sumRMS / counter;
					*LT2Warning = SetAnaerobicFlag(*rawEffort, 0);
					averageVLEffortBuf[0] = averageVLEffortBuf[1];
					averageVLEffortBuf[1] = averageVLEffortBuf[2];
					averageVLEffortBuf[2] = (sumRMS / VLMVC)/counter;
					temp = (averageVLEffortBuf[0] + averageVLEffortBuf[1] + averageVLEffortBuf[2])/3;
					*vlEffort = temp;
				}
				break;
		}
	}
}

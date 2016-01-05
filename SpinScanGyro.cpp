#include "algorithms.h"
#include <fstream>
#include <math.h>

extern std::ofstream f1, f2, f3, f4, f5, f6, f6_s, f6_e, f6_2, X, Y, f8, ei, f7_e, f7_s, f8, f9, f10, f11, f12;

#define GYRO_SMOOTH 11
#define GYRO_WINDOW 125
#define GYRO_DELAY 54					//216(total EMG delay)/4 = 54

extern uint8_t  cadence_gyro;
extern int GyroRFVMS;
extern int GyroRFVME;
extern int GyroHamS;
extern int GyroHamE;
extern int GyroVLS;
extern int GyroVLE;
extern uint32_t global_counter;
//extern EMMC_METRICS_t EMMC_METRICS;
extern uint32_t segStartRFVM;
extern uint32_t segStopRFVM;
extern uint32_t segStartH;
extern uint32_t segStopH;
extern uint32_t segStartVL;
extern uint32_t segStopVL;
extern uint16_t MeanFreq_Q;
extern uint16_t MeanFreq_H;
extern unsigned char EIRMSBuf[750];
extern short segment1_Buff[55];
extern short segment2_Buff[55];
void LittletoBigEndian(uint8_t* Small, uint8_t* Big);

/**
 * @brief  pedal stroke analysis based on Gyro
 * @param  rawdata	   : pointer to buffer holding gyro values
 * @param  Results     : pointer to the bluetooth buffer
 * @parami LR   	   : left/right leg selection. The order of rfvm and ham or swaped inside the buffer for left and right leg.
 * @parami init  	   : if init = 0, initialize buffers and variables, else run the algorithm
 * @detail			   : zero crossing from negative to positive are taken as the starting and end point of each rep. First gyro is segmented and only the first zerocrossing is
 * 						 taken into account. length of the curve between two consecutive zerocrossing is calculated. value of the length of the curve from the start of the rep to start and end point of rfvm and ham
 * 						 segments are then divided by the length of the rep and converted to values from 0 to 359.
 * @return	nothing
 */

void SpinScanGyro(int32_t* rawData, BT4_Data_t *Results, int newGyro, int LR, int init)
{
	static int sz_prev = 0;
	static float delayBuff[GYRO_DELAY];
	static int delayBuff_2[GYRO_DELAY];
	static int ptr3 = 0;
	int ptr4;
	/////////////////////////////
	static int length = 0;
	static int current_length = 0;
	static int averageRFVM_s[3];
	static int averageRFVM_e[3];
	static int averageH_s[3];
	static int averageH_e[3];
	static int averageVL_s[3];
	static int averageVL_e[3];
	static int RepCount_RFVM = 0;
	static int RepCount_H = 0;
	static int RepCount_VL = 0;
	static int zerocross = 0;
	static int rfvm_start = 0;
	static int rfvm_end = 0;
	static int ham_start = 0;
	static int ham_end = 0;
	static int vl_start = 0;
	static int vl_end = 0;
	static int half_cycle = 0;
	static int cycle = 0;
	static int RFVMStart;
	static int HamStart;
	static int saveEMMC = 0;
	int SegmentGyro = 0;
	int angle;
	float max, min;
	int i;
	float g;
	int temp1, temp2, temp3, temp4;
	uint8_t tempq8[2];
	int gyroZ;
	int EI;
	static int counter = 0;
	static int startAngle = 0;
	static int endAngle = 0;
	int EIstart, EIend, fullcircle, EIsum;
	float f, anglestep, anglestep1, anglestep2;
	static int halfCircle_1, halfCircle_2;
	static int pos_peak = 0;
	static int neg_peak = 0;
	static float gyroAngle = 0;
	int pospeakdetected = 0;
	int negpeakdetected = 0;
	static int test_counter = 0;
	static int state_0 = 0;
	static int state_1 = 0;
	static int state_2 = 0;
	static int prevNegPeak = 0;
	static int currentNegPeak = 0;
	static int currentPosPeak = 0;
	static float Max_2, Max_1;
	static int RFVM_Start_Buff[3];
	static int RFVM_End_Buff[3];
	static int Ham_Start_Buff[3];
	static int Ham_End_Buff[3];
	static int VL_Start_Buff[3];
	static int VL_End_Buff[3];
	static int rfvmStartAngle = 0, rfvmStopAngle = 0, vlStartAngle = 0, vlStopAngle = 0; 
	uint16_t tempQ16;

	//These are not int LEO project
	static float RATIO = 0;
	float mytemp;
	if (init == 0 && newGyro == 1)
	{
		gyroZ = rawData[2];
		counter++;
		if (LR != 0)
			gyroZ *= -1;

		gyroAngle = (float)0.98*((float)gyroZ/32.8*(float)0.008 + gyroAngle);
		////////////////Delay Angle to sync with EMG/////////////////////
		delayBuff[ptr3] = gyroAngle;
		delayBuff_2[ptr3] = gyroZ;
		ptr4 = ptr3 + 1;
		if (ptr4 == GYRO_DELAY)
			ptr4 = 0;
		ptr3++;
		if (ptr3 == GYRO_DELAY)
			ptr3 = 0;
		////////////////////////////////////////////////////////////////
		{
			/*Checking for start and end of rfvm and ham segments flags, these flags are set by rfvm and ham segment analysis function
			and indicate the start and end of rfvm and ham segments.*/
			if (GyroRFVMS == 1)
			{
				rfvm_start = 1;
				GyroRFVMS = 0;
				RFVM_Start_Buff[0] = RFVM_Start_Buff[1];
				RFVM_Start_Buff[1] = RFVM_Start_Buff[2];
				RFVM_Start_Buff[2] = global_counter;
			}
			if (GyroRFVME == 1)													// End of rfvm segment, take the current length on the curve and update rfvm values in eemc buffer
			{
				rfvm_end = 1;
				GyroRFVME = 0;
				RFVM_End_Buff[0] = RFVM_End_Buff[1];
				RFVM_End_Buff[1] = RFVM_End_Buff[2];
				RFVM_End_Buff[2] = global_counter;
				LittletoBigEndian((uint8_t*)&segStartRFVM, (uint8_t*)&EMMC_METRICS.RFVMStartIndex);
				LittletoBigEndian((uint8_t*)&segStopRFVM, (uint8_t*)&EMMC_METRICS.RFVMStopIndex);
				tempq8[0] = Results->RFVMTypeI_Fatigue[1];
				tempq8[1] = Results->RFVMTypeI_Fatigue[0];
				memcpy(&EMMC_METRICS.RFVMTypeIFatigue, tempq8, 2);
				tempq8[0] = Results->RFVMTypeII_Fatigue[1];
				tempq8[1] = Results->RFVMTypeII_Fatigue[0];
				memcpy(&EMMC_METRICS.RFVMTypeIIFatigue, tempq8, 2);
				if (FTE_MODE == 1)
					LittletoBigEndian((uint8_t*)(&effortRFVMnonNormalized), (uint8_t*)&EMMC_METRICS.RFVMEffort);
				else
					LittletoBigEndian((uint8_t*)(&(effortRFVM)), (uint8_t*)&EMMC_METRICS.RFVMEffort);
				////
				RATIO = (float)(Results->RatioQ16[1]*256 + Results->RatioQ16[0])/256;
				////
				tempq8[0] = Results->RatioQ16[1];
				tempq8[1] = Results->RatioQ16[0];
				memcpy(EMMC_METRICS.Ratio, tempq8, 2);
			}
			if (GyroHamS == 1)													// Start of Ham segment, take the current length on the curve
			{
				ham_start = 1;
				GyroHamS = 0;
				Ham_Start_Buff[0] = Ham_Start_Buff[1];
				Ham_Start_Buff[1] = Ham_Start_Buff[2];
				Ham_Start_Buff[2] = global_counter;
			}
			if (GyroHamE == 1)													// End of ham segment, take the current length on the curve and update ham values in eemc buffer
			{
				ham_end = 1;
				GyroHamE = 0;
				Ham_End_Buff[0] = Ham_End_Buff[1];
				Ham_End_Buff[1] = Ham_End_Buff[2];
				Ham_End_Buff[2] = global_counter;
				LittletoBigEndian((uint8_t*)&segStartH, (uint8_t*)&EMMC_METRICS.HamStartIndex);
				LittletoBigEndian((uint8_t*)&segStopH, (uint8_t*)&EMMC_METRICS.HamStopIndex);
				tempq8[0] = Results->HamTypeI_Fatigue[1];
				tempq8[1] = Results->HamTypeI_Fatigue[0];
				memcpy(&EMMC_METRICS.HamTypeIFatigue, tempq8, 2);
				tempq8[0] = Results->HamTypeII_Fatigue[1];
				tempq8[1] = Results->HamTypeII_Fatigue[0];
				memcpy(&EMMC_METRICS.HamTypeIIFatigue, tempq8, 2);
				if (FTE_MODE == 1)
					LittletoBigEndian((uint8_t*)(&effortHamnonNormalized), (uint8_t*)&EMMC_METRICS.HamEffort);
				else
					LittletoBigEndian((uint8_t*)(&effortHam), (uint8_t*)&EMMC_METRICS.HamEffort);
			}
			if (GyroVLS == 1)
			{
				vl_start = 1;
				GyroVLS = 0;
				VL_Start_Buff[0] = VL_Start_Buff[1];
				VL_Start_Buff[1] = VL_Start_Buff[2];
				VL_Start_Buff[2] = global_counter;
			}
			if (GyroVLE == 1)													// End of rfvm segment, take the current length on the curve and update rfvm values in eemc buffer
			{
				vl_end = 1;
				GyroVLE = 0;
				VL_End_Buff[0] = VL_End_Buff[1];
				VL_End_Buff[1] = VL_End_Buff[2];
				VL_End_Buff[2] = global_counter;
				LittletoBigEndian((uint8_t*)&segStartVL, (uint8_t*)&EMMC_METRICS.VLStartIndex);
				LittletoBigEndian((uint8_t*)&segStopVL, (uint8_t*)&EMMC_METRICS.VLStopIndex);
				tempq8[0] = Results->VLTypeI_Fatigue[1];
				tempq8[1] = Results->VLTypeI_Fatigue[0];
				memcpy(&EMMC_METRICS.VLTypeIFatigue, tempq8, 2);
				tempq8[0] = Results->VLTypeII_Fatigue[1];
				tempq8[1] = Results->VLTypeII_Fatigue[0];
				memcpy(&EMMC_METRICS.VLTypeIIFatigue, tempq8, 2);
				if (FTE_MODE == 1)
					LittletoBigEndian((uint8_t*)(&effortVLnonNormalized), (uint8_t*)&EMMC_METRICS.VLEffort);
				else
					LittletoBigEndian((uint8_t*)(&effortVL), (uint8_t*)&EMMC_METRICS.VLEffort);
			}

			switch(state_0)
			{
				case 0:
					if (sz_prev < 0 && delayBuff_2[ptr4] > 0)
					{
						state_0 = 1;
						saveEMMC = 1;
					}
					break;
				case 1:
					if (sz_prev > 0 && delayBuff_2[ptr4] < 0)
						state_0 = 0;
					break;
			}

			switch(state_2)
			{
				case 0:
					if (segment2_Buff[0] == 1)
					{
						state_2 = 1;
						Max_2 = -delayBuff[ptr4];
						neg_peak = global_counter;
					}
					break;
				case 1:
					if (-delayBuff[ptr4] > Max_2)
					{
						Max_2 = -delayBuff[ptr4];
						neg_peak = global_counter;
					}
					if (segment2_Buff[0] == 0)
					{
						currentNegPeak = neg_peak;
						state_2 = 0;
						//saveEMMC = 1;
						cycle = 1;
					}
					break;
			}

			switch(state_1)
			{
				case 0:
					if (segment1_Buff[0] == 1)
					{
						state_1 = 1;
						Max_1 = delayBuff[ptr4];
						pos_peak = global_counter;
					}
					break;
				case 1:
					if (delayBuff[ptr4] > Max_1)
					{
						Max_1 = delayBuff[ptr4];
						pos_peak = global_counter;
					}
					if (segment1_Buff[0] == 0)
					{
						currentPosPeak = pos_peak;
						state_1 = 0;
					}
					break;
			}

			if (rfvm_start == 1 && cycle != 0)
			{
				temp4 = 0;
				for (i = 0; i < 3; i++)
				{
					if (RFVM_Start_Buff[i] <= currentNegPeak && RFVM_Start_Buff[i] >= prevNegPeak)
					{
						temp4 = 1;
						temp1 = RFVM_Start_Buff[i];
					}
				}
				if (temp4 == 1)
				{
					if (temp1 <= currentPosPeak)
						angle = (float)abs(temp1-prevNegPeak)/abs(currentPosPeak-prevNegPeak)*180;
					else
						angle = (float)abs(temp1-currentPosPeak)/abs(currentNegPeak-currentPosPeak)*180 + 180;
					if (angle >= 360)
						angle = 0;
					if (RepCount_RFVM > 2)
					{
						averageRFVM_s[0] = averageRFVM_s[1];
						averageRFVM_s[1] = averageRFVM_s[2];
						averageRFVM_s[2] = angle;
					}
					if (RepCount_RFVM > 4)
					{
						temp1 = averageRFVM_s[0];
						temp2 = averageRFVM_s[1];
						temp3 = averageRFVM_s[2];
						if (abs(temp1 - temp2) >= 180)
								temp1 -= 360;
						angle = (temp1 +  temp2)/2;
						if (angle < 0)
								angle += 360;
						if (abs(angle - temp3) >= 180)
							angle -= 360;
						angle = (temp3 +  angle)/2;
						if (angle < 0)
							angle += 360;
					}
					if (RepCount_RFVM > 2)
					{
						Results->RFVM_StartAngle = angle >> 1;
						memcpy(Results->RFVM_StartAngle16, &angle, 2);
						rfvmStartAngle = angle;
					}
					tempq8[0] = Results->RFVM_StartAngle16[1];
					tempq8[1] = Results->RFVM_StartAngle16[0];
					memcpy(&EMMC_METRICS.RFVMStartAngle, tempq8, 2);
					rfvm_start = 0;
				}
			}
			if (rfvm_end == 1 && cycle != 0)
			{
				temp4 = 0;
				for (i = 0; i < 3; i++)
				{
					if (RFVM_End_Buff[i] <= currentNegPeak && RFVM_End_Buff[i] >= prevNegPeak)
					{
						temp4 = 1;
						temp1 = RFVM_End_Buff[i];
					}
				}
				if (temp4 == 1)
				{
					if (temp1 <= currentPosPeak)
						angle = (float)abs(temp1-prevNegPeak)/abs(currentPosPeak-prevNegPeak)*180;
					else
						angle = (float)abs(temp1-currentPosPeak)/abs(currentNegPeak-currentPosPeak)*180 + 180;
					if (angle >= 360)
						angle = 0;
					if (RepCount_RFVM > 2)
					{
						averageRFVM_e[0] = averageRFVM_e[1];
						averageRFVM_e[1] = averageRFVM_e[2];
						averageRFVM_e[2] = angle;
					}
					if (RepCount_RFVM > 4)
					{

						temp1 = averageRFVM_e[0];
						temp2 = averageRFVM_e[1];
						temp3 = averageRFVM_e[2];
						if (abs(temp1 - temp2) >= 180)
								temp1 -= 360;
						angle = (temp1 +  temp2)/2;
						if (angle < 0)
								angle += 360;
						if (abs(angle - temp3) >= 180)
							angle -= 360;
						angle = (temp3 +  angle)/2;
						if (angle < 0)
							angle += 360;
					}
					RepCount_RFVM++;
					if (RepCount_RFVM > 2)											// Updating bluetooth and emmc buffer with angle
					{
						Results->RFVM_EndAngle = angle >> 1;
						memcpy(Results->RFVM_EndAngle16, &angle, 2);
						rfvmStopAngle = angle;
					}
					tempq8[0] = Results->RFVM_EndAngle16[1];
					tempq8[1] = Results->RFVM_EndAngle16[0];
					memcpy(&EMMC_METRICS.RFVMStopAngle, tempq8, 2);
					rfvm_end = 0;
				}
			}
			if (ham_start == 1 && cycle != 0)
			{
				temp4 = 0;
				for (i = 0; i < 3; i++)
				{
					if (Ham_Start_Buff[i] <= currentNegPeak && Ham_Start_Buff[i] >= prevNegPeak)
					{
						temp4 = 1;
						temp1 = Ham_Start_Buff[i];
					}
				}
				if (temp4 == 1)
				{
					if (temp1 <= currentPosPeak)
						angle = (float)abs(temp1-prevNegPeak)/abs(currentPosPeak-prevNegPeak)*180;
					else
						angle = (float)abs(temp1-currentPosPeak)/abs(currentNegPeak-currentPosPeak)*180 + 180;
					if (angle >= 360)
						angle = 0;
					if (RepCount_H > 2)
					{
						averageH_s[0] = averageH_s[1];
						averageH_s[1] = averageH_s[2];
						averageH_s[2] = angle;
					}
					if (RepCount_H > 4)
					{
						temp1 = averageH_s[0];
						temp2 = averageH_s[1];
						temp3 = averageH_s[2];
						if (abs(temp1 - temp2) >= 180)
								temp1 -= 360;
						angle = (temp1 +  temp2)/2;
						if (angle < 0)
								angle += 360;
						if (abs(angle - temp3) >= 180)
							angle -= 360;
						angle = (temp3 +  angle)/2;
						if (angle < 0)
							angle += 360;
					}
					if (RepCount_H > 2)												// Updating bluetooth and emmc buffer with angle
					{
						  Results->Ham_StartAngle = angle >> 1;
						  memcpy(Results->Ham_StartAngle16, &angle, 2);
					}
					tempq8[0] = Results->Ham_StartAngle16[1];
					tempq8[1] = Results->Ham_StartAngle16[0];
					memcpy(&EMMC_METRICS.HamStartAngle, tempq8, 2);
					ham_start = 0;
				}
			}
			if (ham_end == 1 && cycle != 0)
			{
				temp4 = 0;
				for (i = 0; i < 3; i++)
				{
					if (Ham_End_Buff[i] <= currentNegPeak && Ham_End_Buff[i] >= prevNegPeak)
					{
						temp4 = 1;
						temp1 = Ham_End_Buff[i];
					}
				}
				if (temp4 == 1)
				{
					if (temp1 <= currentPosPeak)
						angle = (float)abs(temp1-prevNegPeak)/abs(currentPosPeak-prevNegPeak)*180;
					else
						angle = (float)abs(temp1-currentPosPeak)/abs(currentNegPeak-currentPosPeak)*180 + 180;
					if (angle >= 360)
						angle = 0;

					if (RepCount_H > 2)
					{
						averageH_e[0] = averageH_e[1];
						averageH_e[1] = averageH_e[2];
						averageH_e[2] = angle;
					}
					if (RepCount_H > 4)
					{

						temp1 = averageH_e[0];
						temp2 = averageH_e[1];
						temp3 = averageH_e[2];
						if (abs(temp1 - temp2) >= 180)
								temp1 -= 360;
						angle = (temp1 +  temp2)/2;
						if (angle < 0)
								angle += 360;
						if (abs(angle - temp3) >= 180)
							angle -= 360;
						angle = (temp3 +  angle)/2;
						if (angle < 0)
							angle += 360;
					}
					RepCount_H++;
					if (RepCount_H > 2)												// Updating bluetooth and emmc buffer with angle
					{
						  Results->Ham_EndAngle = angle >> 1;
						  memcpy(Results->Ham_EndAngle16, &angle, 2);
					}
					tempq8[0] = Results->Ham_EndAngle16[1];
					tempq8[1] = Results->Ham_EndAngle16[0];
					memcpy(&EMMC_METRICS.HamStopAngle, tempq8, 2);
					ham_end = 0;
				}
			}
			if (vl_start == 1 && cycle != 0)
			{
				temp4 = 0;
				for (i = 0; i < 3; i++)
				{
					if (VL_Start_Buff[i] <= currentNegPeak && VL_Start_Buff[i] >= prevNegPeak)
					{
						temp4 = 1;
						temp1 = VL_Start_Buff[i];
					}
				}
				if (temp4 == 1)
				{
					if (temp1 <= currentPosPeak)
						angle = (float)abs(temp1-prevNegPeak)/abs(currentPosPeak-prevNegPeak)*180;
					else
						angle = (float)abs(temp1-currentPosPeak)/abs(currentNegPeak-currentPosPeak)*180 + 180;
					if (angle >= 360)
						angle = 0;
					if (RepCount_VL > 2)
					{
						averageVL_s[0] = averageVL_s[1];
						averageVL_s[1] = averageVL_s[2];
						averageVL_s[2] = angle;
					}
					if (RepCount_VL > 4)
					{
						temp1 = averageVL_s[0];
						temp2 = averageVL_s[1];
						temp3 = averageVL_s[2];
						if (abs(temp1 - temp2) >= 180)
								temp1 -= 360;
						angle = (temp1 +  temp2)/2;
						if (angle < 0)
								angle += 360;
						if (abs(angle - temp3) >= 180)
							angle -= 360;
						angle = (temp3 +  angle)/2;
						if (angle < 0)
							angle += 360;
					}
					if (RepCount_VL > 2)
					{
						Results->VL_StartAngle = angle >> 1;
						memcpy(Results->VL_StartAngle16, &angle, 2);
						vlStartAngle = angle;
					}
					tempq8[0] = Results->VL_StartAngle16[1];
					tempq8[1] = Results->VL_StartAngle16[0];
					memcpy(&EMMC_METRICS.VLStartAngle, tempq8, 2);
					vl_start = 0;
				}
			}
			if (vl_end == 1 && cycle != 0)
			{
				temp4 = 0;
				for (i = 0; i < 3; i++)
				{
					if (VL_End_Buff[i] <= currentNegPeak && VL_End_Buff[i] >= prevNegPeak)
					{
						temp4 = 1;
						temp1 = VL_End_Buff[i];
					}
				}
				if (temp4 == 1)
				{
					if (temp1 <= currentPosPeak)
						angle = (float)abs(temp1-prevNegPeak)/abs(currentPosPeak-prevNegPeak)*180;
					else
						angle = (float)abs(temp1-currentPosPeak)/abs(currentNegPeak-currentPosPeak)*180 + 180;
					if (angle >= 360)
						angle = 0;
					if (RepCount_VL > 2)
					{
						averageVL_e[0] = averageVL_e[1];
						averageVL_e[1] = averageVL_e[2];
						averageVL_e[2] = angle;
					}
					if (RepCount_VL > 4)
					{

						temp1 = averageVL_e[0];
						temp2 = averageVL_e[1];
						temp3 = averageVL_e[2];
						if (abs(temp1 - temp2) >= 180)
								temp1 -= 360;
						angle = (temp1 +  temp2)/2;
						if (angle < 0)
								angle += 360;
						if (abs(angle - temp3) >= 180)
							angle -= 360;
						angle = (temp3 +  angle)/2;
						if (angle < 0)
							angle += 360;
					}
					RepCount_VL++;
					if (RepCount_VL > 2)											// Updating bluetooth and emmc buffer with angle
					{
						Results->VL_EndAngle = angle >> 1;
						memcpy(Results->VL_EndAngle16, &angle, 2);
						vlStopAngle = angle;
					}
					tempq8[0] = Results->VL_EndAngle16[1];
					tempq8[1] = Results->VL_EndAngle16[0];
					memcpy(&EMMC_METRICS.VLStopAngle, tempq8, 2);
					vl_end = 0;
				}
			}
			if (cycle != 0)
			{
				cycle = 0;
				prevNegPeak = currentNegPeak;
			}

		  if (cadence == 0)											// If cadence became zero, zero out angle values and averaging buffers
		  {
				Results->Ham_EndAngle = 0;
				Results->RFVM_EndAngle = 0;
				Results->Ham_StartAngle = 0;
				Results->RFVM_StartAngle = 0;
				Results->Ham_EndAngle16[0] = 0; Results->Ham_EndAngle16[1] = 0;
				Results->Ham_StartAngle16[0] = 0; Results->Ham_StartAngle16[1] = 0;
				Results->RFVM_EndAngle16[0] = 0; Results->RFVM_EndAngle16[1] = 0;
				Results->RFVM_StartAngle16[0] = 0; Results->RFVM_StartAngle16[1] = 0;
				Results->VL_EndAngle16[0] = 0; Results->VL_EndAngle16[1] = 0;
				Results->VL_StartAngle16[0] = 0; Results->VL_StartAngle16[1] = 0;
				averageH_e[0] = 0; averageH_e[1] = 0; averageH_e[2] = 0;
				averageH_s[0] = 0; averageH_s[1] = 0; averageH_s[2] = 0;
				averageRFVM_e[0] = 0; averageRFVM_e[1] = 0; averageRFVM_e[2] = 0;
				averageRFVM_s[0] = 0; averageRFVM_s[1] = 0; averageRFVM_s[2] = 0;
				averageVL_e[0] = 0; averageVL_e[1] = 0; averageVL_e[2] = 0;
				averageVL_s[0] = 0; averageVL_s[1] = 0; averageVL_s[2] = 0;
				RepCount_RFVM = 0;
				RepCount_H = 0;
				RepCount_VL = 0;
				rfvmStartAngle = 0; rfvmStopAngle = 0; vlStartAngle = 0; vlStopAngle = 0;
		  }
		  if (saveEMMC == 1 && cadence != 0)
		  {
			    saveEMMC = 0;
			  	//Reps++;
			  	//BT_repUpdate = 1;
				temp1 = 0;
				//temp1 |= ((Reps & 0xff) << 8) | ((Reps & 0xff00) >> 8);
				//memcpy(&Results->Reps[0], (uint8_t*)&temp1, 2);
				//temp1 = AccSampleCounter*8;
				//LittletoBigEndian((uint8_t*)&AccSampleCounter, (uint8_t*)&EMMC_METRICS.RepIndex);
				//LittletoBigEndian((uint8_t*)&temp1, (uint8_t*)&EMMC_METRICS.Timestamp);
				//LittletoBigEndian((uint8_t*)&temp1, (uint8_t*)&(Results->Timestamp));
				memcpy(&EMMC_METRICS.Cadence, Results->Cadence, 2);
				memcpy(&EMMC_METRICS.RepCounter, Results->Reps, 2);
				if ((BT4_Data.Mode & 0x7f) == 0x03)
				  EMMC_METRICS.Activity[1] = Results->Activity;
				else
				  EMMC_METRICS.Activity[1] = Results->Mode & 0x7f;
				//f_write(&fp, &EMMC_METRICS, sizeof(EMMC_METRICS), &i);
				/////Emulation
				f5 << RATIO << std::endl;
				f4 << ((EMMC_METRICS.Cadence & 0x00ff)*256 + EMMC_METRICS.Cadence >> 8) << std::endl;
				f6 << (EMMC_METRICS.RFVMStartAngle[0]*256 + EMMC_METRICS.RFVMStartAngle[1]) << "\t" << (EMMC_METRICS.RFVMStopAngle[0]*256 + EMMC_METRICS.RFVMStopAngle[1]) << std::endl;
				f6_2 << (EMMC_METRICS.HamStartAngle[0]*256 + EMMC_METRICS.HamStartAngle[1]) << "\t" << (EMMC_METRICS.HamStopAngle[0]*256 + EMMC_METRICS.HamStopAngle[1]) << std::endl;
				f11 << (EMMC_METRICS.VLStartAngle[0]*256 + EMMC_METRICS.VLStartAngle[1]) << "\t" << (EMMC_METRICS.VLStopAngle[0]*256 + EMMC_METRICS.VLStopAngle[1]) << std::endl;

				temp1 = rfvmStartAngle;
				if (abs(rfvmStartAngle - vlStartAngle) >= 180)  //keep this paet in leo project
								temp1 -= 360;
						angle = (temp1 + vlStartAngle)/2;
				EMMC_METRICS.QuadStartAngle[0] = angle >> 8; EMMC_METRICS.QuadStartAngle[1] = (angle & 0xff);
				temp1 = rfvmStopAngle;
				if (abs(rfvmStopAngle - vlStopAngle) >= 180)
								temp1 -= 360;
						angle = (temp1 + vlStopAngle)/2;
				EMMC_METRICS.QuadStopAngle[0] = angle >> 8; EMMC_METRICS.QuadStopAngle[1] = (angle & 0xff);

				f12 << (EMMC_METRICS.QuadStartAngle[0]*256 + EMMC_METRICS.QuadStartAngle[1]) << "\t" << (EMMC_METRICS.QuadStopAngle[0]*256 + EMMC_METRICS.QuadStopAngle[1]) << std::endl;
				////
				LittletoBigEndian((uint8_t*)&EMMC_METRICS.RFVMEffort, (uint8_t*)&mytemp);
				f8 << mytemp << std::endl;
				LittletoBigEndian((uint8_t*)&EMMC_METRICS.HamEffort, (uint8_t*)&mytemp);
				f9 << mytemp << std::endl;
				LittletoBigEndian((uint8_t*)&EMMC_METRICS.VLEffort, (uint8_t*)&mytemp);
				f10 << mytemp << std::endl;
				/////
				LittletoBigEndian((uint8_t*)EMMC_METRICS.Ind, (uint8_t*)&i);
				i++;
				LittletoBigEndian((uint8_t*)&i, (uint8_t*)EMMC_METRICS.Ind);

				/*Update Efforts in BT buffers*/
				/*RFVM*/
				Results->rfvmEffort_f32 = effortRFVMnonNormalized;  //FTE mode
				Results->rfvmEffortNormalized_f32 = effortRFVM;     //This one is not used right now
				if (effortRFVM > 2.5)
					effortRFVM = 2.5;
				tempQ16 = (uint16_t)(effortRFVM*25600);
				memcpy(&Results->rfvmEffortQ16[0], &tempQ16, 2);    //Normal mode

				/*Ham*/
				Results->hamEffort_f32 = effortHamnonNormalized;	//FTE mode
				Results->hamEffortNormalized_f32 = effortHam;		//This one is not used right now
				if (effortHam > 2.5)
					effortHam = 2.5;
				tempQ16 = (uint16_t)(effortHam*25600);
				memcpy(&Results->hamEffortQ16[0], &tempQ16, 2);		//Normal mode

				/*VL*/
				Results->vlEffort_f32 = effortVLnonNormalized;		//FTE mode
				Results->vlEffortNormalized_f32 = effortVL;			//This one is not used right now
				if (effortVL > 2.5)
					effortVL = 2.5;
				tempQ16 = (uint16_t)(effortVL*25600);
				memcpy(&Results->vlEffortQ16[0], &tempQ16, 2);  	//Normal mode


		  }
		  //g = (float)(delayBuff[ptr4]-sz_prev)*(delayBuff[ptr4]-sz_prev) + (float)(6.4e-5);   // Update the length of the curve. 6.4e-5 = 1/125^2, 1/125 being the sampling rate
		  length++;// += sqrt(g);
		  sz_prev = delayBuff_2[ptr4];
		}
	}
	else if (init == 0 && newGyro == 0)										// There is now new gyro data, but to compensate for different sampling rates, keep checking for start and stop of segment flags
	{
		if (GyroRFVMS == 1)													// Start of rfvm segment, take the current length on the curve
		{
			rfvm_start = 1;
			GyroRFVMS = 0;
			RFVM_Start_Buff[0] = RFVM_Start_Buff[1];
			RFVM_Start_Buff[1] = RFVM_Start_Buff[2];
			RFVM_Start_Buff[2] = global_counter;
		}
		if (GyroRFVME == 1)													// End of rfvm segment, take the current length on the curve and update rfvm values in eemc buffer
		{
			rfvm_end = 1;
			GyroRFVME = 0;
			RFVM_End_Buff[0] = RFVM_End_Buff[1];
			RFVM_End_Buff[1] = RFVM_End_Buff[2];
			RFVM_End_Buff[2] = global_counter;
			LittletoBigEndian((uint8_t*)&segStartRFVM, (uint8_t*)&EMMC_METRICS.RFVMStartIndex);
			LittletoBigEndian((uint8_t*)&segStopRFVM, (uint8_t*)&EMMC_METRICS.RFVMStopIndex);
			tempq8[0] = Results->RFVMTypeI_Fatigue[1];
			tempq8[1] = Results->RFVMTypeI_Fatigue[0];
			memcpy(&EMMC_METRICS.RFVMTypeIFatigue, tempq8, 2);
			tempq8[0] = Results->RFVMTypeII_Fatigue[1];
			tempq8[1] = Results->RFVMTypeII_Fatigue[0];
			memcpy(&EMMC_METRICS.RFVMTypeIIFatigue, tempq8, 2);
			if (FTE_MODE == 1)
				LittletoBigEndian((uint8_t*)(&effortRFVMnonNormalized), (uint8_t*)&EMMC_METRICS.RFVMEffort);
			else
				LittletoBigEndian((uint8_t*)(&(effortRFVM)), (uint8_t*)&EMMC_METRICS.RFVMEffort);
			////
				RATIO = (float)(Results->RatioQ16[1]*256 + Results->RatioQ16[0])/256;
			////
			tempq8[0] = Results->RatioQ16[1];
			tempq8[1] = Results->RatioQ16[0];
			memcpy(EMMC_METRICS.Ratio, tempq8, 2);
		}
		if (GyroHamS == 1)													// Start of Ham segment, take the current length on the curve
		{
			ham_start = 1;
			GyroHamS = 0;
			Ham_Start_Buff[0] = Ham_Start_Buff[1];
			Ham_Start_Buff[1] = Ham_Start_Buff[2];
			Ham_Start_Buff[2] = global_counter;
		}
		if (GyroHamE == 1)													// End of ham segment, take the current length on the curve and update ham values in eemc buffer
		{
			ham_end = 1;
			GyroHamE = 0;
			Ham_End_Buff[0] = Ham_End_Buff[1];
			Ham_End_Buff[1] = Ham_End_Buff[2];
			Ham_End_Buff[2] = global_counter;
			LittletoBigEndian((uint8_t*)&segStartH, (uint8_t*)&EMMC_METRICS.HamStartIndex);
			LittletoBigEndian((uint8_t*)&segStopH, (uint8_t*)&EMMC_METRICS.HamStopIndex);
			tempq8[0] = Results->HamTypeI_Fatigue[1];
			tempq8[1] = Results->HamTypeI_Fatigue[0];
			memcpy(&EMMC_METRICS.HamTypeIFatigue, tempq8, 2);
			tempq8[0] = Results->HamTypeII_Fatigue[1];
			tempq8[1] = Results->HamTypeII_Fatigue[0];
			memcpy(&EMMC_METRICS.HamTypeIIFatigue, tempq8, 2);
			if (FTE_MODE == 1)
				LittletoBigEndian((uint8_t*)(&effortHamnonNormalized), (uint8_t*)&EMMC_METRICS.HamEffort);
			else
				LittletoBigEndian((uint8_t*)(&effortHam), (uint8_t*)&EMMC_METRICS.HamEffort);
		}
		if (GyroVLS == 1)
		{
			vl_start = 1;
			GyroVLS = 0;
			VL_Start_Buff[0] = VL_Start_Buff[1];
			VL_Start_Buff[1] = VL_Start_Buff[2];
			VL_Start_Buff[2] = global_counter;
		}
		if (GyroVLE == 1)													// End of rfvm segment, take the current length on the curve and update rfvm values in eemc buffer
		{
			vl_end = 1;
			GyroVLE = 0;
			VL_End_Buff[0] = VL_End_Buff[1];
			VL_End_Buff[1] = VL_End_Buff[2];
			VL_End_Buff[2] = global_counter;
			LittletoBigEndian((uint8_t*)&segStartVL, (uint8_t*)&EMMC_METRICS.VLStartIndex);
			LittletoBigEndian((uint8_t*)&segStopVL, (uint8_t*)&EMMC_METRICS.VLStopIndex);
			tempq8[0] = Results->VLTypeI_Fatigue[1];
			tempq8[1] = Results->VLTypeI_Fatigue[0];
			memcpy(&EMMC_METRICS.VLTypeIFatigue, tempq8, 2);
			tempq8[0] = Results->VLTypeII_Fatigue[1];
			tempq8[1] = Results->VLTypeII_Fatigue[0];
			memcpy(&EMMC_METRICS.VLTypeIIFatigue, tempq8, 2);
			if (FTE_MODE == 1)
				LittletoBigEndian((uint8_t*)(&effortVLnonNormalized), (uint8_t*)&EMMC_METRICS.VLEffort);
			else
				LittletoBigEndian((uint8_t*)(&effortVL), (uint8_t*)&EMMC_METRICS.VLEffort);
		}
	}
	else
	{
		current_length = 0;
		//Reps = 0;
		gyroAngle = 0;
		counter = 0;
		RepCount_H = 0;
		RepCount_RFVM = 0;
		RepCount_VL = 0;
		state_0 = 0;
		state_1 = 0;
		state_2 = 0;
		prevNegPeak = 0;
		currentNegPeak = 0;
		currentPosPeak = 0;
	}
}
/*
 * Changing a 4-byte variable from small-endian to big-endian or vice versa
 */
void LittletoBigEndian(uint8_t* Small, uint8_t* Big)
{
	Big[0] = Small[3];
	Big[1] = Small[2];
	Big[2] = Small[1];
	Big[3] = Small[0];
}



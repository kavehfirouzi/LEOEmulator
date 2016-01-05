#include "algorithms.h"

#define SEG_WINDOW_LENGHT_GYRO 125
#define GYRO_DELAY 63
#define MINIMUMANGLE_THRESHOLD 15
#define GYRO_DIVISION_FACTOR 7
#define GYRO_DIVISION_FACTOR_2 5

void findDistance_Gyro(int in, int* out, int init);
short segment1_Buff[55];
short segment2_Buff[55];
extern int GyroZangle;
extern int GyroSegmentation;

/**
 * @brief  Gyro based cadence. Cadence is calculated by means of segmenting angle form Z-axis gyro
 * @param  *rawdata	   : pointer to raw gyro values
 * @param	Results    : pointer to bluetooth buffer
 * @param   LR		   : selects to run algorithm for left or right leg
 * @parami init  	   : if init = 0, initialize, else run the algorithm
 * @return	nothing
 */
void cadenceGyro(int32_t* rawData, BT4_Data_t *Results, int LR, int init)
{
	static float GyroDelay_Buf[GYRO_DELAY];							// Buffer to delay gyro data to be in sync with action recognition algorithm
	static int32_t gyroPnt = 0;
	static int32_t gyroPnt_delay;
	float gyroZ_delayed;												// Delayed Z-axis gyro
	static float Angle = 0;
	float Max_1;
	float Min_1;
	float Max_2;
	float Min_2;
	static float MaxMin_Buf[SEG_WINDOW_LENGHT_GYRO];					// Buffer to hold angle values for segmentation
	static int MaxMin_pnt = 0;
	int gyroZ, gyroY;
	int temp;
	////////////////
	//static float S[2] = {1, 3};										// Kalman filter states (not used right now)
	//float s1, s2;
	int i;
	int Segment_1;
	int Segment_2;
	int Segment_3;
	static int Segment3_Buff[GYRO_DELAY];
	if (init == 0)
	{
		gyroZ = rawData[2];
		gyroY = rawData[1];
		if (LR != 0)													// Negate Z-axis for right leg
			gyroZ *= -1;

		Angle = (float)0.98*(gyroZ/(float)32.8*(float)0.008 + Angle);

		MaxMin_Buf[MaxMin_pnt] = Angle;
		Max_1 = Angle;
		Min_1 = Angle;
		Max_2 = -Angle;
		Min_2 = -Angle;
		for (i = 0; i <SEG_WINDOW_LENGHT_GYRO; i++)						// Segmenting based on max and min over the window
		{
			if (Max_1 < MaxMin_Buf[i])
				Max_1 = MaxMin_Buf[i];
			if (Min_1 > MaxMin_Buf[i])
				Min_1 = MaxMin_Buf[i];
			if (Max_2 < -MaxMin_Buf[i])
				Max_2 = -MaxMin_Buf[i];
			if (Min_2 > -MaxMin_Buf[i])
				Min_2 = -MaxMin_Buf[i];
		}
		MaxMin_pnt++;
		if (MaxMin_pnt == SEG_WINDOW_LENGHT_GYRO)
			MaxMin_pnt = 0;
		if (Angle > (Max_1-Min_1)/GYRO_DIVISION_FACTOR + Min_1)   			// comparing angle with the calculated threshold for segmentation
			Segment_1 = 1;
		else
			Segment_1 = 0;
		if (-Angle > (Max_2-Min_2)/GYRO_DIVISION_FACTOR + Min_2)																		// Classification of accelerometer values
			Segment_2 = 1;
		else
			Segment_2 = 0;
		if (Angle > (Max_1-Min_1)/GYRO_DIVISION_FACTOR_2 + Min_1)   			// comparing angle with the calculated threshold for segmentation
			Segment_3 = 1;
		else
			Segment_3 = 0;
		for (i = 0; i < 54; i++)
		{
			segment1_Buff[i] = segment1_Buff[i+1];
			segment2_Buff[i] = segment2_Buff[i+1];
		}
		segment1_Buff[54] = Segment_1;
		segment2_Buff[54] = Segment_2;
		///////For Activity Recognition///////
		GyroDelay_Buf[gyroPnt] = Angle;
		Segment3_Buff[gyroPnt] = Segment_3;
		gyroPnt_delay = gyroPnt + 1;
		if (gyroPnt_delay == GYRO_DELAY)
			gyroPnt_delay = 0;
		gyroPnt++;
		if (gyroPnt == GYRO_DELAY)
			gyroPnt = 0;
		GyroZangle = GyroDelay_Buf[gyroPnt_delay];						// Setting global variables to be used by action recognition algorithm
		GyroSegmentation = Segment3_Buff[gyroPnt_delay];
		////////////////////////////////
		findDistance_Gyro(Segment_3, &cadence, 0);						// Sending segmentation data to be analyzed to calculate the distance between segments and cadence
		if (cadence > 300)												// Clip cadence value at 300
			cadence = 300;
		if (Max_1 - Min_1 < MINIMUMANGLE_THRESHOLD)							// To get rid of small vibrating movements, set a threshold on angle
			cadence = 0;
		Results->Cadence[0] = (cadence & 0xff00) >> 8;					// Update bluetooth buffer
		Results->Cadence[1] = cadence;
	}
	else																// Reset the algorithm variables and buffers
	{
		gyroPnt = 0;
		Angle = 0;
		//S[0] = 1;
		//S[1] = 3;
		findDistance_Gyro(0, 0, 1);
		cadence = 0;
		for (i = 0; i < GYRO_DELAY; i++)
			GyroDelay_Buf[i] = 0;
		for (i = 0; i<SEG_WINDOW_LENGHT_GYRO; i++)
			MaxMin_Buf[i] = 0;
	}
}

void findDistance_Gyro(int in, int* out, int init)
{
	static int state = 0;										  // State machine
	static int counter = 0;										  // Counter to keep track of distance between segments
	static int dists[2] = {0,0};								  // Buffer to hold distances for averaging. Everytime a new distance is calculated, oldest one is tossed out and new one replaces it
	static int segment_counter = 0;                               // Keeps track of the number of segments detected so far
	static int stop_counter = 0;								  // Counter to keep track of time since last segment was detected
	static int stop_counter_enable = 0;                           // Enables and disable the above counter
	static int StartStop = 0;
	int i;
	if (init == 0)
	{
		if (stop_counter == 375)                                  // If no segment has been detected for the past 375 samples (3 seconds), reset all buffers and variables
		{
			*out = 0;
			state = 0;
			counter = 0;
			segment_counter = 0;
			stop_counter = 0;
			stop_counter_enable = 0;
			for (i = 0; i < 2; i++)
			{
				dists[i] = 0;
			}
		}
		if (stop_counter_enable == 1)
					stop_counter++;
		switch(state)
		{
			case 0:
				counter++;
				if (in == 1)
				{
					StartStop = 0;
					state = 1;
				}
				break;
			case 1:
				StartStop++;
				counter++;
				if (in == 0)
				{
					if (segment_counter > 0)						// Calculate cadence if more than 1 segment have been detected so far
					{
						stop_counter = 0;							// Reset the stop_counter to zero
						stop_counter_enable = 1;                    // Reset counter only should start counting when at least two segments have been detected
						dists[0] = dists[1];
						dists[1] = counter - StartStop / 2;			// Putting the current distance in the buffer
						if (segment_counter < 2)					// If only one distance has been detected so far, output it as cadence
						{
							if (dists[1] != 0)
							{
								*out = 7500  / (dists[1]);
								//f_write(&Debug, out, 4, &dummy);
							}
						}
						else										// If more than one distances have been detected, start averaging over the last two
						{
							if ((dists[0] + dists[1]) != 0)
							{
								*out = 7500 * 2 / (dists[0] + dists[1]);
								//f_write(&Debug, out, 4, &dummy);
							}
						}
					}
					//Reps++;										// Reps are updated by Spin Scan algorithm now
					segment_counter++;
					counter = StartStop / 2;
					state = 0;
				}
				break;
		}
	}
	else															// Reset the variables and state
	{
		state = 0;
		counter = 0;
		dists[0] = 0;
		dists[1] = 0;
		segment_counter = 0;
		stop_counter = 0;
		stop_counter_enable = 0;
		StartStop = 0;
	}
}

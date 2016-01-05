#include "algorithms.h"
#include <math.h>
#include "ActionRecHMM.h"

#define ACTIVITY_WIN 125
#define MAX_BUFFER_SIZE 200
#define SMOOTH_SIZE 15
#define DELAY 56
#define pi (float)3.141592653589793

/*Functions to calculate accelerometer velocity*/
void velocityX(int x, int *v, int init);
void velocityY(int y, int *v, int init);
void velocityZ(int z, int *v, int init);
/*Function to smooth accelerometer data for all three axis*/
void SmoothAccelerometer(int x, int y, int z, int* out_x, int* out_y, int* out_z);
/*Functions to calculate feature vectors*/
int FeatureVec_3(float *vec, int acc_y, int seg, int init);
int FeatureVec_2(float *vec, int acc_y, int vel_y, int angle, int seg, int init);
int FeatureVec_1(float *vec, int acc_x, int acc_y, int vel_y, int seg, int init);
int FeatureVec_4(float *vec, int acc_x, int seg, int init);
/*SMV for walk/run/cycle*/
int SVM_WRC(float *feats_1, float *feats_2, float*feats_3);
/*SVM for Sitting/Standing cycling*/
int SVM_SiSt(float *feats);
/*Finding the next power of two for a number, used for fft*/
int nextPowerof2(int in);

int GyroZangle = 0;
int GyroYangle = 0;
int GyroSegmentation = 0;

/**
 * @brief  Activity recognition. Decides whether the user is walking, running or cycling
 * 					   : and if he/she is cycling, whether it's sitting or standing
 * @param  *rawdata	   : pointer to raw acc values, the required gyro data is provided
 * 					   : by cadenceGyro algorithm
 * @param  *Results    : pointer to bluetooth buffer
 * @param   LR		   : selects to run algorithm for left or right leg
 * @parami  init  	   : if init = 0, initialize, else run the algorithm
 * @returns	nothing
 */
void AvtiviyRecognition(int32_t* rawData,  BT4_Data_t *Results, int init, int LR)
{
	static int state = 0;
	int sx, sy, sz, vx, vy, vz;
	float vec1[4], vec2[4], vec3[4], vec4[5];
	static int counter = 0;
	int overflow;
	int crw, sist, x, y, z, temp;
	static int activity = 0;
	if (init == 0)
	{
		x = rawData[0];
		y = rawData[1];
		z = rawData[2];
		if (LR != 0)												// Accelerometer X needs to be flipped for right leg
			x *= -1;
		SmoothAccelerometer(x, y, z, &sx, &sy, &sz);
		velocityY(y, &vy, 0);
		velocityX(x, &vx, 0);
		counter++;													// This counter and the next statement are used to guarantee velocity us calculate exactly the same as Matlab
		if ( (counter & 0x1) != 0 || counter >= 125)
		{
			/////////////////////////////
			overflow = FeatureVec_3(vec3, sy, GyroSegmentation, 0);	// the amount of buffer reserved for each segment is limited (150 samples), overflow indicates that the segment is too long, and should be ignored
			FeatureVec_2(vec2, sy, vy, GyroZangle, GyroSegmentation, 0);
			FeatureVec_1(vec1, sx, sy, vy, GyroSegmentation, 0);
			FeatureVec_4(vec4, sx, GyroSegmentation, 0);
			vec4[0] = vec1[3];										// The first element of feature vector number 4 is the same as element 4 of vector 1, so just reuse it
			//f_write(&Debug, &GyroZangle, 4, &temp);
			switch(state)											// Use gyro based segmentation from the cadencegyro algorithm for the state machine
			{
				case 0:
					if (GyroSegmentation == 1)						// start of the session
						state = 1;
				break;
				case 1:
					if (GyroSegmentation == 0)						// end of the session, feed the SVMs with feature vectors
					{
						if (overflow == 0)
						{
							crw = SVM_WRC(vec1, vec2, vec3);
							sist = SVM_SiSt(vec4);
							if (crw == 4)
								activity = sist;
							else
								activity = crw;
						}
						state = 0;
					}
				break;
			}
		}
		Results->Activity = activity;
	}
	else															// initialize the buffers, variables and states
	{
		state = 0;
		activity = 0;
		velocityX(0, 0, 1);
		velocityY(0, 0, 1);
		velocityZ(0, 0, 1);
		FeatureVec_3(0, 0, 0, 1);
		FeatureVec_2(0, 0, 0, 0, 0, 1);
		FeatureVec_1(0, 0, 0, 0, 0, 1);
		FeatureVec_4(0, 0, 0, 1);
		counter = 0;
	}
}

int SVM_WRC(float *feats_1, float *feats_2, float *feats_3)
{
	static int ptr = 0;
	static int CycCount = 0;
	static int RunCount = 0;
	static int WalkCount = 0;
	static unsigned char ActBuff[5];
	int c, r, w;

	float s1_1 = Scale1_1; float s1_2 = Scale1_2; float s1_3 = Scale1_3;
	float s1_4 = Scale1_4;

	float sh1_1 = Shift1_1; float sh1_2 = Shift1_2; float sh1_3 = Shift1_3;
	float sh1_4 = Shift1_4;
	
	float HP1_1 = HPlane1_1; float HP1_2 = HPlane1_2; float HP1_3 = HPlane1_3;
	float HP1_4 = HPlane1_4;

	float Bias_1 = Bias1;

	float s2_1 = Scale2_1; float s2_2 = Scale2_2; float s2_3 = Scale2_3;
	float s2_4 = Scale2_4;

	float sh2_1 = Shift2_1; float sh2_2 = Shift2_2; float sh2_3 = Shift2_3;
	float sh2_4 = Shift2_4;
	
	float HP2_1 = HPlane2_1; float HP2_2 = HPlane2_2; float HP2_3 = HPlane2_3;
	float HP2_4 = HPlane2_4;

	float Bias_2 = Bias2;

	float s3_1 = Scale3_1; float s3_2 = Scale3_2; float s3_3 = Scale3_3;
	float s3_4 = Scale3_4;

	float sh3_1 = Shift3_1; float sh3_2 = Shift3_2; float sh3_3 = Shift3_3;
	float sh3_4 = Shift3_4;
	
	float HP3_1 = HPlane3_1; float HP3_2 = HPlane3_2; float HP3_3 = HPlane3_3;
	float HP3_4 = HPlane3_4;

	float Bias_3 = Bias3;


	float f1, f2, f3;
	int currentAct, OldestAct;

	f1 = HP1_1*(feats_1[0] + sh1_1)*s1_1 + HP1_2*(feats_1[1] + sh1_2)*s1_2 + HP1_3*(feats_1[2] + sh1_3)*s1_3 +
		 HP1_4*(feats_1[3] + sh1_4)*s1_4 + Bias_1;

	f2 = HP2_1*(feats_2[0] + sh2_1)*s2_1 + HP2_2*(feats_2[1] + sh2_2)*s2_2 + HP2_3*(feats_2[2] + sh2_3)*s2_3 +
		 HP2_4*(feats_2[3] + sh2_4)*s2_4 + Bias_2;
	
	f3 = HP3_1*(feats_3[0] + sh3_1)*s3_1 + HP3_2*(feats_3[1] + sh3_2)*s3_2 + HP3_3*(feats_3[2] + sh3_3)*s3_3 +
		 HP3_4*(feats_3[3] + sh3_4)*s3_4 + Bias_3;
	r = 0;
	w = 0;
	c = 0;
	if (f1 <= 0)
		r++;
	else
		w++;
	if (f2 <= 0)
		c++;
	else
		w++;
	if (f3 <= 0)
		c++;
	else
		r++;
	if (c == 2)
		currentAct = 3;
	else if (r == 2)
		currentAct = 2;
	else if (w == 2)
		currentAct = 1;
	else
		currentAct = 3;

	OldestAct = ActBuff[ptr];
	switch(OldestAct)
	{
	case 3:
		CycCount--;
		break;
	case 2:
		RunCount--;
		break;
	case 1:
		WalkCount--;
		break;
	}
	ActBuff[ptr] = currentAct;
	switch(currentAct)
	{
	case 3:
		CycCount++;
		break;
	case 2:
		RunCount++;
		break;
	case 1:
		WalkCount++;
		break;
	}
	ptr++;
	if (ptr == 5)
		ptr = 0;
	if (CycCount >= RunCount && CycCount >= WalkCount)
		return 4;
	else if  (RunCount >= CycCount && RunCount >= WalkCount)
		return 2;
	else if (WalkCount >= CycCount && WalkCount >= RunCount)
		return 1;
	else
		return 4;
}

int SVM_SiSt(float *feats)
{
	static int ptr = 0;
	static int sit = 0;
	static int stand = 0;
	static unsigned char ActBuff[5];
	int si, st;

	float s1 = ScaleSiSt_1; float s2 = ScaleSiSt_2; float s3 = ScaleSiSt_3;
	float s4 = ScaleSiSt_4; float s5 = ScaleSiSt_5;

	float sh1 = ShiftSiSt_1; float sh2 = ShiftSiSt_2; float sh3 = ShiftSiSt_3;
	float sh4 = ShiftSiSt_4; float sh5 = ShiftSiSt_5;

	float HP1 = HPlaneSiSt_1; float HP2 = HPlaneSiSt_2; float HP3 = HPlaneSiSt_3;
	float HP4 = HPlaneSiSt_4; float HP5 = HPlaneSiSt_5;



	float f1;
	int currentAct, OldestAct;

	f1 = HP1*(feats[0] + sh1)*s1 + HP2*(feats[1] + sh2)*s2 + HP3*(feats[2] + sh3)*s3 +
		 HP4*(feats[3] + sh4)*s4 + HP5*(feats[4] + sh5)*s5  + BiasSiSt;

	if (f1 <= 0)
		currentAct = 4;
	else
		currentAct = 3;

	OldestAct = ActBuff[ptr];
	switch(OldestAct)
	{
	case 4:
		sit--;
		break;
	case 3:
		stand--;
	}
	ActBuff[ptr] = currentAct;
	switch(currentAct)
	{
	case 4:
		sit++;
		break;
	case 3:
		stand++;
		break;
	}
	ptr++;
	if (ptr == 5)
		ptr = 0;
	if (sit >= stand)
		return 4;
	else
		return 3;
}

int FeatureVec_3(float *vec, int acc_y, int seg, int init)
{
	static int buffer[MAX_BUFFER_SIZE];
	static int ptr = 0;
	static int state;
	static int max;
	static int min;
	static int counter;
	static int overflow = 0;
	float cosine, sine, constcos, constsin;
	int i;
	float real = 0;
	float imag = 0;
	float temp1, temp2;
	int l;
	if (init == 0)
	{
		switch(state)
		{
			case 0:
				overflow = 0;
				counter = 0;
				ptr = 0;
				real = 0;
				imag = 0;
				if (seg == 1)
				{
					state = 1;
					max = acc_y;
					min = acc_y;
					counter = 1;
					buffer[ptr++] = acc_y;
					if (ptr == 150)
					{
						ptr = 0;
						overflow = 1;
					}
				}
			break;
			case 1:
				counter++;
				buffer[ptr++] = acc_y;
				if (ptr == 150)
				{
					ptr = 0;
					overflow = 1;
				}
				if (acc_y > max)
					max = acc_y;
				if (acc_y < min)
					min = acc_y;
				if (seg == 0)
				{
					state = 0;
					vec[0] = max;
					vec[1] = abs(max - min);
					l = nextPowerof2(counter-1);
					constcos = cosf(2*pi*5/(l));
					constsin = sinf(2*pi*5/(l));
					/////fft6//////
					cosine = 1;
					sine = 0;
					for (i = 0; i < counter-1; i++)
					{
						real += buffer[i]*cosine;
						imag -= buffer[i]*sine;
						temp1 = cosine*constcos - sine*constsin;
						temp2 = cosine*constsin + sine*constcos;
						cosine = temp1;
						sine = temp2;
					}
					vec[2] = sqrtf(real*real + imag*imag)/(counter-1);
					/////fft4//////
					constcos = cosf(2*pi*3/(l));
					constsin = sinf(2*pi*3/(l));
					cosine = 1;
					sine = 0;
					real = 0;
					imag = 0;
					for (i = 0; i < counter-1; i++)
					{
						real += buffer[i]*cosine;
						imag -= buffer[i]*sine;
						temp1 = cosine*constcos - sine*constsin;
						temp2 = cosine*constsin + sine*constcos;
						cosine = temp1;
						sine = temp2;
					}
					vec[3] = sqrtf(real*real + imag*imag)/(counter-1);
				}
		}
		return overflow;
	}
	else
	{
		state = 0;
		overflow = 0;
		counter = 0;
		ptr = 0;
		real = 0;
		imag = 0;
	}
}

int FeatureVec_2(float *vec, int acc_y, int vel_y, int angle, int seg, int init)
{
	static int buffer_vel[MAX_BUFFER_SIZE];
	static int ptr = 0;
	static int state;
	static int max_vel;
	static int min_vel;
	static int max_ang;
	static int min_ang;
	static int max_y;
	static int min_y;
	static int counter;
	static int overflow = 0;
	float cosine, sine, constcos, constsin;
	int i;
	float real = 0;
	float imag = 0;
	float temp1, temp2;
	int l;
	if (init == 0)
	{
		switch(state)
		{
			case 0:
				overflow = 0;
				counter = 0;
				ptr = 0;
				real = 0;
				imag = 0;
				if (seg == 1)
				{
					state = 1;
					max_vel = vel_y;
					min_vel = vel_y;
					max_ang = angle;
					min_ang = angle;
					max_y = acc_y;
					min_y = acc_y;
					counter = 1;
					buffer_vel[ptr++] = vel_y;
					if (ptr == 150)
					{
						ptr = 0;
						overflow = 1;
					}
				}
			break;
			case 1:
				counter++;
				buffer_vel[ptr++] = vel_y;
				if (ptr == 150)
				{
					ptr = 0;
					overflow = 1;
				}
				if (vel_y > max_vel)
					max_vel = vel_y;
				if (vel_y < min_vel)
					min_vel = vel_y;

				if (angle > max_ang)
					max_ang = angle;
				if (angle < min_ang)
					min_ang = angle;

				if (acc_y > max_y)
					max_y = acc_y;
				if (acc_y < min_y)
					min_y = acc_y;

				if (seg == 0)
				{
					state = 0;
					vec[1] = max_y;
					vec[2] = max_ang;
					vec[3] = max_vel;
					l = nextPowerof2(counter-1);
					constcos = cosf(2*pi*1/(l));
					constsin = sinf(2*pi*1/(l));
					/////fft2//////
					cosine = 1;
					sine = 0;
					for (i = 0; i < counter-1; i++)
					{
						real += buffer_vel[i]*cosine;
						imag -= buffer_vel[i]*sine;
						temp1 = cosine*constcos - sine*constsin;
						temp2 = cosine*constsin + sine*constcos;
						cosine = temp1;
						sine = temp2;
					}
					vec[0] = sqrtf(real*real + imag*imag)/(counter-1);
					/////////////////////////////
				}
				break;
		}
		return overflow;
	}
	else
	{
		state = 0;
		overflow = 0;
		counter = 0;
		ptr = 0;
		real = 0;
		imag = 0;
	}
}

int FeatureVec_1(float *vec, int acc_x, int acc_y, int vel_y, int seg, int init)
{
	static int state;
	static int buffer_acc[MAX_BUFFER_SIZE];
	static int ptr = 0;
	static int max_vel;
	static int min_vel;
	static int max_y;
	static int min_y;
	static int counter;
	static int overflow = 0;
	float cosine, sine, constcos, constsin;
	int i;
	float real = 0;
	float imag = 0;
	float temp1, temp2;
	int l;
	if (init == 0)
	{
		switch(state)
		{
			case 0:
				overflow = 0;
				counter = 0;
				ptr = 0;
				real = 0;
				imag = 0;
				if (seg == 1)
				{
					counter = 1;
					buffer_acc[ptr++] = acc_x;
					if (ptr == 150)
					{
						ptr = 0;
						overflow = 1;
					}
					state = 1;
					max_vel = vel_y;
					min_vel = vel_y;
					max_y = acc_y;
					min_y = acc_y;
				}
			break;
			case 1:
				if (vel_y > max_vel)
					max_vel = vel_y;
				if (vel_y < min_vel)
					min_vel = vel_y;

				if (acc_y > max_y)
					max_y = acc_y;
				if (acc_y < min_y)
					min_y = acc_y;

				counter++;
				buffer_acc[ptr++] = acc_x;
				if (ptr == 150)
				{
					ptr = 0;
					overflow = 1;
				}
				if (seg == 0)
				{
					state = 0;
					vec[0] = max_y;
					vec[1] = abs(max_y - min_y);
					vec[2] = abs(max_vel - min_vel);
					l = nextPowerof2(counter-1);
					constcos = cosf(2*pi*1/(l));
					constsin = sinf(2*pi*1/(l));
					/////fft2//////
					cosine = 1;
					sine = 0;
					for (i = 0; i < counter-1; i++)
					{
						real += buffer_acc[i]*cosine;
						imag -= buffer_acc[i]*sine;
						temp1 = cosine*constcos - sine*constsin;
						temp2 = cosine*constsin + sine*constcos;
						cosine = temp1;
						sine = temp2;
					}
					vec[3] = sqrtf(real*real + imag*imag)/(counter-1);
				}
				break;
		}
		return overflow;
	}
	else
	{
		state = 0;
		overflow = 0;
		counter = 0;
		ptr = 0;
		real = 0;
		imag = 0;
	}
}

int FeatureVec_4(float *vec, int acc_x, int seg, int init)
{
	static int buffer_acc[150];
	static int ptr = 0;
	static int state;
	static int max_acc, min_acc;
	static int counter;
	static int overflow = 0;
	float cosine, sine, constsin, constcos;
	int i;
	float real = 0;
	float imag = 0;
	float temp1, temp2;
	int l;
	if (init == 0)
	{
		switch(state)
		{
			case 0:
				overflow = 0;
				counter = 0;
				ptr = 0;
				real = 0;
				imag = 0;
				if (seg == 1)
				{
					state = 1;
					max_acc = acc_x;
					min_acc = acc_x;
					counter = 1;
					buffer_acc[ptr++] = acc_x;
					if (ptr == 150)
					{
						ptr = 0;
						overflow = 1;
					}
				}
			break;
			case 1:
				counter++;
				buffer_acc[ptr++] = acc_x;
				if (ptr == 150)
				{
					ptr = 0;
					overflow = 1;
				}
				if (acc_x > max_acc)
					max_acc = acc_x;
				if (acc_x < min_acc)
					min_acc = acc_x;

				if (seg == 0)
				{
					state = 0;
					vec[1] = max_acc;
					vec[4] = abs(max_acc - min_acc);
					l = nextPowerof2(counter-1);
					/////fft4//////
					constcos = cosf(2*pi*3/(l));
					constsin = sinf(2*pi*3/(l));
					cosine = 1;
					sine = 0;
					for (i = 0; i < counter-1; i++)
					{
						real += buffer_acc[i]*cosine;
						imag -= buffer_acc[i]*sine;
						temp1 = cosine*constcos - sine*constsin;
						temp2 = cosine*constsin + sine*constcos;
						cosine = temp1;
						sine = temp2;
					}
					vec[2] = sqrtf(real*real + imag*imag)/(counter-1);
					/////fft5//////
					constcos = cosf(2*pi*4/(l));
					constsin = sinf(2*pi*4/(l));
					cosine = 1;
					sine = 0;
					for (i = 0; i < counter-1; i++)
					{
						real += buffer_acc[i]*cosine;
						imag -= buffer_acc[i]*sine;
						temp1 = cosine*constcos - sine*constsin;
						temp2 = cosine*constsin + sine*constcos;
						cosine = temp1;
						sine = temp2;
					}
					vec[3] = sqrtf(real*real + imag*imag)/(counter-1);
				}
		}
		return overflow;
	}
	else
	{
		state = 0;
		overflow = 0;
		counter = 0;
		ptr = 0;
		real = 0;
		imag = 0;
	}
}

void velocityX(int x, int *v, int init)
{
	static float Buffer_1[ACTIVITY_WIN];
	static float sum_1 = 0;
	static float high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
	static float cumsum_1 = 0;
	static int counter = 0;
	if (init == 0)
	{
		if (counter < ACTIVITY_WIN)                             // This condition is used to guarantee velocity is calculated exactly the same as Matlab
		{
			sum_1 += x;
			Buffer_1[ptr1_1] = x;
			counter++;
			if ( (counter & 0x1) != 0)
			{
				high_1 = Buffer_1[counter/2] - (sum_1)/counter;
				cumsum_1 += high_1;
				*v = cumsum_1;
			}
			(ptr1_1)++;
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			
		}
		else
		{
			sum_1 = sum_1 - Buffer_1[ptr1_1] + x;
			Buffer_1[ptr1_1] = x;
			high_1 = Buffer_1[ptr1_2] - (sum_1)/ACTIVITY_WIN;
			cumsum_1 += high_1;
			(ptr1_1)++;
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			*v = cumsum_1;
		}
		
	}
	else
	{
		counter = 0;
		cumsum_1 = 0;
		ptr1_1 = 0;
		ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
		sum_1 = 0;
	}
}


void velocityY(int y, int *v, int init)
{
	static float Buffer_1[ACTIVITY_WIN];
	static float sum_1 = 0;
	static float high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
	static float cumsum_1 = 0;
	static int counter = 0;
	if (init == 0)
	{
		if (counter < ACTIVITY_WIN)								// This condition is used to guarantee velocity is calculated exactly the same as Matlab
		{
			sum_1 += y;
			Buffer_1[ptr1_1] = y;
			counter++;
			if ( (counter & 0x1) != 0)
			{
				high_1 = Buffer_1[counter/2] - (sum_1)/counter;
				cumsum_1 += high_1;
				*v = cumsum_1;
			}
			(ptr1_1)++;
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			
		}
		else
		{
			sum_1 = sum_1 - Buffer_1[ptr1_1] + y;
			Buffer_1[ptr1_1] = y;
			high_1 = Buffer_1[ptr1_2] - (sum_1)/ACTIVITY_WIN;
			cumsum_1 += high_1;
			(ptr1_1)++;
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			*v = cumsum_1;
		}
		
	}
	else
	{
		counter = 0;
		cumsum_1 = 0;
		ptr1_1 = 0;
		ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
		sum_1 = 0;
	}
}

void velocityZ(int z, int *v, int init)
{
	static float Buffer_1[ACTIVITY_WIN];
	static float sum_1 = 0;
	static float high_1;
	static int ptr1_1 = 0;
	static int ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
	static float cumsum_1 = 0;
	static int counter = 0;
	if (init == 0)
	{
		if (counter < ACTIVITY_WIN)										// This condition is used to guarantee velocity is calculated exactly the same as Matlab
		{
			sum_1 += z;
			Buffer_1[ptr1_1] = z;
			counter++;
			if ( (counter & 0x1) != 0)
			{
				high_1 = Buffer_1[counter/2] - (sum_1)/counter;
				cumsum_1 += high_1;
				*v = cumsum_1;
			}
			(ptr1_1)++;
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			
		}
		else
		{
			sum_1 = sum_1 - Buffer_1[ptr1_1] + z;
			Buffer_1[ptr1_1] = z;
			high_1 = Buffer_1[ptr1_2] - (sum_1)/ACTIVITY_WIN;
			cumsum_1 += high_1;
			(ptr1_1)++;
			(ptr1_2)++;
			if ( (ptr1_1) == ACTIVITY_WIN )																
				ptr1_1 = 0;
			if ( (ptr1_2) == ACTIVITY_WIN )
				ptr1_2 = 0;
			*v = cumsum_1;
		}
		
	}
	else
	{
		counter = 0;
		cumsum_1 = 0;
		ptr1_1 = 0;
		ptr1_2 = (ACTIVITY_WIN - 1)/2 + 1;
		sum_1 = 0;
	}
}


void SmoothAccelerometer(int x, int y, int z, int* out_x, int* out_y, int* out_z)
{
	static int Buffer_1_1[SMOOTH_SIZE];
	//static int Buffer_1_2[SMOOTH_SIZE];
	//static int Buffer_1_3[SMOOTH_SIZE];
	static int sum_1_1 = 0;
	//static int sum_1_2 = 0;
	//static int sum_1_3 = 0;
	static int delayBufferX[DELAY];
	static int ptrDelayX_1 = 0;
	static int ptrDelayX_2;
	static float Buffer_2_1[SMOOTH_SIZE];
	//static float Buffer_2_2[SMOOTH_SIZE];
	//static float Buffer_2_3[SMOOTH_SIZE];
	static float sum_2_1 = 0;
	//static float sum_2_2 = 0;
	//static float sum_2_3 = 0;
	static float delayBufferY[DELAY];
	static int ptrDelayY_1 = 0;
	static int ptrDelayY_2;
	static int Buffer_3_1[SMOOTH_SIZE];
	//static int Buffer_3_2[SMOOTH_SIZE];
	//static int Buffer_3_3[SMOOTH_SIZE];
	static int sum_3_1 = 0;
	//static int sum_3_2 = 0;
	//static int sum_3_3 = 0;
	static int delayBufferZ[DELAY];
	static int ptrDelayZ_1 = 0;
	static int ptrDelayZ_2;
	static int ptr_1 = 0;
	static int ptr_2 = 0;
	static int ptr_3 = 0;
	float f_1;
	/////////////////////////////////////
	sum_1_1 = sum_1_1 - Buffer_1_1[ptr_1] + x;
	Buffer_1_1[ptr_1] = x;
	f_1 = sum_1_1 / SMOOTH_SIZE;

	/*sum_1_2 = sum_1_2 - Buffer_1_2[ptr_1] + f_1;
	Buffer_1_2[ptr_1] = f_1;
	f_1 = sum_1_2 / SMOOTH_SIZE;

	sum_1_3 = sum_1_3 - Buffer_1_3[ptr_1] + f_1;
	Buffer_1_3[ptr_1] = f_1;
	f_1 = sum_1_3 / SMOOTH_SIZE;*/
	
	delayBufferX[ptrDelayX_1] = f_1;
	ptrDelayX_2 = ptrDelayX_1 + 1;
	if (ptrDelayX_2 >= DELAY)
		ptrDelayX_2 -= DELAY;
	*out_x = delayBufferX[ptrDelayX_2];
	/////////////////////////////////////
	
	sum_2_1 = sum_2_1 - Buffer_2_1[ptr_1] + y;
	Buffer_2_1[ptr_1] = y;
	f_1 = sum_2_1 / SMOOTH_SIZE;

	/*sum_2_2 = sum_2_2 - Buffer_2_2[ptr_1] + f_1;
	Buffer_2_2[ptr_1] = f_1;
	f_1 = sum_2_2 / SMOOTH_SIZE;

	sum_2_3 = sum_2_3 - Buffer_2_3[ptr_1] + f_1;
	Buffer_2_3[ptr_1] = f_1;
	f_1 = sum_2_3 / SMOOTH_SIZE;*/

	delayBufferY[ptrDelayY_1] = f_1;
	ptrDelayY_2 = ptrDelayY_1 + 1;
	if (ptrDelayY_2 >= DELAY)
		ptrDelayY_2 -= DELAY;
	*out_y = delayBufferY[ptrDelayY_2];
	/////////////////////////////////////
	
	sum_3_1 = sum_3_1 - Buffer_3_1[ptr_1] + z;
	Buffer_3_1[ptr_1] = z;
	f_1 = sum_3_1 / SMOOTH_SIZE;

	/*sum_3_2 = sum_3_2 - Buffer_3_2[ptr_1] + f_1;
	Buffer_3_2[ptr_1] = f_1;
	f_1 = sum_3_2 / SMOOTH_SIZE;

	sum_3_3 = sum_3_3 - Buffer_3_3[ptr_1] + f_1;
	Buffer_3_3[ptr_1] = f_1;
	f_1 = sum_3_3 / SMOOTH_SIZE;*/

	delayBufferZ[ptrDelayZ_1] = f_1;
	ptrDelayZ_2 = ptrDelayZ_1 + 1;
	if (ptrDelayZ_2 >= DELAY)
		ptrDelayZ_2 -= DELAY;
	*out_z = delayBufferZ[ptrDelayZ_2];
	/////////////////////////////////////
	

	ptr_1++;
	if (ptr_1 == SMOOTH_SIZE)
		ptr_1 = 0;

	ptr_2++;
	if (ptr_2 == SMOOTH_SIZE)
		ptr_2 = 0;

	ptr_3++;
	if (ptr_3 == SMOOTH_SIZE)
		ptr_3 = 0;

	ptrDelayX_1++;
	if (ptrDelayX_1 == DELAY)
		ptrDelayX_1 = 0;

	ptrDelayY_1++;
	if (ptrDelayY_1 == DELAY)
		ptrDelayY_1 = 0;

	ptrDelayZ_1++;
	if (ptrDelayZ_1 == DELAY)
		ptrDelayZ_1 = 0;
}

int nextPowerof2(int in)
{
	float f;
	int i, j, k;
	f = logf((float)in)/logf(2.0);
	i = f;
	if ((float)i != f)
		i++;
	else
		return in;
	k = 1;
	for (j = 0; j<i; j++)
		k *= 2;
	return k;

}

#include "algorithms.h"

#define ANAE_BUFFER_SIZE 15
#define ANAE_REPS		10

extern int32_t LT2Threshold;

int SetAnaerobicFlag(float rfvmEffort, int init)
{
	static float effortBuffer[ANAE_BUFFER_SIZE];
	static int ptr = 0;
	static int counterLT2 = 0;
	static int ANAEflag = 0;
	static int state = 0;
	float f1, f2;
	int dummy;
	if (init == 0) // Run the algorithm
	{
		if (LT2Threshold != 0 && cadence != 0)
		{
			if (effortBuffer[ptr] > LT2Threshold)
				counterLT2--;
			effortBuffer[ptr] = rfvmEffort;
			ptr++;
			if (ptr == ANAE_BUFFER_SIZE)
				ptr = 0;
			if (rfvmEffort > LT2Threshold)
				counterLT2++;
			if (state == 0)
			{
				if (counterLT2 >= ANAE_REPS)
				{
					ANAEflag = 1;
					state = 1;
				}
			}
			else if (state == 1)
			{
				if (counterLT2 < ANAE_BUFFER_SIZE - ANAE_REPS)
				{
					ANAEflag = 0;
					state = 0;
				}
			}
			//f1 = rfvmEffort;
			//f2 = counterLT2;
			//f_write(&Debug, &f1, 4, &dummy);
			//f_write(&Debug, &f2, 4, &dummy);
			return ANAEflag;
		}
		else
			return 0;
	}
	else	// Reset buffers and variables
	{
		state = 0;
		ptr = 0;
		counterLT2 = 0;
		ANAEflag = 0;
		for (int i = 0; i < ANAE_BUFFER_SIZE; i++)
			effortBuffer[i] = 0;
		return 0;
	}
}

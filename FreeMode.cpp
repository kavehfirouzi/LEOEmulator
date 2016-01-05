#include "algorithms.h"

int32_t	 Rawfiltered1, Rawfiltered2;		

void FreeMode(int32_t* rawData, int LR)
{
	uint8_t sel;
	float32_t filtered, RMS;
	int32_t	 temp;
	/*if (LR != 0)
	{
		temp = rawData[0];
		rawData[0] = rawData[1];
		rawData[1] = temp;
	}*/
	for (sel = 0; sel<2; sel++)  																									 // for loop to run RFVM and Ham algorithms. sel = 0: RFVM, sel = 1: Ham
	{
			PreProcessing(rawData, &filtered, &RMS, sel, 1);
			temp = filtered / 10; 																											// Putting Free-mode filtered EMG values in the bluetooth buffer
			if (sel == 0)																																	// RFVM EMG
			{
				    //f_write(&Debug, &filter_out, 4, &dummy);
					Rawfiltered1 = 0;
					Rawfiltered1 |= ((temp & 0xff) << 8) | ((temp & 0xff00) >> 8);  // Changing small-endian value to big-endian for bluetooth
			}
			else																																					// Ham EMG
			{
					Rawfiltered2 = 0;
					Rawfiltered2 |= ((temp & 0xff) << 8) | ((temp & 0xff00) >> 8);	// Changing small-endian value to big-endian for bluetooth
			}
	}
}

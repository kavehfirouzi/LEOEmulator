#include "algorithms.h"
#include <fstream>
extern std::ofstream f1_1, f1_2, f1_3, f2, f3, f4_1, f4_2, f4_3, f5;

extern uint32_t global_counter;										// A global counter that counts the number of samples processed by naive bayes
extern int32_t  RFVMMVC;												// MVC for rfvm for normalizing RMS values
extern int32_t  HamMVC;
extern uint8_t  cadence_gyro;										// Cadence calculated from EMG
extern uint32_t LT2Warning;
float effortRFVM = 0;														// RFVM effort
float effortHam = 0;															// Ham effort
float effortVL = 0;															// Ham effort
float Ratio = 0;																// Ratio (rfvm as the numerator)
float effortRFVMnonNormalized = 0;											// non-normalized Effort values (for updating MVC)
float effortHamnonNormalized = 0;
float effortVLnonNormalized = 0;

void EMGAlgorithms(int32_t* rawData, BT4_Data_t *Results, int LR)
{
	static int segRFVM = 0;																// RFVM and Ham segmentations
    static int segHam = 0;
    static int segVL = 0;
	static uint16_t meanFrequencyRFVM1;													// Types I and II rfvm and ham fatigue
    static uint16_t meanFrequencyRFVM2;
  	static uint16_t meanFrequencyHam1;
	static uint16_t meanFrequencyHam2;
	float32_t filteredRMS, highPassedEMG, RMS_delayed, RMS;
	uint8_t sel;
	uint32_t temp_1, temp_2;
	uint16_t tempQ16;
	uint8_t temp8_t;
	/*if (LR != 0)																		// Left and Right leg selection. rfvm and ham values in the raw EMG buffer
	{
	  temp_1 = rawData[0];
	  rawData[0] = rawData[1];
	  rawData[1] = temp_1;
	}*/

	for (sel = 0; sel<3; sel++)  														 // for loop to run RFVM and Ham algorithms. sel = 0: RFVM, sel = 1: Ham
	{
		PreProcessing(rawData, &highPassedEMG, &RMS, sel, 0);
		if (sel == 0) 																	 // Running segmentation and segment analysis algorithms for rfvm
		{
			HammmingSmooth_RFVM(RMS, &filteredRMS, &RMS_delayed, 0);					 // smoothing RMS, segmenting and post processing on segmentation
			Unsupervised(filteredRMS/2, &segRFVM, sel, 0);
			PostProcessing(segRFVM, &segRFVM, RMS_delayed, &RMS_delayed, 0, 0, 0, 0, sel);
			SegmetnAnalysisRFVM(segRFVM, RMS_delayed, &effortRFVM, &effortRFVMnonNormalized, &Ratio, global_counter, 0);		// analyzing each rfvm segment, Finding Effort and Ratio
			pWelch_rfvm(highPassedEMG, &meanFrequencyRFVM1, &meanFrequencyRFVM2, segRFVM, 0);// calculating Fatigue
			tempQ16 = (uint16_t)(Ratio*25600);
			temp8_t = (uint8_t)(Ratio*100);
			memcpy(&Results->RatioQ16[0], &tempQ16, 2);
			//////
			f4_1 << filteredRMS << std::endl;
			f1_1 << segRFVM << std::endl;
			//////
			/*Fatigue*/
			memcpy(&Results->RFVMTypeI_Fatigue[0], &meanFrequencyRFVM1, 2);
			memcpy(&Results->RFVMTypeII_Fatigue[0], &meanFrequencyRFVM2, 2);
			//Dynamic Update not used anymore
			/*if (efforRFVMnonNormalized > RFVMMVC)										 // Updating MVC for RFVM
			{
				RFVMMVC = efforRFVMnonNormalized;
				temp_2 = RFVMMVC/2;
				temp_1 = 0;
				temp_1 |= ((temp_2 & 0xff) << 8) | ((temp_2 & 0xff00) >> 8);        	 // Sending a 2-byte unsigned integer MVC value to bluetooth buffer. Change from big-endian to small-endian.
				memcpy(&Results->MVC_IO[0], (uint8_t*)&temp_1, 2);
				temp_2 = HamMVC/2;
				temp_1 = 0;
				temp_1 |= ((temp_2 & 0xff) << 8) | ((temp_2 & 0xff00) >> 8);        	 // Sending a 2-byte unsigned integer MVC value to bluetooth buffer. Change from big-endian to small-endian.
				memcpy(&Results->MVC_IO[2], (uint8_t*)&temp_1, 2);
			}*/
			if (cadence == 0) 														 // If cadence is 0, set the intesity and targeting values to 0
			{
				effortRFVM = 0;
				effortHam = 0;
				effortVL = 0;
				effortRFVMnonNormalized = 0;
				effortHamnonNormalized = 0;
				effortVLnonNormalized = 0;
				Ratio = 0;
				Results->rfvmEffort_f32 = 0;  						//FTE mode
				Results->rfvmEffortNormalized_f32 = 0;
				tempQ16 = 0;
				memcpy(&Results->rfvmEffortQ16[0], &tempQ16, 2);	//Normal mode
				Results->hamEffort_f32 = 0;							//FTE mode
				Results->hamEffortNormalized_f32 = 0;
				tempQ16 = 0;
				memcpy(&Results->hamEffortQ16[0], &tempQ16, 2);		//Normal mode
				Results->vlEffort_f32 = 0;							//FTE mode
				Results->vlEffortNormalized_f32 = 0;
				tempQ16 = 0;
				memcpy(&Results->vlEffortQ16[0], &tempQ16, 2);  	//Normal mode
			}

		}
		if (sel == 1) 															        // Running segmentation and segment analysis algorithms for ham
		{
			HammmingSmooth_Ham(RMS, &filteredRMS, &RMS_delayed, 0);					    // smoothing RMS, segmenting and post processing on segmentation
			Unsupervised(filteredRMS/2, &segHam, sel, 0);
			PostProcessing(segHam, &segHam, 0, 0, RMS_delayed, &RMS_delayed, 0, 0, sel);
			SegmetnAnalysisHam(segHam, RMS_delayed, &effortHam, &effortHamnonNormalized, global_counter, 0); // Analyzing each ham segment
			pWelch_ham(highPassedEMG, &meanFrequencyHam1, &meanFrequencyHam2, segHam, 0);// Calculating fatigue
			f1_2 << segHam << std::endl;
			f4_2 << filteredRMS << std::endl;
			/*Fatigue*/
			memcpy(&Results->HamTypeI_Fatigue[0], &meanFrequencyHam1, 2);
			memcpy(&Results->HamTypeII_Fatigue[0], &meanFrequencyHam2, 2);
			//Dynamic Update not used anymore
			/*if (efforHamnonNormalized > HamMVC)											 // Updating MVC for HAM
			{
				HamMVC = efforHamnonNormalized;
				temp_2 = RFVMMVC/2;
				temp_1 = 0;
				temp_1 |= ((temp_2 & 0xff) << 8) | ((temp_2 & 0xff00) >> 8);          	 // Sending a 2-byte unsigned integer MVC value to bluetooth buffer. Change from big-endian to small-endian.
				memcpy(&Results->MVC_IO[0], (uint8_t*)&temp_1, 2);
				temp_2 = HamMVC/2;
				temp_1 = 0;
				temp_1 |= ((temp_2 & 0xff) << 8) | ((temp_2 & 0xff00) >> 8);          	 // Sending a 2-byte unsigned integer MVC value to bluetooth buffer. Change from big-endian to small-endian.
				memcpy(&Results->MVC_IO[2], (uint8_t*)&temp_1, 2);
			}*/
		}
		if (sel == 2) 															        // Running segmentation and segment analysis algorithms for ham
		{
			HammmingSmooth_VL(RMS, &filteredRMS, &RMS_delayed, 0);					    // smoothing RMS, segmenting and post processing on segmentation
			Unsupervised(filteredRMS/2, &segVL, sel, 0);
			PostProcessing(segVL, &segVL, 0, 0, 0, 0, RMS_delayed, &RMS_delayed, sel);
			SegmetnAnalysisVL(segVL, RMS_delayed, &effortVLnonNormalized, &effortVL, &LT2Warning, global_counter, 0); // Analyzing each vl segment
			Results->LacticAcid = LT2Warning;
			f4_3 << RMS << std::endl;
			f1_3 << segVL << std::endl;
		}
	}
}

#ifndef __ALGORITHMS__
#define __ALGORITHMS__

#include <stdint.h>
#include <stdlib.h>
#include "string.h"
#include <math.h>
#include "BT4_structure.h"

//#define debug

#define EMGACC 81
#define FATIGUE_BUFFER 209								   // 8(RMS) + 100 (hamming+smooth) + 102(postprocess)
#define RMS_WIN_SIZE		19                             // Size of RMS sliding window
#define MA_SIZE 9																	 // Size of High-Pass sliding window for raw EMG
#define MA_SIZE_LOW1 19														 // Size of first round of smoothing on RMS
#define MA_SIZE_LOW2 19														 // Size of second round of smoothing on RMS
#define MA_SIZE_LOW3 19														 // Size of smoothing for Accelerometer Walk 
#define DET_MA_SIZE 125														 // Maximum size needed for detrending window
#define LUT1 201
#define LUT2 1000

typedef   float float32_t;
//Debugging data//
extern const int32_t f_x[3750];
extern const int32_t f_y[3750];
extern const int32_t f_gz[3750];
extern const int32_t f_gy[3750];
extern const int32_t huge_rfvm[20000];
extern const int32_t huge_ham[20000];
extern const int32_t a_q[20000];
extern const int32_t a_h[20000];
extern const int32_t a_vl[20000];
extern const int32_t a_x[5000];
extern const int32_t a_y[5000];
extern const int32_t a_g_z[5000];
extern const int32_t bc36_q[20000];
extern const int32_t bc36_h[20000];
extern const int32_t bc36_x[5000];
extern const int32_t bc36_y[5000];	
extern const int32_t a_2_x[16300];
extern const int32_t a_2_y[16300];
extern const int32_t a_2_gz[16300];
extern const int32_t a_2_gy[16300];
extern const int32_t sine_test[4688];

extern float effortRFVM;														// RFVM effort
extern float effortHam;															// Ham effort
extern float effortVL;															// Ham effort
extern float Ratio;																// Ratio (rfvm as the numerator)
extern float effortRFVMnonNormalized;											// non-normalized Effort values (for updating MVC)
extern float effortHamnonNormalized;
extern float effortVLnonNormalized;

typedef union                                      // Structure to read raw EMG and accelerometer values from sensors
{
	uint8_t 	data_byte[4];
	int32_t		raw_data;
} RAW_DATA;


struct hamSegments																		// A structure used for the Coordination algorithm, explained at coordination function
{
	float Ave;
	float Raw_RMS;
	int point;
	int point_mid;
};

struct MyBuffers																		// A structure used for the Coordination algorithm, explained at coordination function
{
	float B1[100];
	float B2[50];
	float B3[100];
	float B4[50];
	unsigned int l1[500];
	unsigned int l2[200];
};

struct EKF_Buffer  //557*4 bytes
{
	float RMSBuff[61];
	float smoothBuff[100]; //= (float*)(0x10000FA0);
	float delay2[50]; // = (float*)(0x10001130);
	float MaxMin[251];
	float Delay[95];
};

struct RMS_Buffer   //44*4
{
	float dummy_1[RMS_WIN_SIZE];																				// Buffer to hold rfvm values
	float dummy_2[RMS_WIN_SIZE];																				// Buffer to hold ham values
	float RMS_1;																										// Variable to hold the sum of squared values inside the sliding window
	int pointer_1;																									// Pointer to the place inside the buffer that holds the odest value
	float RMS_2;
	int pointer_2;
	int count1;																										// A counter that keeps the output at zero before the buffer is filled
	int count2;
};

struct High_Pass_Buffer   //26*4
{
	float dummy_1[MA_SIZE];                             // The buffer to hold rfvm raw EMG rfvm values
	float dummy_2[MA_SIZE];														 // The buffer to hold rfvm raw EMG ham values
	float sum_rfvm;																 // Variables to hold the sum of the values inside the sliding window
};

struct MVC_Buffer
{
	float Buffer[850];
};

extern int FTE_MODE;
extern int32_t cadence;
int SetAnaerobicFlag(float rfvmEffort, int init);
void AllocateMemoryAlg(void);
void FreeMemoryAlg(void);
float sqrt_f32(float f);
void FreeMode(int32_t* rawData, int LR);
void Unsupervised(uint16_t RMS, int32_t *segment, int sel, int init);
void PreProcessing(int *RawEMG, float* FilteredEMG, float* SmoothedRMS, int sel, int freeMode);
void EMGAlgorithms(int32_t* rawData, BT4_Data_t *Results, int LR);
void ResetAlgorithm(void);
void EMGHighPass(int32_t Data, float32_t *MAout, uint32_t init, uint32_t sel);
void MVCRFVM(int32_t* rawData, BT4_Data_t *Results, int init, int LR);
void MVCHam(int32_t* rawData, BT4_Data_t *Results, int init, int LR);
void MVC(int32_t* rawData, BT4_Data_t *Results, int init, int LR);
void emgRMS(float32_t Data, float32_t *RMSOut, uint32_t init, uint32_t sel, int size);
void SegmetnAnalysisRFVM(float in, float RMS, float *effortRFVM, float *rawEffort, float *ratio, uint32_t globalCounter, int init);
void SegmetnAnalysisHam(float in, float RMS, float *hamEffort, float *rawEffort, uint32_t globalCounter, int init);
void SegmetnAnalysisVL(float in, float RMS, float *rawEffort, float *vlEffort, uint32_t *LT2Warning, uint32_t globalCounter, int init);
void PostProcessSeg(int in, int* out, float RMS_rfvm, float *RMSout_rfvm, float RMS_ham, float *RMSout_ham, float RMS_vl, float *RMSout_vl, int sel, int init);
void PostProcessGap(int in, int* out, float RMS_rfvm, float *RMSout_rfvm, float RMS_ham, float *RMSout_ham, float RMS_vl, float *RMSout_vl, int sel, int init);
void cadenceAccel(int32_t* rawData, BT4_Data_t *Results, int init);
void cadenceGyro(int32_t* rawData, BT4_Data_t *Results, int LR, int init);
void AccAngle(int32_t* rawData, int init);
void PostProcessing(int32_t SegIn, int32_t *SegProcessed, float32_t RMS1_in, float32_t *RMS1_out, float32_t RMS2_in, float32_t *RMS2_out, float32_t RMS3_in, float32_t *RMS3_out, int sel);
void pWelch_rfvm(float newData, uint16_t* mean_freq1, uint16_t* mean_freq2, int start, int init);
void pWelch_ham(float newData, uint16_t* mean_freq1, uint16_t* mean_freq2, int start, int init);
void AvtiviyRecognition(int32_t* rawData,  BT4_Data_t *Results, int init, int LR);
void SpinScan(uint8_t* rawData, BT4_Data_t *Results, int newACC, int init);
void SpinScanGyro(int32_t* rawData, BT4_Data_t *Results, int newGyro, int LR, int init);
void HammmingSmooth_RFVM(float RMS, float *Filtered, float *delayedRMS, int init);
void HammmingSmooth_Ham(float RMS, float *Filtered, float *delayedRMS, int init);
void HammmingSmooth_VL(float RMS, float *Filtered, float *delayedRMS, int init);
extern const float32_t exp_lookup_frac[LUT2];
extern const float32_t exp_lookup_int[LUT1];

typedef struct EMMC_METRICS_strct{
        uint8_t    ID[2];     // ID for BIO signal
        uint8_t    Length[2]; // length of payload
        uint8_t    Ind[4];                // //increment by 1 every 2ms. It will be used as the index of the following members.
        uint8_t    RepIndex[4];           // The index at which LEO reaches it's highest  point, in accelerometer samples
        // Processed Data
        uint8_t     Activity[2];           // Activity flag, zero-padded
        uint16_t    RepCounter;
        uint32_t    Timestamp;          // in milliseconds
        uint16_t    LactateWarning;     // 0x0000 - False, 0x0001-0xffff - True
        uint16_t    Cadence;            // RPM
        uint8_t     Ratio[2];           // Half-precision, fixed-point value - Q8.8 (%)
        uint8_t 	QuadEffort[4];
        uint8_t     QuadStartIndex[4];   // The index at which the rfvm contraction began in EMG samples (RFVM, Ham, VL)
        uint8_t     QuadStopIndex[4];    // The index at which the rfvm contraction finished, in EMG samples (RFVM, Ham, VL)
        uint8_t     QuadStartAngle[2];   // The index at which the rfvm contraction began in EMG samples (RFVM, Ham, VL)
        uint8_t     QuadStopAngle[2];    // The index at which the rfvm contraction finished, in EMG samples (RFVM, Ham, VL)
        uint8_t     QuadTypeIFatigue[2];   // RF, VM, VL, RFVM, Ham Type I Fatigue - Q8.8 (Hz)
        uint8_t     QuadTypeIIFatigue[2];  // RF, VM, VL, RFVM, Ham Type II Fatigue - Q8.8 (Hz)
        uint8_t     MusclesNumber[2];
		uint8_t		RFVMMuscleID[2];
		uint8_t 	RFVMEffort[4];
        uint8_t     RFVMStartIndex[4];   // The index at which the rfvm contraction began in EMG samples (RFVM, Ham, VL)
        uint8_t     RFVMStopIndex[4];    // The index at which the rfvm contraction finished, in EMG samples (RFVM, Ham, VL)
        uint8_t     RFVMStartAngle[2];   // The index at which the rfvm contraction began in EMG samples (RFVM, Ham, VL)
        uint8_t     RFVMStopAngle[2];    // The index at which the rfvm contraction finished, in EMG samples (RFVM, Ham, VL)
        uint8_t     RFVMTypeIFatigue[2];   // RF, VM, VL, RFVM, Ham Type I Fatigue - Q8.8 (Hz)
        uint8_t     RFVMTypeIIFatigue[2];  // RF, VM, VL, RFVM, Ham Type II Fatigue - Q8.8 (Hz)
        uint8_t		HamMuscleID[2];
        uint8_t 	HamEffort[4];
	    uint8_t     HamStartIndex[4];   // The index at which the rfvm contraction began in EMG samples (RFVM, Ham, VL)
	    uint8_t     HamStopIndex[4];    // The index at which the rfvm contraction finished, in EMG samples (RFVM, Ham, VL)
	    uint8_t     HamStartAngle[2];   // The index at which the rfvm contraction began in EMG samples (RFVM, Ham, VL)
	    uint8_t     HamStopAngle[2];    // The index at which the rfvm contraction finished, in EMG samples (RFVM, Ham, VL)
	    uint8_t     HamTypeIFatigue[2];   // RF, VM, VL, RFVM, Ham Type I Fatigue - Q8.8 (Hz)
	    uint8_t     HamTypeIIFatigue[2];  // RF, VM, VL, RFVM, Ham Type II Fatigue - Q8.8 (Hz)
        uint8_t		VLMuscleID[2];
        uint8_t 	VLEffort[4];
	    uint8_t     VLStartIndex[4];   // The index at which the rfvm contraction began in EMG samples (RFVM, Ham, VL)
	    uint8_t     VLStopIndex[4];    // The index at which the rfvm contraction finished, in EMG samples (RFVM, Ham, VL)
	    uint8_t     VLStartAngle[2];   // The index at which the rfvm contraction began in EMG samples (RFVM, Ham, VL)
	    uint8_t     VLStopAngle[2];    // The index at which the rfvm contraction finished, in EMG samples (RFVM, Ham, VL)
	    uint8_t     VLTypeIFatigue[2];   // RF, VM, VL, RFVM, Ham Type I Fatigue - Q8.8 (Hz)
	    uint8_t     VLTypeIIFatigue[2];  // RF, VM, VL, RFVM, Ham Type II Fatigue - Q8.8 (Hz)
}EMMC_METRICS_t;
extern EMMC_METRICS_t EMMC_METRICS;

#endif

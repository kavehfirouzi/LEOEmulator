
#include "algorithms.h"

uint8_t  cadence_acc = 0;                                			// Cadence calculated from accelerometer
uint8_t  cadence_gyro = 0;											// Cadence calculated from EMG
uint32_t global_counter = 0;										// A global counter that counts the number of samples processed by naive bayes																		 // The distance between the rfvm peak and closest Ham peak to it																 // Window size for post_processing
uint8_t  Normalizers[8] = {0,0,0,0,0,0,0,0};						// Buffer to read the two 2-byte MVC values from flash
int32_t  RFVMMVC;													// MVC for rfvm for normalizing RMS values
int32_t  HamMVC;
int32_t  VLMVC;
int32_t  saveMVC = 0;
int32_t LT2Threshold;
uint32_t LT2Warning = 0;
extern int32_t RAWACC_BigBuf[15];									// circular Buffers and pointers inside them to hold raw EMG, ACC and Gyro values
extern int32_t RAWGyro_BigBuf[15];
extern int32_t RAWEMGInt_Buf[150];
unsigned char GyroFlag_Buf[50];
extern int ptremg;
extern int ptracc;
extern int ptrgyro;
int ptr_emg = 0;													// Pointers to new data inside EMG, ACC and Gyro Buffers
int ptr_acc = 0;
int ptr_gyro = 0;
extern BT4_Data_t BT4_Data;
extern float tempFiltered_RF;
/*Used in Debugging Mode*/
extern int test_data_counter;
extern int test_data_counter_2;
extern int test_data_counter_3;
int my_counter;
int BT_repUpdate;
int32_t cadence;
/*FUNCTION*----------------------------------------------------------------
*
* Function Name  : ExecuteAlgorithms
* Returned Value : None
* Parameter	stream : response or events message from BT4 module
* Comments       :
*     A command can only be sent after the response of last command has
*     been received.
*
*END*--------------------------------------------------------------------*/
void ExecuteAlgorithms(uint32_t EMGACCGYRO)         						//EMGACCGYRO = 0 means EMG event, 1 means ACC event and 2 means Gyro
{
}

/*
 * Reset algorithm buffers and other variable to their initial values and states
 */
void ResetAlgorithm(void)
{
	int init = 1;
	RAW_DATA raw;
	int32_t dummy_int;
	//SpinScan(0, 0, 0, init);
	SpinScanGyro(0, 0, 0, 0, init);										// Resetting buffers, variables and states for each function
	AvtiviyRecognition(0, 0, init, 0);
	Unsupervised(0, 0, 0, init);
	EMGHighPass(0, 0, init, 1);
	emgRMS(0, 0, init, 1, 0);
	SegmetnAnalysisRFVM(0, 0, 0, 0, 0, 0, init);
	SegmetnAnalysisHam(0, 0, 0, 0, 0, init);
	SegmetnAnalysisVL(0, 0, 0, 0, 0, 0, init);
	cadenceGyro(0, 0, 0, init);
	pWelch_rfvm(0, 0, 0, 0, init);
	pWelch_ham(0, 0, 0, 0, init);
	AvtiviyRecognition(0,  0, 1, 0);
	SetAnaerobicFlag(0, 1);
	/* Reading MVC values from the files*/
	RFVMMVC = 9753;													// MVC for rfvm for normalizing RMS values
	HamMVC = 17369;
	VLMVC = 10744;

	global_counter = 0;
	BT4_Data.Cadence[0] = 0;
	BT4_Data.Cadence[1] = 0;
	BT4_Data.RFVMHAM_Intensity[0] = 0;
	BT4_Data.RFVMHAM_Intensity[1] = 0;
	BT4_Data.Targeting = 0;
	BT4_Data.HamTypeI_Fatigue[0] = 0;
	BT4_Data.HamTypeI_Fatigue[1] = 0;
	BT4_Data.HamTypeII_Fatigue[0] = 0;
	BT4_Data.HamTypeII_Fatigue[1] = 0;
	BT4_Data.RFVMTypeI_Fatigue[0] = 0;
	BT4_Data.RFVMTypeI_Fatigue[1] = 0;
	BT4_Data.RFVMTypeII_Fatigue[0] = 0;
	BT4_Data.RFVMTypeII_Fatigue[1] = 0;
	BT4_Data.RFVM_StartAngle = 0;
	BT4_Data.RFVM_EndAngle = 0;
	BT4_Data.Ham_StartAngle = 0;
	BT4_Data.Ham_EndAngle = 0;
	BT4_Data.Reps[0] = 0;
	BT4_Data.Reps[1] = 0;
	BT4_Data.RFVM_StartAngle16[0] = 0;
	BT4_Data.RFVM_StartAngle16[1] = 0;
	BT4_Data.RFVM_EndAngle16[0] = 0;
	BT4_Data.RFVM_EndAngle16[1] = 0;
	BT4_Data.Ham_StartAngle16[0] = 0;
	BT4_Data.Ham_StartAngle16[1] = 0;
	BT4_Data.Ham_EndAngle16[0] = 0;
	BT4_Data.Ham_EndAngle16[1] = 0;
	BT4_Data.rfvmEffortQ16[0] = 0;
	BT4_Data.rfvmEffortQ16[1] = 0;
	BT4_Data.hamEffortQ16[0] = 0;
	BT4_Data.hamEffortQ16[1] = 0;
	BT4_Data.RatioQ16[0] = 0;
	BT4_Data.RatioQ16[1] = 0;
	LT2Warning = 0;
	BT_repUpdate = 0;
}

/* Allocating and re-allocating memory for algorithms. Will be used in future revisions*/
void AllocateMemoryAlg(void)
{

//	if ((BT4_Data.Mode == 0x4 || BT4_Data.Mode == 0x2 || BT4_Data.Mode == 0x1))
//	{
//		EKF_Ptr_Q = (uint32_t*)malloc(2240);
//		EKF_Ptr_H = (uint32_t*)malloc(2240);
//		EKF_RFVM = (struct EKF_Buffer*)EKF_Ptr_Q;
//		EKF_Ham = (struct EKF_Buffer*)EKF_Ptr_H;
//		EKF_ham(0,0,0,1);
//		EKF_rfvm(0,0,0,1);
//		RMS_f32(0,0,1,0,0);
//		Moving_Average_HIGH(0,0,1,0);
//
//	}
//
//	if (BT4_Data.Mode == 0x11)
//	{
//		MVC_Q_Ptr = (uint32_t*)malloc(3400);
//		MVC_H_Ptr = (uint32_t*)malloc(3400);
//		MVC_RFVM = (struct MVC_Buffer*)MVC_Q_Ptr;
//		MVC_Ham = (struct MVC_Buffer*)MVC_H_Ptr;
//		MVC(0, 0, 1);
//	}
//	//RMS_Ptr = (uint32_t*)malloc(44*4);
//	//High_Pass_Ptr = (uint32_t*)malloc(18*4);
//
//
//	//RMS = (struct RMS_Buffer*)RMS_Ptr;
//	//High_Pass = (struct High_Pass_Buffer*)High_Pass_Ptr;
//	test_data_counter = 0;
//	my_counter = 0;
}

void FreeMemoryAlg(void)
{
//	free(EKF_Ptr_Q);
//	free(EKF_Ptr_H);
//	free(MVC_Q_Ptr);
//	free(MVC_H_Ptr);
//	//free(RMS_Ptr);
//	//free(High_Pass_Ptr);
}


#ifndef __BT4__
#define __BT4__

typedef struct BT4_DATA_strct{
	uint8_t	Cadence[2];
	uint8_t	LacticAcid;
	uint8_t	raw_singlebyte;
	uint8_t	Activity;
	uint8_t	HeartRate;
	uint8_t	RMS_RFVM[2];//[0]: H, [1]: L.
	uint8_t	RMS_Ham[2];//[0]: H, [1]: L.
	uint8_t	Accelerometer[6];//raw acceletometer. [0][1]: x-axis; [2][3]: y-axis; [4][5]: z-axis.
	uint8_t	RAW_RFVM[3];//[0]: H, [1]: M, [2]: L.
	uint8_t	RAW_Ham[3];//[0]: H, [1]: M, [2]: L.
	uint8_t	RFVMHAM_Intensity[2];//[0]: H, [1]: L.
	uint8_t	Coordination;
	uint8_t	Targeting;
	uint8_t	Battery;
	uint8_t CurrentTime[10];
	uint8_t LocalTime[2];
	uint8_t	Status20[20];
	uint8_t	MVC20[20];//[0] ~ [2]: sequence number, 1 represents 33ms; [3][4] RFVM RMS_Current_H, RFVM RMS_Current_L; [5][6] HAM RMS_Current_H, HAM RMS_Current_L;
										//[7][8] RFVM RMS_MAX_H, RFVM RMS_MAX_L; [9][10] HAM RMS_MAX_H, HAM RMS_MAX_L; Others are dummy.
	uint8_t	SessionID[2];//[0]: H, [1]: L.
	uint8_t	Status[2];//bit0 of [1] SPI flash full flag; bit1 of [1] low battery flag; bit2 of [105] lead off;
										//bit3 of [1] LEO activity flag; bit4 of [1] no SPI flash writing because of super low battery
	uint8_t RFVMTypeI_Fatigue[2];
	uint8_t RFVMTypeII_Fatigue[2];
	uint8_t HamTypeI_Fatigue[2];
	uint8_t HamTypeII_Fatigue[2];
	uint8_t VLTypeI_Fatigue[2];
	uint8_t VLTypeII_Fatigue[2];
	uint8_t	MVC_IO[4];//[0]: RFVM_H, [1]: RFVM_L, [2]: Ham_H, [3]: Ham_L
	uint8_t	EMG20[20];//[0]~[2]: sequence number, 1 represents 33ms; [3]~[10]: RFVM_EMG_t0,1,2,3;
										//[11]~[18]: HAM_EMG_t0,1,2,3;
	uint8_t	DATA20A[20];//[0][1]: sequence number, 1 represents 33ms; [2]~[10]12-bitAccelerometer x, y, z at t0, t1;
										//[11]~[19]12-bitAccelerometer x, y, z at t2, t3;
	uint8_t	Mode;
	uint32_t    Timestamp;          // in milliseconds
	uint8_t RFVM_StartAngle;
	uint8_t RFVM_StartAngle16[2];
	uint8_t RFVM_EndAngle;
	uint8_t RFVM_EndAngle16[2];
	uint8_t Ham_StartAngle;
	uint8_t Ham_StartAngle16[2];
	uint8_t Ham_EndAngle;
	uint8_t Ham_EndAngle16[2];
	uint8_t VL_StartAngle;
	uint8_t VL_StartAngle16[2];
	uint8_t VL_EndAngle;
	uint8_t VL_EndAngle16[2];
	uint8_t Reps[2];
	uint8_t rfvmEffortQ16[2];
	uint8_t hamEffortQ16[2];
	uint8_t vlEffortQ16[2];
	float rfvmEffort_f32;
	float hamEffort_f32;
	float vlEffort_f32;
	float rfvmEffortNormalized_f32;
	float hamEffortNormalized_f32;
	float vlEffortNormalized_f32;
	uint8_t RatioQ16[2];
	uint8_t Firmware[2];
}BT4_Data_t;

extern BT4_Data_t BT4_Data;
#endif

#include <iostream>
#include <fstream>
#include <string>
#include <fstream>
#include "algorithms.h"
using namespace std;
extern void ColumnConcat(float *matrix_1, float *matrix_2, float *matrix_out, int row, int col_1, int col_2);
extern void RowConcat(float *matrix_1, float *matrix_2, float *matrix_out, int col, int row_1, int row_2);
extern int read_file(float* data, string filename);
ofstream f1_1, f1_2, f1_3, f2, f3, f4_1, f4_2, f4_3, f5, f6, f6_s, f6_e, f7, f7_e, f7_s, f9_2, f6_2, ftest, X, Y, f1, f4, f8, f9, f10, f11, f12;
void test_function(float alaki[4][3]);
float exponent(float f);
extern int32_t Normalize_rfvm;
EMMC_METRICS_t EMMC_METRICS;
float max_test(float in);
extern uint8_t  cadence_acc;
BT4_Data_t BT4_Data;
int FTE_MODE = 0;
void main()
{
   
	int size;
	float *data1 = new float[14000000];
	float *data2 = new float[14000000];
	float *data3 = new float[7000000];
	float *data4 = new float[7000000];
	float *data5 = new float[7000000];
	float *data6 = new float[14000000];
	float *data7 = new float[7000000];
	float *data8 = new float[7000000];

	string filename= "bradfatigue3_q.txt";
	size = read_file(data1, filename);

	filename= "bradfatigue3_h.txt";
	read_file(data2, filename);

	filename= "bradfatigue3_x.txt";
	read_file(data3, filename);

	filename= "bradfatigue3_y.txt";
	read_file(data4, filename);

	filename= "bradfatigue3_gz.txt";
	read_file(data5, filename);

	filename= "bradfatigue3_gy.txt";
	read_file(data8, filename);

	filename= "bradfatigue3_gx.txt";
	read_file(data7, filename);

	filename= "bradfatigue3_vl.txt";
	read_file(data6, filename);

	f1_1.open("segq.txt");
	f1_2.open("segh.txt");
	f1_3.open("segvl.txt");
	f2.open("butter_2.txt");
	f1.open("butter_1.txt");
	f3.open("MVC.txt");
	f4_1.open("RMSq.txt");
	f4_2.open("RMSh.txt");
	f4_3.open("RMSvl.txt");
	f5.open("Ratio.txt");
	f6.open("Spin_rfvm.txt");
	f6_2.open("Spin_ham.txt");
	f6_s.open("spin_rfvm_s.txt");
	f6_e.open("spin_rfvm_e.txt");
	f7_s.open("spin_ham_s.txt");
	f7_e.open("spin_ham_e.txt");
	f7.open("fatigue.txt");
	ftest.open("test.txt");
	X.open("pos_x.txt");
	Y.open("pos_y.txt");
	f4.open("cadence.txt");
	f8.open("RFVMEffort.txt");
	f9.open("HamEffort.txt");
	f10.open("VLEffort.txt");
	f11.open("Spin_vl.txt");
	f12.open("spin_quad.txt");
	float out;
	float Sigma, RMS_out;
	int seg_rfvm;
	unsigned char dummy;
	int alaki = 0;
	int k,z;
	BT4_Data_t Results;
	int rawData[6];
	int  GyroData[3];
	int R = 0;
	ResetAlgorithm();
	for (int j = 0; j <1; j++)
	{   
		int ii = 0;
		for (int i = 0; i < size; i++)
		{
			
			k = data1[i];
			rawData[0] = k;
			k = data2[i];
			rawData[1]  = k;
			k = data6[i];
			rawData[2] = k;
			//MVC(rawData, Results, 0);
			EMGAlgorithms(rawData, &Results, R);

			if ( (i % 4)  == 0)
			{	
				k = data7[ii+0];
				GyroData[0] =  k;
				k = data8[ii+0];
				GyroData[1] =  k;
				if (R == 0)
					k = data5[ii+0];
				else
					k = -data5[ii+0];
				GyroData[2] =  k;
				cadenceGyro(GyroData, &Results, R, 0);
				SpinScanGyro(GyroData, &Results, 1, R, 0);
				ii++;
			}
			else
			{
				SpinScanGyro(GyroData, &Results, 0, R, 0);
			}
				//SpinScan(rawData, 0);
			
		}
	}


	f1_1.close();
	f1_2.close();
	f1_3.close();
	f2.close();
	f3.close();
	f4_1.close();
	f4_2.close();
	f4_3.close();
	f5.close();
	f6.close();
	f6_s.close();
	f6_e.close();
	f6_2.close();
	f7_e.close();
	f7_s.close();
	f7.close();
	ftest.close();
	f4.close();
	f8.close();
	f9.close();
	f10.close();
	f11.close();
	f12.close();
}


#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <conio.h>
#include <time.h>
#include <iostream>
#include "KFUpdate.h"


using namespace Geom2D;

void KFUpdate::KFInitialize(Geom2D::Pedestrian & new_pedestrian){
	int n = 0;
	double Tmean[] = {0,0,0,0};
	double Tsigma[] = {1, 0, 0, 0,
					  0, 1, 0, 0,	
					  0, 0, 1, 0,
					  0, 0, 0, 1};
	new_pedestrian.Tmean[0] = new_pedestrian.x;
	new_pedestrian.Tmean[1] = new_pedestrian.y;
	new_pedestrian.Tmean[2] = 0;
	new_pedestrian.Tmean[3] = 0;
	//for (n = 0; n < 4; n++)
	//	new_pedestrian.Tmean[n] = Tmean[n];			
	for (n = 0; n < 16; n++)
		new_pedestrian.Tsigma[n] = Tsigma[n];

}


void KFUpdate::measurementUpdate(Geom2D::Pedestrian & current_pedestrian, double z_measurement[]){
	int i = 0;
	int j = 0;
	int iterate = 0;
	float a[] = {1, 0, 1, 0,
				 0, 1, 0, 1,	
				 0, 0, 1, 0,
				 0, 0, 0, 1};
	float c[] = {1, 0, 0, 0,
				 0, 1, 0, 0};

	float transpose_c[] = {0, 0,
						   0, 0,	
						   0, 0,
						   0, 0};
	float q[] = {0.01, 0,
				 0, 0.01};
	float z[2];
	float eye[] = {1, 0, 0, 0,
				   0, 1, 0, 0,	
				   0, 0, 1, 0,
				   0, 0, 0, 1};
	//K Gain Initialization
	float k[] = {0, 0,
				 0, 0,	
				 0, 0,
				 0, 0};
	//Four_by_Two = Tsigma*C'
	float temp1[] = {0, 0,
					 0, 0,	
					 0, 0,
					 0, 0};
	//Two_by_Four = C*Tsigma*C'
	float temp2[] = {0, 0,0,0,
					 0, 0,0,0};

	//Two_by_Four1 = Two_by_Four + Q
	float temp3[] = {0, 0,
					 0, 0};
	float temp6[] = {0,
					 0};
	
	float temp8[] = {0,
					 0,	
					 0,
					 0};
	float temp9[] = {0, 0, 0, 0,
				     0, 0, 0, 0,	
				     0, 0, 0, 0,
				     0, 0, 0, 0};
	float Tmean[4];
	float Tsigma[16];
	for (i = 0; i < 4; i++)
		Tmean[i] = current_pedestrian.Tmean[i];			
	for (i = 0; i < 16; i++)
		Tsigma[i] = current_pedestrian.Tsigma[i];	
	for (i = 0; i < 2; i++)
		z[i] = z_measurement[i];
	CvMat TmeanM=cvMat(4,1,CV_32FC1,Tmean);
	CvMat TsigmaM=cvMat(4,4,CV_32FC1,Tsigma);
	CvMat Z=cvMat(2,1,CV_32FC1,z);
	CvMat A=cvMat(4,4,CV_32FC1,a);
	CvMat C=cvMat(2,4,CV_32FC1,c);
	CvMat TRANSPOSE_C=cvMat(4,2,CV_32FC1,transpose_c);
	CvMat Q=cvMat(2,2,CV_32FC1,c);
	CvMat EYE=cvMat(4,4,CV_32FC1,eye);
	CvMat K=cvMat(4,2,CV_32FC1,k);
	CvMat Four_by_Two=cvMat(4,2,CV_32FC1,temp1);
	CvMat Two_by_Four=cvMat(2,4,CV_32FC1,temp2);
	CvMat Two_by_Two1=cvMat(2,2,CV_32FC1,temp3);
	CvMat Two_by_Two2=cvMat(2,2,CV_32FC1,temp3);
	CvMat Two_by_Two3=cvMat(2,2,CV_32FC1,temp3);
	CvMat Two_by_One1=cvMat(2,1,CV_32FC1,temp6);
	CvMat Two_by_One2=cvMat(2,1,CV_32FC1,temp6);
	CvMat Four_by_One=cvMat(4,1,CV_32FC1,temp8);
	CvMat Four_by_Four1=cvMat(4,4,CV_32FC1,temp9);
	CvMat Four_by_Four2=cvMat(4,4,CV_32FC1,temp9);

	// Measurement Update
	//K Gain Calculation
	//**************************
	cvTranspose(&C, &TRANSPOSE_C);
	cvMatMul(&TsigmaM, &TRANSPOSE_C, &Four_by_Two);
	cvMatMul(&C,&TsigmaM,&Two_by_Four);
	cvMatMul(&Two_by_Four,&TRANSPOSE_C,&Two_by_Two1);
	cvAdd(&Two_by_Two1, &Q, &Two_by_Two2);
	cvInvert(&Two_by_Two2, &Two_by_Two3); 
	cvMatMul(&Four_by_Two,&Two_by_Two3,&K);
	//getch();///////////////////////////////////
	//**************************
	//Mean Calculation
	//**************************
	cvMatMul(&C,&TmeanM,&Two_by_One1);
	cvSub(&Z,&Two_by_One1,&Two_by_One2);
	cvMatMul(&K,&Two_by_One2,&Four_by_One);
	cvAdd(&TmeanM,&Four_by_One,&TmeanM);
	//getch();///////////////////////////////////
	//**************************
	//Covariance Calculation
	//**************************
	cvMatMul(&K,&C,&Four_by_Four1);
	cvSub(&EYE,&Four_by_Four1,&Four_by_Four2);
	cvMatMul(&Four_by_Four2,&TsigmaM,&TsigmaM);
	//getch();///////////////////////////////////
	//**************************
	for (i = 0; i < 4; i++)
		current_pedestrian.Tmean[i] = cvmGet(&TmeanM,i,0);	

	for (i = 0; i < 4; i++)
		for (j = 0; j< 4; j++){
			current_pedestrian.Tsigma[iterate] = cvmGet(&TsigmaM,i,j);	
			iterate += 1;
		}
}

void KFUpdate::timeUpdate(Geom2D::Pedestrian & current_pedestrian, DWORD scan_time){
	int i = 0;
	int j = 0;
	int iterate = 0;
	double dt = (double)scan_time/1000;
	float a[] = {1, 0, 1, 0,
				 0, 1, 0, 1,	
				 0, 0, 1, 0,
				 0, 0, 0, 1};
	float transpose_a[] = {0, 0, 0, 0,
						   0, 0, 0, 0,	
						   0, 0, 0, 0,
						   0, 0, 0, 0};
	float r[] = {1, 0, 0, 0,
				 0, 1, 0, 0,	
				 0, 0, 1, 0,
				 0, 0, 0, 1};

	
	float temp9[] = {0, 0, 0, 0,
				     0, 0, 0, 0,	
				     0, 0, 0, 0,
				     0, 0, 0, 0};
	float Tmean[4];
	float Tsigma[16];
	for (i = 0; i < 4; i++)
		Tmean[i] = current_pedestrian.Tmean[i];			
	for (i = 0; i < 16; i++)
		Tsigma[i] = current_pedestrian.Tsigma[i];

	CvMat TmeanM=cvMat(4,1,CV_32FC1,Tmean);
	CvMat TsigmaM=cvMat(4,4,CV_32FC1,Tsigma);
	CvMat A=cvMat(4,4,CV_32FC1,a);

	//cvmSet(&A,0,2,dt);
	//cvmSet(&A,1,3,dt);
	
	CvMat TRANSPOSE_A=cvMat(4,4,CV_32FC1,transpose_a);
	CvMat R=cvMat(4,4,CV_32FC1,r);
	CvMat Four_by_Four1=cvMat(4,4,CV_32FC1,temp9);
	CvMat Four_by_Four2=cvMat(4,4,CV_32FC1,temp9);

	// Time Update
	//Mean Calculation
	//**************************
	cvMatMul(&A,&TmeanM,&TmeanM);
	//**************************
	//Covariance Calculation
	//**************************
	cvMatMul(&A,&TsigmaM,&Four_by_Four1);
	cvTranspose(&A, &TRANSPOSE_A);
	cvMatMul(&Four_by_Four1,&TRANSPOSE_A,&Four_by_Four2);
	cvAdd(&Four_by_Four2,&R,&TsigmaM);

	for (i = 0; i < 4; i++)
		current_pedestrian.Tmean[i] = cvmGet(&TmeanM,i,0);	

	for (i = 0; i < 4; i++)
		for (j = 0; j< 4; j++){
			current_pedestrian.Tsigma[iterate] = cvmGet(&TsigmaM,i,j);	
			iterate += 1;
		}
}
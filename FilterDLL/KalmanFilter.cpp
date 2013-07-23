#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <conio.h>
#include <time.h>
#include <iostream>
#include "measurementUpdate.h"
#include "timeUpdate.h"

using namespace std;

int main()
{
	double x[] = {0.19, 0.81, 1.38, 1.90, 2.31, 2.58, 2.78, 2.91, 2.91, 2.94, 2.74, 2.45, 1.88, 1.60, 1.15, 1.03, 1.01, 1.08, 1.16, 1.52, 2.05, 2.38, 2.67, 2.67, 2.69, 2.66, 2.40, 1.84, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22};
	double y[] = {-1.11, -1.20, -1.02, -1.04, -1.01, -0.81, -0.57, -0.23, -0.23, 0.65, 1.16, 1.42, 1.39, 1.23, 0.88, 0.40, -0.03, -0.58, -0.90, -1.19, -1.36, -1.19, -0.93, -0.51, -0.10, 0.41, 0.77, 1.06, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10};
	float z[] = {0,
				 0,};
	float Tmean[] = {0, 0,
					 0, 0};

	float Tsigma[] = {1, 0, 0, 0,
					  0, 1, 0, 0,	
					  0, 0, 1, 0,
					  0, 0, 0, 1};
	int i = 0;
	double predicted_x = 0;
	double predicted_y = 0;

	CvMat TmeanM=cvMat(4,1,CV_32FC1,Tmean);
	CvMat TsigmaM=cvMat(4,4,CV_32FC1,Tsigma);
	CvMat Z=cvMat(2,1,CV_32FC1,z);

	for (i=0;i<40;i++){
		cvmSet(&Z,0,0,x[i]);
		cvmSet(&Z,1,0,y[i]);
		measurementUpdate(TmeanM,TsigmaM,Z);
		timeUpdate(TmeanM, TsigmaM);
		predicted_x = cvmGet(&TmeanM,0,0);
		predicted_y = cvmGet(&TmeanM,1,0);
		if (i != 39)
			cout << "Actual x: " << x[i+1] << " Actual y: " << y[i+1] << " KF x: " << predicted_x << " KF y: " << predicted_y << endl;
		
	}
	getch();
}


#if 0
int main()
{
	double x[] = {0.19, 0.81, 1.38, 1.90, 2.31, 2.58, 2.78, 2.91, 2.91, 2.94, 2.74, 2.45, 1.88, 1.60, 1.15, 1.03, 1.01, 1.08, 1.16, 1.52, 2.05, 2.38, 2.67, 2.67, 2.69, 2.66, 2.40, 1.84, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22};
	double y[] = {-1.11, -1.20, -1.02, -1.04, -1.01, -0.81, -0.57, -0.23, -0.23, 0.65, 1.16, 1.42, 1.39, 1.23, 0.88, 0.40, -0.03, -0.58, -0.90, -1.19, -1.36, -1.19, -0.93, -0.51, -0.10, 0.41, 0.77, 1.06, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10, 1.10};
	float a[] = {1, 0, 1, 0,
				 0, 1, 0, 1,	
				 0, 0, 1, 0,
				 0, 0, 0, 1};
	float transpose_a[] = {0, 0, 0, 0,
						   0, 0, 0, 0,	
						   0, 0, 0, 0,
						   0, 0, 0, 0};
	float c[] = {1, 0, 0, 0,
				 0, 1, 0, 0};

	float transpose_c[] = {0, 0,
						   0, 0,	
						   0, 0,
						   0, 0};

	float r[] = {1, 0, 0, 0,
				 0, 1, 0, 0,	
				 0, 0, 1, 0,
				 0, 0, 0, 1};
	float q[] = {0.01, 0,
				 0, 0.01};
	float z[] = {0,
				 0,};
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
	float temp4[] = {0, 0,
					 0, 0};
	float temp5[] = {0, 0,
					 0, 0};
	float temp6[] = {0,
					 0};
	float temp7[] = {0,
					 0};
	float temp8[] = {0,
					 0,	
					 0,
					 0};
	float temp9[] = {0, 0, 0, 0,
				     0, 0, 0, 0,	
				     0, 0, 0, 0,
				     0, 0, 0, 0};

	float Tmean[] = {0, 0,
					 0, 0};

	float Tsigma[] = {1, 0, 0, 0,
					  0, 1, 0, 0,	
					  0, 0, 1, 0,
					  0, 0, 0, 1};
	int i = 0;
	double predicted_x = 0;
	double predicted_y = 0;

	CvMat TmeanM=cvMat(4,1,CV_32FC1,Tmean);
	CvMat TsigmaM=cvMat(4,4,CV_32FC1,Tsigma);
	CvMat Z=cvMat(2,1,CV_32FC1,z);

	CvMat A=cvMat(4,4,CV_32FC1,a);
	CvMat TRANSPOSE_A=cvMat(4,4,CV_32FC1,transpose_a);
	CvMat C=cvMat(2,4,CV_32FC1,c);
	CvMat TRANSPOSE_C=cvMat(4,2,CV_32FC1,transpose_c);
	CvMat R=cvMat(4,4,CV_32FC1,r);
	CvMat Q=cvMat(2,2,CV_32FC1,c);
	CvMat EYE=cvMat(4,4,CV_32FC1,eye);
	CvMat K=cvMat(4,2,CV_32FC1,k);
	CvMat Four_by_Two=cvMat(4,2,CV_32FC1,temp1);
	CvMat Two_by_Four=cvMat(2,4,CV_32FC1,temp2);
	CvMat Two_by_Two1=cvMat(2,2,CV_32FC1,temp3);
	CvMat Two_by_Two2=cvMat(2,2,CV_32FC1,temp4);
	CvMat Two_by_Two3=cvMat(2,2,CV_32FC1,temp5);
	CvMat Two_by_One1=cvMat(2,1,CV_32FC1,temp6);
	CvMat Two_by_One2=cvMat(2,1,CV_32FC1,temp7);
	CvMat Four_by_One=cvMat(4,1,CV_32FC1,temp8);
	CvMat Four_by_Four1=cvMat(4,4,CV_32FC1,temp9);
	CvMat Four_by_Four2=cvMat(4,4,CV_32FC1,temp9);
	
	
	for (i=0;i<40;i++){
		cvmSet(&Z,0,0,x[i]);
		cvmSet(&Z,1,0,y[i]);
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

	predicted_x = cvmGet(&TmeanM,0,0);
	predicted_y = cvmGet(&TmeanM,1,0);
	if (i != 39)
	//cout << "Actual x: " << x[i+1] << " Actual y: " << y[i+1] << " KF x: " << predicted_x << " KF y: " << predicted_y << endl;
	cout << &TmeanM << endl;
	//getch();///////////////////////////////////
	}
	
	//cout << "hello" <<endl;
	getch();
}
#endif
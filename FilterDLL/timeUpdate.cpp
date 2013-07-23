#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <conio.h>
#include <time.h>
#include <iostream>
#include "timeUpdate.h"

using namespace std;

void timeUpdate(CvMat TmeanM, CvMat TsigmaM)
{
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

	CvMat A=cvMat(4,4,CV_32FC1,a);
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

}
#pragma once

#include<math.h>
#include<vector>
#include "ObsArray.h"
using namespace std;

#define VITVsize 360

class _declspec(dllexport) VisableArea
{
public:
	VisableArea(void);
	~VisableArea(void);

	void SetSingleVA(ObsArray& obsarr,double px,double py);// the target is in (px,py) 
	void SetItvValue(int i1,int i2,double value);
	bool GetVisableState(double rx,double ry,double rth,double& range, double& angle);// return true: visable, 

    int AngtoVFH(double ang);//+pi ~ -pi --> 0~VITVsize-1
	double VFHtoDeg(int index);
	void setITV(int idx,double dist);// manage the rounding
	void getITV(int idx,double& r, double& a);
	void GetCenter(double& x, double& y){x = cx;y=cy;}

	double Predict_Visbility(double rx,double ry,double dis);

	void ResetAll(){
		cx = 0;
		cy = 0;
		for(int i=0;i<VITVsize;++i)
			ITVs[i] = 20.0;		
	}

private:
	double ITVs[VITVsize];
	double cx;
	double cy;//Area center
};

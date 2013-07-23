#pragma once

#include <math.h>
#include <vector>
#include "OBSArray.h"
#include "Reg.h"

namespace RobotTra{

#define PI 3.14159265

class _declspec(dllexport) VelItv
{
public:
	VelItv(double ang_resolution,double dist_max);//(degree, meter)
	~VelItv(void);

public:
	inline void SetITV_Single(OBS& obs);
	void SetITV_Multi(vector<OBS>& obsarr);
	
	inline int VeltoIdx(double v,double w);// Limit: V>=0
	inline int AngtoIdx(double ang); 
	double IdxtoRad(int i);
	int getsize(){return ITV_size;}

	double getdist(int i){return dist[i];}



private:
	//data members
	std::vector<double> dist;
	double angrsl; //angle resolution
	double dmax;//Max dist value
	int ITV_size;

};

}
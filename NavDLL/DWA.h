#pragma once

#include "VelItv.h"
#include <vector>

namespace RobotTra{

struct _declspec(dllexport) VEL { //velocity
double v;
double w;
double unsafety;//0~1
int i; //Corresponding ITV index, easy for indexing
bool safe;//If it can stop before collision
bool reach;//If it can be reached immediately

int hi;//position in the VEL array
int wi;
};


class _declspec(dllexport) DWA
{
public:
	DWA(double* limits,RobotTra::VelItv& velitv);// double[8]: (Vmax,Vmin,Wmax,Wmin,V_acc,W_acc,V_rsl,Wrsl,dt)
	~DWA(void);

	void Update_Vel_State(vector<OBS>& obsarr,double pv,double pw);//update necessary states according to vitv

	double UnSafety(VEL& vel,double dv,double dw);//The probability of collision, dv,dw depicts standard derivation 
	bool Reachable(VEL& pvel,VEL& nvel);//If it is reachable from pvel->nvel
	bool Brakeable(VEL& vel);//can stop before collision

	std::vector<VEL> GetCandidates(){return candidates;}

//private:
	int width;
	int height;
	VEL** vels;//velocity array, 2D array: [height][2*width+1]
	RobotTra::VelItv* vitv;//ITV object


	double Vmax;
	double Vmin;
	double Wmax;
	double Wmin;
	double Vrsl;
	double Wrsl;
	double Vacc;
	double Wacc;
	double dt;// sample time interval

	std::vector<VEL> candidates;

};

}
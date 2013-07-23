#include "StdAfx.h"
#include "PubRobotData.h"
#include "IO_Functions.h"

using namespace Geom2D;
using namespace std;

PubRobotData::PubRobotData(void)
{
	//Create Mutex
	mutex = CreateMutex(NULL,false,(LPCWSTR)"PubDataSync");  
}

PubRobotData::~PubRobotData(void)
{
}

void PubRobotData::SetSensorData(std::vector<double> &nrange, std::vector<double> &nrad, Geom2D::Pose rawpose){

    WaitForSingleObject(mutex,INFINITE);

	nowrawpose = rawpose;

	range.resize(nrange.size());
	rad.resize(nrad.size());
	copy(nrange.begin(),nrange.end(),range.begin());
	copy(nrad.begin(),nrad.end(),rad.begin());

	//release mutex
    ReleaseMutex(mutex);

}

void PubRobotData::GetSensorData(std::vector<double> &nrange, std::vector<double> &nrad, Geom2D::Pose& rawpose){

    WaitForSingleObject(mutex,INFINITE);

	rawpose = nowrawpose;

	nrange.resize(range.size());
	nrad.resize(rad.size());
	copy(range.begin(),range.end(),nrange.begin());
	copy(rad.begin(),rad.end(),nrad.begin());

	//release mutex
    ReleaseMutex(mutex);

}


bool PubRobotData::GetLaserDataFromText(FILE* lin){
	
	if(!ReadRawLaserData(lin, nowrawpose.p.x, nowrawpose.p.y, nowrawpose.phi,rad, range))
		return false;
	else
		return true;

}

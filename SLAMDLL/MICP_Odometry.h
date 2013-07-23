#pragma once
#include "Filter_ICP.h"
#include "math.h"

using namespace std;
using namespace Geom2D;

class _declspec(dllexport)MICP_Odometry
{
public:
	MICP_Odometry(void);
	~MICP_Odometry(void);

	void ICP_Init(const vector <Point> &InitPolarData, const Pose &IniRawPose);

	void DoICPOdemetry(const vector <Point> &NewPolarData, const Pose &NewRawPose);
	Pose ComputeICPOdemetry(const vector <Point> &NewPolarData, const Pose &NewRawPose);//Compute ICP without updating
	
	Pose& GetIcpPose(){return IcpPose;}

    void GetTransReading(std::vector <Point>& res);


private:
	void UpdateNewData(const vector <Point> &NewPolarData, const Pose &NewRawPose);
	void GenerateModelData(); //write the LastDatas into Model

private:

	Filter_ICP icp;

	bool If_Init;

	int datacounter;
    const static int ldsize = 5;

	std::vector <Point> LastPolarDatas[ldsize];//In robot coordinate, (x,y)
	std::vector <Point> LastData;
	std::vector <Point> NowData;//In robot coordinate, (x,y)
    std::vector <double> dmin_R;
	std::vector <double> dmin_Z;

	Pose LastRawPose;
	Pose LastIcpPoses[ldsize];//Init pose determined by first reading
	Pose IcpPose;//Integrated pose corrected by ICP, related with LastReading

};

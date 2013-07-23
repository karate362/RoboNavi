#pragma once

#include "geometry2D.hpp"
#include <vector>

class _declspec(dllexport) PubRobotData //Necessary public data, including raw pose, icp pose, mcl pose and laser data. It should be shared by all threads 
{
public:
	PubRobotData(void);
	~PubRobotData(void);

	void SetSensorData(std::vector<double>& nrange,std::vector<double>& nrad,Geom2D::Pose rawpose);
	void GetSensorData(std::vector<double>& nrange,std::vector<double>& nrad,Geom2D::Pose &rawpose);

	bool GetLaserDataFromText(FILE* lin);

	void SetTime(DWORD nowtime){gtime = nowtime;}

	void SetPoseData(Geom2D::Pose rawpose,double nv,double nw){
		nowrawpose = rawpose;
		v = nv;
		w = nw;
	}

	void GetPoseData(Geom2D::Pose& rawpose,double& nv,double& nw){
		rawpose = nowrawpose;
		nv = v;
		nw = w;
	}

	//Geom2D::Pose Get_corrected_pose(Geom2D::Pose nowrawpose);//return the pose corrected by slampose result


//attributes

	std::vector<double> range;
	std::vector<double> rad;//last sensor reading
	Geom2D::Pose nowrawpose;//raw robot pose when the last reading was taking

	Geom2D::Pose lastrawpose;//raw robot pose when the last slampose was updating
	Geom2D::Pose slampose;//robot pose from last SLAM result

	double v;
	double w;
	DWORD gtime;

private:
	HANDLE mutex;

};

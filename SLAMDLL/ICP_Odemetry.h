#pragma once

#include "Filter_ICP.h"
#include "math.h"

using namespace std;

namespace Geom2D{

class _declspec(dllexport) ICP_Odemetry
{
public:
	ICP_Odemetry(void);
	~ICP_Odemetry(void);

	void ICP_Init(const vector <Point> &InitPolarData, const Pose &IniRawPose){
		
		Point_vector_copy(InitPolarData,LastData);
		InitPose = IniRawPose;		
		RawPose = IniRawPose;
		IcpPose = IniRawPose;
		DataPolarToXY(LastData);
		If_Init = true;
	}

	void DoICPOdemetry(const vector <Point> &NewPolarData, const Pose &NewRawPose);
	Pose ComputeICPOdemetry(const vector <Point> &NewPolarData, const Pose &NewRawPose);//Compute ICP without updating
	
	Pose& GetIcpPose(){return IcpPose;}
	
	void GetTransReading(std::vector <Point>& res){//get Data in global coordunate

		Point_vector_copy(LastData,res);

		VectorTrans(res,IcpPose);
	}



private:
	Pose ReletivePose(Pose p1,Pose p2){// reletive movement in p1 coordinate
		double dx = p2.p.x-p1.p.x;
		double dy = p2.p.y-p1.p.y;
		double c = cos(-1.0*p1.phi);
		double s = sin(-1.0*p1.phi);
		Pose rpose;

		rpose.p.x = c*dx - s*dy;
		rpose.p.y = s*dx + c*dy;
		rpose.phi = p2.phi-p1.phi;

		return rpose;
	}


	void PointTrans(Point& p, double dx,double dy,double dphi){
		double x = dx + cos(dphi)*p.x - sin(dphi)*p.y;
		double y = dy + sin(dphi)*p.x + cos(dphi)*p.y;
		p.x = x;
		p.y = y;
	}

	void PointTrans(Point& p, Pose t){
		PointTrans(p, t.p.x, t.p.y, t.phi);
	}

	void VectorTrans(vector <Point>& data, double dx,double dy,double dphi){
		std::vector<Point>::iterator it; 
		for(it=data.begin(); it!=data.end(); ++it)
			PointTrans(*it,dx,dy,dphi);
	}

	void VectorTrans(vector <Point>& data, Pose t){
		VectorTrans(data, t.p.x, t.p.y, t.phi);
	}

public:
		Pose del_pose;

private:

	Filter_ICP icp;

	bool If_Init;

	std::vector <Point> LastData;//In robot coordinate, (x,y)
	std::vector <Point> NowData;//In robot coordinate, (x,y)
    std::vector <double> dmin_R;
	std::vector <double> dmin_Z;

	Pose InitPose;//Init pose determined by first reading
	Pose RawPose;//raw pose parameter, related with IcpPose, used to compute initguess
	Pose IcpPose;//Integrated pose corrected by ICP, related with LastReading



};
}
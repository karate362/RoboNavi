#include "StdAfx.h"
#include "ICP_Odemetry.h"
#include "geometry2D.hpp"

namespace Geom2D{

ICP_Odemetry::ICP_Odemetry(void)
{
	If_Init = false;
}

ICP_Odemetry::~ICP_Odemetry(void)
{
}


Pose ICP_Odemetry::ComputeICPOdemetry(const vector <Point> &NewPolarData, const Pose &NewRawPose){


	if(!If_Init){
		If_Init = true;
		this->ICP_Init(NewPolarData, NewRawPose);
	}
    
 
	Pose newpose = IcpPose;
    Pose initGuess = NewRawPose;
	Pose dpose = initGuess;
	Point_vector_copy(NewPolarData,NowData);

	Transform2D Tr(this->RawPose);//Last raw pose
	Tr.transform_to_relative(initGuess);


	Pose pose_err = robot_encoder_err(initGuess);//robot pose error
    robot_pose_err_radius(pose_err, NowData,  dmin_R);//additional search range caused by robot pose error
    Laser_measure_err_radius(NowData,  dmin_Z);//additional search range caused by measurement error
    

	DataPolarToXY(NowData);//(r,th)-->(x,y)
	//ICP
	dpose = icp.DoICP(LastData,NowData,initGuess,30,dmin_R,dmin_Z);
	del_pose = dpose;

	//Compute global pose
	PointTrans(dpose.p,0,0,IcpPose.phi);

	newpose.p.x += dpose.p.x;
	newpose.p.y += dpose.p.y;
	newpose.phi += dpose.phi;


	return newpose;
}


void ICP_Odemetry::DoICPOdemetry(const vector <Point> &NewPolarData, const Pose &NewRawPose){


	if(!If_Init){
		If_Init = true;
		this->ICP_Init(NewPolarData, NewRawPose);
	}
    
 
    Pose initGuess = NewRawPose;
	Pose dpose = initGuess;
	Point_vector_copy(NewPolarData,NowData);

	Transform2D Tr(this->RawPose);//Last raw pose
	Tr.transform_to_relative(initGuess);


	Pose pose_err = robot_encoder_err(initGuess);//robot pose error
    robot_pose_err_radius(pose_err, NowData,  dmin_R);//additional search range caused by robot pose error
    Laser_measure_err_radius(NowData,  dmin_Z);//additional search range caused by measurement error
    

	DataPolarToXY(NowData);//(r,th)-->(x,y)
	//ICP
	dpose = icp.DoICP(LastData,NowData,initGuess,30,dmin_R,dmin_Z);
	del_pose = dpose;

	//Compute global pose
	PointTrans(dpose.p,0,0,IcpPose.phi);

	IcpPose.p.x += dpose.p.x;
	IcpPose.p.y += dpose.p.y;
	IcpPose.phi += dpose.phi;


	//Reset
	RawPose = NewRawPose;
	Point_vector_copy(NowData,LastData);
}
}
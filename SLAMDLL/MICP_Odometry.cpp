#include "StdAfx.h"
#include "MICP_Odometry.h"

MICP_Odometry::MICP_Odometry(void)
{
	If_Init = false;
	datacounter = 0;
}

MICP_Odometry::~MICP_Odometry(void)
{
}

void MICP_Odometry::ICP_Init(const vector <Point> &InitPolarData, const Pose &IniRawPose){
    
	//datacounter = (datacounter+1)%ldsize;
	vector<Point>& LastData = LastPolarDatas[datacounter];// datacounter-> point to the newest data
	Pose& InitPose = LastIcpPoses[datacounter];
	
		
	Point_vector_copy(InitPolarData,LastData);
	InitPose = IniRawPose;
	LastRawPose = IniRawPose;
	IcpPose = IniRawPose;
	//DataPolarToXY(LastData);
	If_Init = true;
}

void MICP_Odometry::UpdateNewData(const vector <Point> &NewPolarData, const Pose &NewRawPose){
	datacounter = (datacounter+1)%ldsize;
	vector<Point>& LastData = LastPolarDatas[datacounter];// datacounter-> point to the newest data
	LastIcpPoses[datacounter] = IcpPose;
	Point_vector_copy(NewPolarData,LastData);
	LastRawPose = NewRawPose;
}

void MICP_Odometry::GenerateModelData(){
	int j=0;
	int i=0;
	int s=0;
    Transform2D t(LastIcpPoses[datacounter]);//Last raw pose
	Pose LastPose;
	Point np;

	LastData.clear();

	for(j=0;j<ldsize;++j){
		LastPose = LastIcpPoses[j];
		t.transform_to_relative(LastPose);
		s = LastPolarDatas[j].size();
		for(i=0;i<s;++i){
			np.x = (LastPolarDatas[j][i].x)*cos(LastPolarDatas[j][i].y + LastPose.phi) +  LastPose.p.x;
			np.y = (LastPolarDatas[j][i].x)*sin(LastPolarDatas[j][i].y + LastPose.phi) +  LastPose.p.y;
			LastData.push_back(np);
		}
	}

}

void MICP_Odometry::GetTransReading(std::vector <Point>& res){
	vector<Point>& LastData = LastPolarDatas[datacounter];// datacounter-> point to the newest data
	Point np;
	res.clear();

	int s = LastData.size();
	for(int i=0;i<s;++i){
			np.x = (LastData[i].x)*cos(LastData[i].y + IcpPose.phi) +  IcpPose.p.x;
			np.y = (LastData[i].x)*sin(LastData[i].y + IcpPose.phi) +  IcpPose.p.y;
			res.push_back(np);
	}

}


void MICP_Odometry::DoICPOdemetry(const vector <Point> &NewPolarData, const Pose &NewRawPose){


	if(!If_Init){
		If_Init = true;
		this->ICP_Init(NewPolarData, NewRawPose);
	}
    
	//datacounter: last new data
	//Pose& LastRawPose = LastPoses[datacounter];
 
    Pose initGuess = NewRawPose;
	Pose dpose = initGuess;
	Point_vector_copy(NewPolarData,NowData);

	Transform2D Tr(LastRawPose);//Last raw pose
	Tr.transform_to_relative(initGuess);


	Pose pose_err = robot_encoder_err(initGuess);//robot pose error
    robot_pose_err_radius(pose_err, NowData,  dmin_R);//additional search range caused by robot pose error
    Laser_measure_err_radius(NowData,  dmin_Z);//additional search range caused by measurement error
    

	DataPolarToXY(NowData);//(r,th)-->(x,y)
	GenerateModelData(); 

	//ICP
	dpose = icp.DoICP(LastData,NowData,initGuess,30,dmin_R,dmin_Z);


	
	//Compute global pose
	Tr.SetBase(IcpPose);
	Tr.transform_to_global(dpose);
	IcpPose = dpose;

	//UpdateData
    UpdateNewData(NewPolarData, NewRawPose);
}


Pose MICP_Odometry::ComputeICPOdemetry(const vector <Point> &NewPolarData, const Pose &NewRawPose){


	if(!If_Init){
		If_Init = true;
		this->ICP_Init(NewPolarData, NewRawPose);
	}
    
	//datacounter: last new data
	//Pose& LastRawPose = LastPoses[datacounter];
 
    Pose initGuess = NewRawPose;
	Pose dpose = initGuess;
	Point_vector_copy(NewPolarData,NowData);

	Transform2D Tr(LastRawPose);//Last raw pose
	Tr.transform_to_relative(initGuess);


	Pose pose_err = robot_encoder_err(initGuess);//robot pose error
    robot_pose_err_radius(pose_err, NowData,  dmin_R);//additional search range caused by robot pose error
    Laser_measure_err_radius(NowData,  dmin_Z);//additional search range caused by measurement error
    

	DataPolarToXY(NowData);//(r,th)-->(x,y)
	GenerateModelData(); 

	//ICP
	dpose = icp.DoICP(LastData,NowData,initGuess,30,dmin_R,dmin_Z);


	
	//Compute global pose
	Tr.SetBase(IcpPose);
	Tr.transform_to_global(dpose);
	
	return dpose;
}

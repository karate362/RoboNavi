#include "stdafx.h"
#include "PassiveSocket.h"       // Include header for active socket object definition
#include "ActiveSocket.h"       // Include header for active socket object definition
#include "Thread_Functions.h"
#include "Function_Pointers.h"
#include "RobotNavi.h"
#include "RobotDlg.h"
#include "math.h"
#include "LaserDlg.h"
#include "JoyDlg.h"
#include "DWAstar.h"
#include "DWA.h"
#include "NavDlg.h"
#include "NaviDraw.h"
#include "DATMO.h"


using namespace RobotTra;
using namespace Geom2D;

//Point lgoal;
vector<GoalPoint> TT;
bool tracked_object = false;

void EndThread(HANDLE& hThread,bool& IsThread){

	DWORD ExitCode;
	IsThread = false;//end the thread

	do{GetExitCodeThread(hThread,&ExitCode);}        
	while(ExitCode == STILL_ACTIVE);//wait until the thread is over
}



DWORD WINAPI LISTEN_THREAD(LPVOID lpt){

    DWORD ExitCode;

	CPassiveSocket socket;
	CActiveSocket** nClient_pt = (CActiveSocket**)lpt;

    socket.Initialize();
    socket.Listen((const uint8 *)"127.0.0.1", 2929);

	while (1){
		(*nClient_pt) = socket.Accept();
    }

	return 0;
}


DWORD WINAPI JOYCTRL_THREAD(LPVOID lpt){


	RobotDlg* rdlg = (RobotDlg*)lpt;
	JoyDlg* jdlg = rdlg->jdlg;
	bool &IsThread = rdlg->IsThread;

	CActiveSocket* nClient = NULL;
	HANDLE hThread; //JoyStick thread
    DWORD dwThreadID;
	hThread=CreateThread(NULL,0, LISTEN_THREAD,(LPVOID)(&nClient), 0,&dwThreadID);//thread start

	int jx = 0;
	int jy = 0;
	int jkey = 0;

	unsigned char SendData[40];

	while(IsThread){

		jdlg->GetPosition(jx,jy);
	    rdlg->JoyVel(jx,jy);

		while( (jkey = jdlg->GetAction()) >=0 ){

			rdlg->JoyKeyDown(jkey);
			if(nClient!=NULL){
				memcpy(SendData,&jkey,sizeof(int));
				nClient->Send((const uint8 *)SendData, 40);
			}
		}
		
		Sleep(100);
	}

	return 0;

}


//Laser Thread function
DWORD WINAPI LASER_THREAD(LPVOID lpt){
	RobotDlg* rdlg = (RobotDlg*)lpt;
	NavDlg* ndlg = rdlg->ndlg;
	PubRobotData& PRData = rdlg->PRData;

	bool &IsThread = rdlg->IsThread2;

	P3robot &probot = rdlg->probot;
	void* laser = rdlg->laserpt;//Laser pointer
	MFC_VIEWER &viewer = rdlg->viewer;

	//Function pointers
	bool (*StartLaser)(void* laser) = rdlg->StartLaser;
	void (*getLaser)(void* laser,std::vector<double> &range,std::vector<double> &rad) = rdlg->getLaser;
	void (*StopLaser)(void* laser) = rdlg->StopLaser;

    double scale = 0.03; //(m/pixel)


	vector<double>range;
	vector<double>rad;
	Pose rawpose;
	DWORD gtime;


	int counter = 0;


	//Start Laser
	if (!StartLaser(laser)){
		rdlg->SetDlgItemText(IDC_EDIT3,(LPCTSTR)"Could not connect to SICK laser... exiting\n");
	}
	

	while(IsThread){	    

		//Get Laser Data
		rawpose.p.x = probot.getX()/1000;//For initialization
	    rawpose.p.y = probot.getY()/1000;
		rawpose.phi = probot.getThRad();
		getLaser(laser,range,rad);
		gtime = timeGetTime();

		PRData.SetSensorData(range,rad,rawpose);
		PRData.SetTime(gtime);
		PRData.SetPoseData(rawpose,probot.getV(),probot.getW());

		viewer.FillAll();//Cleaning
		DrawSensorReading( viewer, range, rad, scale);
		rdlg->OnPaint();

		counter = (counter+1)%5;

		if(counter == 0)	
			rdlg->MeasurementOutput(rdlg->robotpt,rdlg->laserpt,rdlg->lout);

		Sleep(100);
	}
	//TODO: stop laser
	StopLaser(laser);

	ExitThread(0);

	return 0;

}

//Laser Thread function
DWORD WINAPI SLAM_THREAD(LPVOID lpt){//Doing ICP and SLAM
	RobotDlg* rdlg = (RobotDlg*)lpt;
	PubRobotData& PRData = rdlg->PRData;

	ICPDlg* idlg = rdlg->idlg;//Map draw on the ICPDlg  Can not be created in this thread
	MapDlg* mdlg = rdlg->mdlg;

	bool &IsThread = rdlg->IsThread4;

	P3robot &probot = rdlg->probot;
	void* laser = rdlg->laserpt;//Laser pointer

    int i = -1; // Loop counter for readings

	vector<double>range;
	vector<double>rad;

    //ICP
	vector <Point> ICPdata;
	Pose Lastrawpose;
	Pose Nowrawpose;

	Transform2D t(Lastrawpose);
	Pose SLAMpose;// MCLpose + rawpose amount

	PRData.GetSensorData(range,rad,Lastrawpose);//Get sensor data from PRData
	Lastrawpose.p.x -= 10.0; //For initialization


	while(IsThread){	   

		PRData.GetSensorData(range,rad,Nowrawpose);//Get sensor data from PRData

		//If the robot has moved long enough, do icp  && Draw map
		//It should be modified... if the icp result shows that the robot has moved...
		if( dist(Lastrawpose.p,Nowrawpose.p) >= 0.5 || fabs(Lastrawpose.phi - Nowrawpose.phi) >= PI/10){
			if(range.size()>0){
				idlg->CallDoICP(range,rad,Nowrawpose);
				Lastrawpose = Nowrawpose;
				rdlg->SLAM(idlg->getICPpose(),range,rad);
				rdlg->MeasurementOutput(rdlg->robotpt,rdlg->laserpt,rdlg->lout);
			}
		}
		else{/*
			t.SetBase(Lastrawpose);
			t.transform_to_relative(Nowrawpose);
			t.SetBase(mdlg->getMCLpose());
			t.transform_to_global(Nowrawpose);
			SLAMpose = Nowrawpose;*/
		}

		Sleep(500);
	}

	ExitThread(0);

	return 0;

}

DWORD WINAPI NAVI_THREAD(LPVOID lpt){//Tracking using Tratree

	
	RobotDlg* rdlg = (RobotDlg*)lpt;
  
	bool &IsThread = rdlg->IsThread3;

	P3robot &probot = rdlg->probot;
	
	PubRobotData& PRData = rdlg->PRData;

	NavDlg* ndlg = rdlg->ndlg;
	MFC_VIEWER &viewer = ndlg->viewer;

//////////////////////////////////////////////
    DWAstar dywstar;
	ObsArray Lobs;

	double v;
	double w;
	Pose rpose;
	Point lgoal;
	Transform2D t(rpose);

	//////////////////////////////////////////////////
	ifstream in1("DWAp.txt");
    ifstream in2("obj_weight.txt");
    ifstream in3("DWAstarp.txt");
    double at;
    double sa;
    double sm;
    int ma;
    int hur;

    int Sleeptime=(int)(dywstar.dyw.delta_t*1000.0);
    in3>>sa>>at>>sm>>ma>>hur>>dywstar.p_dis>>dywstar.p_obser;
    dywstar.initDWA(in1,in2);
	dywstar.initDWAstar(at,sm,0,ma,hur);





//////////////////////////////////////////////////////////////
//double vlim[] = {0.5,0,0.8,-0.8,0.5,0.8,0.05,0.1,0.5};
double vlim[] = {0.7,0,0.8,-0.8,0.5,0.8,0.07,0.1,0.5};
VelItv vitv(1,80);
ND nd;
DWA dwa(vlim,vitv);
TraTree DWAtree;
DWAtree.ndptr = &(nd);
DWAtree.dwa = &dwa;
	double* alpha = DWAtree.alpha;
	ifstream in4("alpha.txt");
	in4>>alpha[0]>>alpha[1]>>alpha[2];
////////////////////////////////////////////////////////////


    int i = -1; // Loop counter for readings

	vector<double>range;
	vector<double>rad;

	
	//Function pointers
	//void (*getLaser)(void* laser,std::vector<double> &range,std::vector<double> &rad) = rdlg->getLaser;

	char velstr[32];
	double scale = 0.03; //(m/pixel)
	int ratio = (int)(1/scale);
	char x_goal[30];
	char y_goal[30];
	char pseudo_x_goal[30];
	char pseudo_y_goal[30];
	double goal_x = 0;
	double goal_y = 0;
	double theta = 0;
	double pseudo_goal_x = 0;
	double pseudo_goal_y = 0;
	double slope = 0;
	double distance;
	FILE * lout;
	int s = 0;
	DWORD ctime;
	Point p1;
	vector <GoalPoint> temp_lgoal;

	//lout = fopen("theta.txt","w");
	while(IsThread){
		ctime = timeGetTime();
		//Get Laser Data
        PRData.GetSensorData(range,rad,rpose);//Get sensor data from PRData
		//pseudo_goal_x = TT.size();
		//pseudo_goal_y = 0;
		if (TT.size() != 0){
			temp_lgoal.clear();
			temp_lgoal.resize(TT.size());
			copy(TT.begin(),TT.end(),temp_lgoal.begin());
			lgoal.x = temp_lgoal[0].x;
			lgoal.y = temp_lgoal[0].y;
			if (temp_lgoal.size() > 5){
				pseudo_goal_x = temp_lgoal[5].x;
				pseudo_goal_y = temp_lgoal[5].y;
			}
		}
		else{
			lgoal.x = 0;
			lgoal.y = 0;
			//pseudo_goal_x = 0;
			//pseudo_goal_y = 0;
		}
		goal_x = lgoal.x;
		goal_y = lgoal.y;
		
		sprintf(x_goal,"x: %.2f",goal_x);
		sprintf(y_goal,"y: %.2f",goal_y);
		sprintf(pseudo_x_goal,"x: %.2f",pseudo_goal_x);
		sprintf(pseudo_y_goal,"y: %.2f",pseudo_goal_y);
		ndlg->SetDlgItemText(IDC_EDIT4,(LPCSTR)x_goal);
		ndlg->SetDlgItemText(IDC_EDIT5,(LPCSTR)y_goal);
		ndlg->SetDlgItemText(IDC_EDIT2,(LPCSTR)pseudo_x_goal);
		ndlg->SetDlgItemText(IDC_EDIT3,(LPCSTR)pseudo_y_goal);

		if (lgoal.x !=0 || lgoal.y != 0){
			
			s = range.size();

			for(int j=0;j<s;++j){
				p1.x = range[j]*cos(rad[j]);
				p1.y = range[j]*sin(rad[j]);
				if (dist(lgoal,p1) < 1.0){
					range[j] = 1000;
				}
			}

			Lobs.set_obs_laser_array(range,rad);

			//slope = lgoal.x / lgoal.y;

			distance = sqrt(lgoal.x*lgoal.x + lgoal.y*lgoal.y);

			theta = atan2(lgoal.y,lgoal.x)*180.0/3.1415;

			//set velocities
			if (distance < 1){	// Stop Condition - close to target
				v = 0;
				w = 0;
			}
			else{ // DWA mode
				///////////////////////////////////////////////
				/*
				dywstar.ndptr->ResetReg();
				dywstar.ndptr->Set_All_Intervals(range,rad);
				v = probot.getV();
				w = probot.getW();
				DoDWAstar_Tracking(dywstar, Lobs, v, w,temp_lgoal);*/
				//////////////////////////////////////////////////
				nd.ResetAll();
				nd.Set_All_Intervals(range,rad);
				nd.Find_Gap(0.8);
				nd.Find_REGION();
				nd.Find_Door(0.49);
				DoTracking(DWAtree, Lobs, v, w,  temp_lgoal);

				///////////////////////////////////////////////////
			}

			probot.robot.setVel(v*1000);
			probot.robot.setRotVel(w*180.0/3.1415);
		
			viewer.FillAll();
			DrawSensorReading( viewer, range, rad, scale);
			//DrawDWATreeOnView(viewer,dywstar,0,0,0,ratio);
	DrawBestPath( viewer,  DWAtree, scale);
	DrawTargetPath(viewer, TT, scale);
	//DrawNDOnView(viewer,nd,VFHsize,(int)(1/scale));
	//DrawVAsOnView(viewer, DWAtree.VAs[DWAtree.VAs.size()-1], (int)(1/scale));
			ndlg->OnPaint();
		}
		else{
			probot.robot.setVel(0);
			probot.robot.setRotVel(0);
		}
		ctime = timeGetTime()-ctime;
		if( (int)ctime < 400)
			Sleep(500 - (int) ctime);
	}

	return 0;
}



DWORD WINAPI NAVI2_THREAD(LPVOID lpt){//Tracking using dwastar

	
	RobotDlg* rdlg = (RobotDlg*)lpt;
  
	bool &IsThread = rdlg->IsThread3;

	P3robot &probot = rdlg->probot;
	
	PubRobotData& PRData = rdlg->PRData;

	NavDlg* ndlg = rdlg->ndlg;
	MFC_VIEWER &viewer = ndlg->viewer;

//////////////////////////////////////////////
    DWAstar dywstar;
	ObsArray Lobs;

	double v;
	double w;
	Pose rpose;
	Point lgoal;
	Transform2D t(rpose);

	//////////////////////////////////////////////////
	ifstream in1("DWAp.txt");
    ifstream in2("obj_weight.txt");
    ifstream in3("DWAstarp.txt");
    double at;
    double sa;
    double sm;
    int ma;
    int hur;

    int Sleeptime=(int)(dywstar.dyw.delta_t*1000.0);
    in3>>sa>>at>>sm>>ma>>hur>>dywstar.p_dis>>dywstar.p_obser;
    dywstar.initDWA(in1,in2);
	dywstar.initDWAstar(at,sm,0,ma,hur);


    int i = -1; // Loop counter for readings

	vector<double>range;
	vector<double>rad;

	
	//Function pointers
	//void (*getLaser)(void* laser,std::vector<double> &range,std::vector<double> &rad) = rdlg->getLaser;

	char velstr[32];
	double scale = 0.03; //(m/pixel)
	int ratio = (int)(1/scale);
	char x_goal[30];
	char y_goal[30];
	char pseudo_x_goal[30];
	char pseudo_y_goal[30];
	double goal_x = 0;
	double goal_y = 0;
	double theta = 0;
	double pseudo_goal_x = 0;
	double pseudo_goal_y = 0;
	double slope = 0;
	double distance;
	FILE * lout;
	int s = 0;
	DWORD ctime;
	Point p1;
	vector <GoalPoint> temp_lgoal;

	//lout = fopen("theta.txt","w");
	while(IsThread){
		ctime = timeGetTime();
		//Get Laser Data
        PRData.GetSensorData(range,rad,rpose);//Get sensor data from PRData
		//pseudo_goal_x = TT.size();
		//pseudo_goal_y = 0;
		if (TT.size() != 0){
			temp_lgoal.clear();
			temp_lgoal.resize(TT.size());
			copy(TT.begin(),TT.end(),temp_lgoal.begin());
			lgoal.x = temp_lgoal[0].x;
			lgoal.y = temp_lgoal[0].y;
			if (temp_lgoal.size() > 5){
				pseudo_goal_x = temp_lgoal[5].x;
				pseudo_goal_y = temp_lgoal[5].y;
			}
		}
		else{
			lgoal.x = 0;
			lgoal.y = 0;
			//pseudo_goal_x = 0;
			//pseudo_goal_y = 0;
		}
		goal_x = lgoal.x;
		goal_y = lgoal.y;
		
		sprintf(x_goal,"x: %.2f",goal_x);
		sprintf(y_goal,"y: %.2f",goal_y);
		sprintf(pseudo_x_goal,"x: %.2f",pseudo_goal_x);
		sprintf(pseudo_y_goal,"y: %.2f",pseudo_goal_y);
		ndlg->SetDlgItemText(IDC_EDIT4,(LPCSTR)x_goal);
		ndlg->SetDlgItemText(IDC_EDIT5,(LPCSTR)y_goal);
		ndlg->SetDlgItemText(IDC_EDIT2,(LPCSTR)pseudo_x_goal);
		ndlg->SetDlgItemText(IDC_EDIT3,(LPCSTR)pseudo_y_goal);

		if (lgoal.x !=0 || lgoal.y != 0){
			
			s = range.size();

			for(int j=0;j<s;++j){
				p1.x = range[j]*cos(rad[j]);
				p1.y = range[j]*sin(rad[j]);
				if (dist(lgoal,p1) < 1.0){
					range[j] = 1000;
				}
			}

			Lobs.set_obs_laser_array(range,rad);

			//slope = lgoal.x / lgoal.y;

			distance = sqrt(lgoal.x*lgoal.x + lgoal.y*lgoal.y);

			theta = atan2(lgoal.y,lgoal.x)*180.0/3.1415;

			//set velocities
			if (distance < 1){	// Stop Condition - close to target
				v = 0;
				w = 0;
			}
			else{ // DWA mode
				///////////////////////////////////////////////
				
				dywstar.ndptr->ResetReg();
				dywstar.ndptr->Set_All_Intervals(range,rad);
				v = probot.getV();
				w = probot.getW();
				DoDWAstar_Tracking(dywstar, Lobs, v, w,temp_lgoal);
			}

			probot.robot.setVel(v*1000);
			probot.robot.setRotVel(w*180.0/3.1415);
		
			viewer.FillAll();
			DrawSensorReading( viewer, range, rad, scale);
			DrawDWATreeOnView(viewer,dywstar,0,0,0,ratio);
	//DrawBestPath( viewer,  DWAtree, scale);
	//DrawTargetPath(viewer, TT, scale);
	//DrawNDOnView(viewer,nd,VFHsize,(int)(1/scale));
	//DrawVAsOnView(viewer, DWAtree.VAs[DWAtree.VAs.size()-1], (int)(1/scale));
			ndlg->OnPaint();
		}
		else{
			probot.robot.setVel(0);
			probot.robot.setRotVel(0);
		}
		ctime = timeGetTime()-ctime;
		if( (int)ctime < 200)
			Sleep(250 - (int) ctime);
	}

	return 0;
}


DWORD WINAPI NAVI3_THREAD(LPVOID lpt){//Tracking psudeo goal using TraTree

	
	RobotDlg* rdlg = (RobotDlg*)lpt;

	bool &IsThread = rdlg->IsThread3;

	P3robot &probot = rdlg->probot;
	
	PubRobotData& PRData = rdlg->PRData;

	NavDlg* ndlg = rdlg->ndlg;
	MFC_VIEWER &viewer = ndlg->viewer;


	ObsArray Lobs;

	double v;
	double w;
	Pose rpose;
	//Point ggoal;
	Point lgoal;
	lgoal.x = 20;
	lgoal.y = 10;
	//ggoal.x = 40;
	//ggoal.y = -5;
	Transform2D t(rpose);

//////////////////////////////////////////////////////////////
//double vlim[] = {0.5,0,0.8,-0.8,0.5,0.8,0.05,0.1,0.5};
double vlim[] = {0.8,0,0.8,-0.8,0.6,0.8,0.05,0.1,0.5};
VelItv vitv(1,80);
ND nd;
DWA dwa(vlim,vitv);
TraTree DWAtree;
DWAtree.ndptr = &(nd);
DWAtree.dwa = &dwa;
	double* alpha = DWAtree.alpha;
	ifstream in4("alpha.txt");
	in4>>alpha[0]>>alpha[1]>>alpha[2];



////////////////////////////////////////////////////////////

   DWAstar dywstar;
	ifstream in1("DWAp.txt");
    ifstream in2("obj_weight.txt");
    ifstream in3("DWAstarp.txt");
    double at;
    double sa;
    double sm;
    int ma;
    int hur;

    int Sleeptime=(int)(dywstar.dyw.delta_t*1000.0);
    in3>>sa>>at>>sm>>ma>>hur>>dywstar.p_dis>>dywstar.p_obser;
    dywstar.initDWA(in1,in2);
	dywstar.initDWAstar(at,sm,0,ma,hur);


//////////////////////////////////////////////////////////////
    int i = -1; // Loop counter for readings

	vector<double>range;
	vector<double>rad;

	DWORD ctime;
	char velstr[32];
	double scale = 0.03; //(m/pixel)
	double ratio = (1.0/scale);

	vector<GoalPoint> TT;
    GoalPoint ngp;

	ifstream in("inTra.txt");
	ifstream ing("ingoal.txt");

	ing>>lgoal.x>>lgoal.y;
    //ofstream out("outTra.txt");
	TT.clear();

	while(in>>ngp.x>>ngp.y>>ngp.t){
		TT.push_back(ngp);
	}

	
	while(IsThread){

		//time
		ctime = timeGetTime();

		//Get Laser Data
		PRData.GetSensorData(range,rad,rpose);//Get sensor data from PRData

		Lobs.set_obs_laser_array(range,rad);

		v = probot.getV();
		w = probot.getW(); 

//////////////////// DWA* before 3/18 ///////////////////////////
		//dywstar.ndptr->ResetReg();
		//dywstar.ndptr->Set_All_Intervals(range,rad);
		//DoDWAstar(dywstar, Lobs, v, w, lgoal.x, lgoal.y); //Why??
	    //DoDWAstar_Tracking(dywstar, Lobs, v, w,TT);
//////////////////////////////////////////////////////////


///////////////////// DWA* after 3/10 /////////////////////////
		nd.ResetAll();
		nd.Set_All_Intervals(range,rad);
		nd.Find_Gap(0.8);
		nd.Find_REGION();
		nd.Find_Door(0.49);
        //DoNavi(DWAtree, Lobs, v, w, lgoal.x, lgoal.y);//initobs: (0,0,0)
		DoTracking(DWAtree, Lobs, v, w,  TT);
////////////////////////////////////////////////////////////

		probot.robot.setVel(v*1000);
	    probot.robot.setRotVel(w*180.0/3.1415);

		viewer.FillAll();
		DrawSensorReading( viewer, range, rad, scale);

        ////// Before 3/18 ///////////////
		//DrawDWATreeOnView(viewer,dywstar,0,0,0,(int)ratio);
		/////////////////////////////////

		////// After 3/18 ///////////////
		DrawBestPath( viewer,  DWAtree, scale);
		DrawTargetPath(viewer, TT, scale);
		/////////////////////////////////

		ndlg->OnPaint();

		ctime = timeGetTime() - ctime;

		sprintf(velstr,"(%.2f,%.2f) %dms",v,w,ctime);
        ndlg->SetDlgItemText(IDC_EDIT1,(LPCSTR)velstr);

		if(500 - (int)ctime > 50)
			Sleep(500 - (int)ctime);

		//Sleep(200);
	}
	

	return 0;
}
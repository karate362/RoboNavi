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
#include "DATMODlg.h"


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

	//JoyDlg* jdlg = (JoyDlg*)lpt;
	//JoyStick &js = myjoy;

    DWORD ExitCode;

	CPassiveSocket socket;
	CActiveSocket** nClient_pt = (CActiveSocket**)lpt;

    socket.Initialize();
    socket.Listen((const uint8 *)"127.0.0.1", 2929);

	while (1){

		(*nClient_pt) = socket.Accept();
/*
		if(nClient != NULL){//new connection
			jdlg->pClient = nClient;
			nClient = NULL;
			jdlg->pClients.push_back(jdlg->pClient);
			jdlg->SetDlgItemText(IDC_EDIT3,"Accept!!");
			jdlg->Thread_start2(); //Thread_start function will end the last thread if it is still active
		}*/
    }

	return 0;
}

/*

DWORD WINAPI JOYCTRL_THREAD(LPVOID lpt){


	RobotDlg* rdlg = (RobotDlg*)lpt;
	JoyDlg* jdlg = rdlg->jdlg;
	bool &IsThread = rdlg->IsThread;


	int jx = 0;
	int jy = 0;
	int jkey = 0;

	while(IsThread){

		jdlg->GetPosition(jx,jy);
	    rdlg->JoyVel(jx,jy);

		while( (jkey = jdlg->GetAction()) >=0 ){

			rdlg->JoyKeyDown(jkey);
		}
		
		Sleep(100);
	}

	return 0;

}
*/



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
		if( dist(Lastrawpose.p,Nowrawpose.p) >= 0.5 || fabs(Lastrawpose.phi - Nowrawpose.phi) >= PI/10 ){
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



DWORD WINAPI DATMO_THREAD(LPVOID lpt){//Doing ICP and SLAM
	RobotDlg* rdlg = (RobotDlg*)lpt;
	PubRobotData& PRData = rdlg->PRData;

	ICPDlg* idlg = rdlg->idlg;//Map draw on the ICPDlg  Can not be created in this thread
	DATMODlg* ddlg = rdlg->ddlg;

	bool &IsThread = rdlg->IsThread5;

    int i = -1; // Loop counter for readings

	vector<double>range;
	vector<double>rad;

	vector<double>strange;//static, used for ICP
	vector<double>strad;

    //ICP
	vector <Point> ICPdata;
	vector <Point> MOs;
	Pose Lastrawpose;
	Pose Nowrawpose;

	Transform2D t(Lastrawpose);
	Pose ICPpose;// 

	PRData.GetSensorData(range,rad,Lastrawpose);//Get sensor data from PRData
	Lastrawpose.p.x -= 10.0; //For initialization

	//DATMO
	DATMO &datmo = ddlg->datmo;


	//Computing time
	DWORD ctime;
	DWORD last_scantime;
	DWORD dt;
	DWORD scan_time;

	vector<Geom2D::Pedestrian> tracked_pedestrian;
	Occupied_ID current_ID;
	
	int j = 0;
	
	//bool tracked_object;
	double ID1_x = 0;
	double ID1_y = 0;
	double ID2_x = 0;
	double ID2_y = 0;
	double ID1_size = 0;
	double ID3_x = 0;
	double ID3_y = 0;
	double ID4_x = 0;
	double ID4_y = 0;
	double ID5_x = 0;
	double ID5_y = 0;
	char x_id1[30];
	char y_id1[30];
	char x_id2[30];
	char y_id2[30];
	char x_id3[30];
	char y_id3[30];
	char x_id4[30];
	char y_id4[30];
	char x_id5[30];
	char y_id5[30];
	char size_id1[30];
	char ID1_status[30];
	char ID2_status[30];
	char ID3_status[30];
	char ID4_status[30];
	char ID5_status[30];
	char ID6_status[30];
	bool exist;
	bool check_present;
	FILE * ICPout;
	FILE * theta_out;
	FILE * ped_out;
	Pose ICPPose;
	for (j = 0; j < 6; j++)
		current_ID.used_ID[j] = false;

	//lout = fopen("ICP.txt","w");
	Point ggoal;
	Point prev_ggoal;
	Point lgoal;
	GoalPoint temp_goal;	
	vector<GoalPoint> temp_lgoal;
	ggoal.x = 0;
	ggoal.y = 0;
	lgoal.x = 0;
	lgoal.y = 0;
	int u = 0;
	double vel_x = 0;
	double vel_y = 0;
	double theta;
	bool ped_check;
	//ICPout = fopen("Online_ICP.txt","w");
	theta_out = fopen("theta.txt","w");
	ped_out = fopen("Online_Pedestrian.txt","w");
	while(IsThread){	   
		
		ctime = timeGetTime();
		tracked_object = false;
		check_present = false;
		exist = false;
		PRData.GetSensorData(range,rad,Nowrawpose);//Get sensor data from PRData
		PRData.GetSensorData(strange,strad,Nowrawpose);//Get sensor data from PRData
		dt = PRData.gtime - last_scantime;
		last_scantime = PRData.gtime;
		
		if(range.size()>0){
			ped_check = false;


            //////////loop/////////////
			//idlg->CallDoICP(range,rad,Nowrawpose);
            //Lastrawpose = Nowrawpose;
#if 0
			for(j=0;j<1;++j){
				ICPPose = idlg->CallComputeICP(strange,strad,Nowrawpose);			
				datmo.DetectMOs(ICPPose,range,rad);//Do it after there are enough scans?
				datmo.getStaticReadings(strange,strad);
			}
#endif
			////////////////////////////
			idlg->CallDoICP(range,rad,Nowrawpose);
            Lastrawpose = Nowrawpose;
			ICPPose = idlg->getICPpose();
			datmo.DetectMOs(ICPPose,range,rad);//Do it after there are enough scans?
#if 0
			//lout = fopen("Online_ICP.txt","a");
			fprintf(ICPout,"%.3f %.3f %.3f ",ICPPose.p.x,ICPPose.p.y,ICPPose.phi);
			for (j = 0; j < range.size(); j++){
				fprintf(ICPout,"%.3f ",range[j]);
			}
			fprintf(ICPout,"\n");
			//fclose(lout);
#endif

			ddlg->DrawDATMO(tracked_pedestrian, current_ID, scan_time);
			if (tracked_pedestrian.size() != 0) {
				for (j = 0;j < tracked_pedestrian.size(); j++){
					if (tracked_pedestrian[j].id == 1){
						//ID1_x = tracked_pedestrian[j].x;
						//ID1_y = tracked_pedestrian[j].y;
						ID1_x = tracked_pedestrian[j].appear_time;
						ID1_y = ICPPose.p.y;
						ID1_size = tracked_pedestrian[j].size;
						//theta = atan(tracked_pedestrian[j].x / tracked_pedestrian[j].y)*180/3.1415;
						theta = atan2(tracked_pedestrian[j].y , tracked_pedestrian[j].x)*180/3.1415;						

#if 0
						if (ped_check == false){
							//lout = fopen("Online_Pedestrian.txt","a");
							fprintf(ped_out,"%.3f %.3f %.3f %.3f %.3f",tracked_pedestrian[j].x,tracked_pedestrian[j].y, tracked_pedestrian[j].Tmean[0],tracked_pedestrian[j].Tmean[1], theta);
							//fprintf(lout,"\n");
							//fclose(lout);
							ped_check = true;
						}
						#endif
						#if 0
							//lout = fopen("Online_theta.txt","a");
							fprintf(theta_out,"%.3f ",theta);
							//fprintf(lout,"\n");
							//fclose(lout);
						#endif

						if (tracked_pedestrian[j].present == 1) {
							ggoal.x = tracked_pedestrian[j].Tmean[0];
							ggoal.y = tracked_pedestrian[j].Tmean[1];
							vel_x = tracked_pedestrian[j].Tmean[2];
							vel_y = tracked_pedestrian[j].Tmean[3];
							tracked_object = true;
							check_present = true;
						}
						else{
							if (check_present == false){
								ggoal.x = tracked_pedestrian[j].Tmean[0];
								ggoal.y = tracked_pedestrian[j].Tmean[1];
								vel_x = tracked_pedestrian[j].Tmean[2];
								vel_y = tracked_pedestrian[j].Tmean[3];
								tracked_object = true;
							}
						
						}
					}
					else if (tracked_pedestrian[j].id == 2){
						ID2_x = tracked_pedestrian[j].x;
						ID2_y = tracked_pedestrian[j].y;
						//ggoal.x = tracked_pedestrian[j].x;
						//ggoal.y = tracked_pedestrian[j].y;
					}
					else if (tracked_pedestrian[j].id == 3){
						ID3_x = tracked_pedestrian[j].x;
						ID3_y = tracked_pedestrian[j].y;
					}
					else if (tracked_pedestrian[j].id == 4){
						ID4_x = tracked_pedestrian[j].x;
						ID4_y = tracked_pedestrian[j].y;
					}
					else if (tracked_pedestrian[j].id == 5){
						ID5_x = tracked_pedestrian[j].x;
						ID5_y = tracked_pedestrian[j].y;
					}
				}
			}
#if 0
			if (ped_check == false){
				//ped_out = fopen("Online_Pedestrian.txt","a");
				fprintf(ped_out,"%.3f %.3f %.3f %.3f %.3f",0.0,0.0,0.0,0.0,0.0);
				
				//fclose(lout);
			}
				//ped_out = fopen("Online_Pedestrian.txt","a");
				fprintf(ped_out,"\n");
				//fclose(lout);
#endif
			sprintf(ID1_status, "ID1: %.2f", current_ID.used_ID[1]);
		
			sprintf(ID2_status, "ID2: %.2f", current_ID.used_ID[2]);
		
			sprintf(ID3_status, "ID3: %.2f", current_ID.used_ID[3]);
		
			sprintf(ID4_status, "ID4: %.2f", current_ID.used_ID[4]);
		
			sprintf(ID5_status, "ID5: %.2f", current_ID.used_ID[5]);
		
			
			sprintf(x_id1,"x: %.2f",ID1_x);
			sprintf(y_id1,"y: %.2f",ID1_y);
			//sprintf(size_id1,"ID1 Size: %.2f",ID1_size);
			sprintf(x_id2,"x: %.2f",ID2_x);
			sprintf(y_id2,"y: %.2f",ID2_y);
			sprintf(x_id3,"x: %.2f",ID3_x);
			sprintf(y_id3,"y: %.2f",ID3_y);
			sprintf(x_id4,"x: %.2f",ID4_x);
			sprintf(y_id4,"y: %.2f",ID4_y);
			sprintf(x_id5,"x: %.2f",ID5_x);
			sprintf(y_id5,"y: %.2f",ID5_y);

			ddlg->SetDlgItemText(IDC_EDIT4,(LPCSTR)x_id1);
			ddlg->SetDlgItemText(IDC_EDIT5,(LPCSTR)y_id1);

			ddlg->SetDlgItemText(IDC_EDIT6,(LPCSTR)x_id2);
			ddlg->SetDlgItemText(IDC_EDIT7,(LPCSTR)y_id2);

			ddlg->SetDlgItemText(IDC_EDIT2,(LPCSTR)x_id3);
			ddlg->SetDlgItemText(IDC_EDIT3,(LPCSTR)y_id3);

			ddlg->SetDlgItemText(IDC_EDIT8,(LPCSTR)x_id4);
			ddlg->SetDlgItemText(IDC_EDIT14,(LPCSTR)y_id4);

			ddlg->SetDlgItemText(IDC_EDIT15,(LPCSTR)x_id5);
			ddlg->SetDlgItemText(IDC_EDIT16,(LPCSTR)y_id5);
			//ddlg->SetDlgItemText(IDC_EDIT8,(LPCSTR)size_id1);

			ddlg->SetDlgItemText(IDC_EDIT9,(LPCSTR)ID1_status);
			ddlg->SetDlgItemText(IDC_EDIT10,(LPCSTR)ID2_status);
			ddlg->SetDlgItemText(IDC_EDIT11,(LPCSTR)ID3_status);
			ddlg->SetDlgItemText(IDC_EDIT12,(LPCSTR)ID4_status);
			ddlg->SetDlgItemText(IDC_EDIT13,(LPCSTR)ID5_status);

			if (tracked_object == false){
				temp_lgoal.clear();
				temp_goal.x = 0;
				temp_goal.y = 0;
				temp_goal.t = 0;
				temp_lgoal.push_back(temp_goal);				
				//vel_x = 0;
				//vel_y = 0;
			}
			else{
				t.SetBase(ICPPose);
				lgoal = ggoal;
				t.transform_to_relative(lgoal);
				//slope = lgoal.x / lgoal.y;
				temp_lgoal.clear();
				temp_goal.x = lgoal.x;
				temp_goal.y = lgoal.y;
				temp_goal.t = 0;
				temp_lgoal.push_back(temp_goal);
				prev_ggoal.x = ggoal.x;
				prev_ggoal.y = ggoal.y;
				for (u = 0; u < 5; u++){
					ggoal.x = prev_ggoal.x + vel_x;
					ggoal.y = prev_ggoal.y + vel_y;
					lgoal.x = ggoal.x;
					lgoal.y = ggoal.y;
					t.transform_to_relative(lgoal);
					temp_goal.x = lgoal.x;
					temp_goal.y = lgoal.y;
					temp_goal.t = (double)(u+1);
					temp_lgoal.push_back(temp_goal);
					prev_ggoal.x = ggoal.x;
					prev_ggoal.y = ggoal.y;
				}
			}
			TT.clear();
			TT.resize(temp_lgoal.size());
			copy(temp_lgoal.begin(),temp_lgoal.end(),TT.begin());
		
			datmo.UpdateMap();
		}

		ctime = timeGetTime()-ctime;
		scan_time = ctime;
		ddlg->SetDlgItemInt(IDC_EDIT1,ctime);

		if((int)ctime < 400)
			Sleep(500 - ctime);
	}

	delete ddlg;
	delete idlg;

	ExitThread(0);

	return 0;

}


//Laser Thread function
DWORD WINAPI DATMO_THREAD_OFFLINE(LPVOID lpt){//Doing ICP and SLAM
	RobotDlg* rdlg = (RobotDlg*)lpt;
	PubRobotData& PRData = rdlg->PRData;

	ICPDlg* idlg = rdlg->idlg;//Map draw on the ICPDlg  Can not be created in this thread
	DATMODlg* ddlg = rdlg->ddlg;

	bool &IsThread = rdlg->IsThread5;

    int i = -1; // Loop counter for readings
	int j = 0;

	vector<double>range;
	vector<double>rad;
	
	vector<double>strange;//static, used for ICP
	vector<double>strad;

    //ICP
	vector <Point> ICPdata;
	vector <Point> MOs;
	Pose Lastrawpose;
	Pose Nowrawpose;

	Transform2D t(Lastrawpose);
	Pose ICPpose;// 
	Point Ped_pos;
	Point lgoal;


////////////////////////////////////////////////
    FILE* lin = fopen("LASER.txt","r");
	PRData.GetLaserDataFromText(lin);
	PRData.GetSensorData(range,rad,Lastrawpose);//Get sensor data from PRData
////////////////////////////////////////////////////////
	Lastrawpose.p.x -= 10.0; //For initialization

	//DATMO
	DATMO &datmo = ddlg->datmo;


	//Computing time
	DWORD ctime;
	DWORD dt;
	DWORD last_scantime;
	DWORD scan_time;

	vector<Geom2D::Pedestrian> tracked_pedestrian;
	Occupied_ID current_ID;
	
	
	double ID1_x = 0;
	double ID1_y = 0;
	double ID2_x = 0;
	double ID2_y = 0;
	double ID1_size = 0;
	double ID3_x = 0;
	double ID3_y = 0;
	double ID4_x = 0;
	double ID4_y = 0;
	double ID5_x = 0;
	double ID5_y = 0;
	char x_id1[30];
	char y_id1[30];
	char x_id2[30];
	char y_id2[30];
	char x_id3[30];
	char y_id3[30];
	char x_id4[30];
	char y_id4[30];
	char x_id5[30];
	char y_id5[30];
	char size_id1[30];
	char ID1_status[30];
	char ID2_status[30];
	char ID3_status[30];
	char ID4_status[30];
	char ID5_status[30];
	char ID6_status[30];
	FILE * ICPout;
	FILE * theta_out;
	FILE * ped_out;
	FILE * ped1_out;

	Pose ICPPose;
	double theta;
	bool ped_check;
	bool ped1_check;
	int a = 0;
	for (j = 0; j < 6; j++)
		current_ID.used_ID[j] = false;
	ICPout = fopen("ICP_Map.txt","w");
	
	dt = 1000;

	while(IsThread && PRData.GetLaserDataFromText(lin)){	   

		ctime = timeGetTime();

		/////////////////////////////////////////////////////////
//		PRData.GetSensorData(range,rad,Nowrawpose);//Get sensor data from PRData
		/////////////////////////////////////////////////////////

		PRData.GetSensorData(range,rad,Nowrawpose);//Get sensor data from PRData
		PRData.GetSensorData(strange,strad,Nowrawpose);//Get sensor data from PRData
		//dt = PRData.gtime - last_scantime;
		
		last_scantime = PRData.gtime;

		if(range.size()>0){
			ped_check = false;
			ped1_check = false;
			//////////loop/////////////
			//idlg->CallDoICP(range,rad,Nowrawpose);
            //Lastrawpose = Nowrawpose;
#if 1
			for(j=0;j<1;++j){
				ICPPose = idlg->CallComputeMICP(range,rad,Nowrawpose);			
				datmo.DetectMOs(ICPPose,range,rad);//Do it after there are enough scans?
				datmo.getStaticReadings(strange,strad);
			}			
#endif		
			idlg->CallDoMICP(range,rad,Nowrawpose);
			Lastrawpose = Nowrawpose;
			//ICPPose = idlg->getICPpose();		
#if 1
			//lout = fopen("ICP2.txt","a");
			fprintf(ICPout,"%.3f %.3f %.3f ",ICPPose.p.x,ICPPose.p.y,ICPPose.phi);
			for (j = 0; j < strange.size(); j++){
				fprintf(ICPout,"%.3f ",strange[j]);
			}
			fprintf(ICPout,"\n");
			//fclose(lout);
#endif
			//ICPPose = idlg->getICPpose();

			datmo.DetectMOs(idlg->getICPpose(),range,rad);//Do it after there are enough scans?

			ddlg->DrawDATMO(tracked_pedestrian, current_ID, last_scantime);


			if (tracked_pedestrian.size() != 0) {
				for (j = 0;j < tracked_pedestrian.size(); j++){
					if (tracked_pedestrian[j].id == 1){
						ID1_x = tracked_pedestrian[j].appear_time;
						ID1_y = tracked_pedestrian[j].size;
						
						t.SetBase(ICPPose);
						lgoal.x = tracked_pedestrian[j].x;
						lgoal.y = tracked_pedestrian[j].y;
						t.transform_to_relative(lgoal);
						theta = atan(lgoal.x / lgoal.y)*180/3.1415;
						#if 1
						if (ped_check == false){
							ped_out = fopen("Offline_Pedestrian.txt","a");
							//fprintf(ped_out,"%.3f %.3f %.3f %.3f %.3f",tracked_pedestrian[j].x,tracked_pedestrian[j].y, tracked_pedestrian[j].Tmean[0],tracked_pedestrian[j].Tmean[1], theta);
							fprintf(ped_out,"%.3f %.3f %.3f %.3f %.3f",lgoal.x,lgoal.y, tracked_pedestrian[j].Tmean[0],tracked_pedestrian[j].Tmean[1], theta);
							//fprintf(lout,"\n");
							fclose(ped_out);
							ped_check = true;
						}
						#endif
						#if 0
							theta_out = fopen("Offline_theta.txt","a");
							fprintf(theta_out,"%.3f ",theta);
							//fprintf(lout,"\n");
							fclose(theta_out);
							
						#endif
						//ID1_size = tracked_pedestrian[j].size;
					}
					else if (tracked_pedestrian[j].id == 2){
						ID2_x = tracked_pedestrian[j].x;
						ID2_y = tracked_pedestrian[j].y;
						//theta = atan(tracked_pedestrian[j].x / tracked_pedestrian[j].y)*180/3.1415;
						t.SetBase(ICPPose);
						lgoal.x = tracked_pedestrian[j].x;
						lgoal.y = tracked_pedestrian[j].y;
						t.transform_to_relative(lgoal);
						theta = atan(lgoal.x / lgoal.y)*180/3.1415;
						#if 1
						if (ped1_check == false){
							ped1_out = fopen("Offline_Pedestrian1.txt","a");
							//fprintf(ped_out,"%.3f %.3f %.3f %.3f %.3f",tracked_pedestrian[j].x,tracked_pedestrian[j].y, tracked_pedestrian[j].Tmean[0],tracked_pedestrian[j].Tmean[1], theta);
							fprintf(ped1_out,"%.3f %.3f %.3f %.3f %.3f",lgoal.x,lgoal.y, tracked_pedestrian[j].Tmean[0],tracked_pedestrian[j].Tmean[1], theta);
							//fprintf(lout,"\n");
							fclose(ped1_out);
							ped1_check = true;
						}
						#endif
					}
					else if (tracked_pedestrian[j].id == 3){
						ID3_x = tracked_pedestrian[j].x;
						ID3_y = tracked_pedestrian[j].y;
					}
					else if (tracked_pedestrian[j].id == 4){
						ID4_x = tracked_pedestrian[j].x;
						ID4_y = tracked_pedestrian[j].y;
					}
					else if (tracked_pedestrian[j].id > 4){
						ID5_x = tracked_pedestrian[j].x;
						ID5_y = tracked_pedestrian[j].y;
					}
				}
			}
			if (ped_check == false){
				ped_out = fopen("Offline_Pedestrian.txt","a");
				fprintf(ped_out,"%.3f %.3f %.3f %.3f %.3f",0.0,0.0,0.0,0.0,0.0);
				fclose(ped_out);
			}
			if (ped1_check == false){
				ped1_out = fopen("Offline_Pedestrian1.txt","a");
				fprintf(ped1_out,"%.3f %.3f %.3f %.3f %.3f",0.0,0.0,0.0,0.0,0.0);
				fclose(ped1_out);
			}
				ped_out = fopen("Offline_Pedestrian.txt","a");
				fprintf(ped_out,"\n");
				fclose(ped_out);
				ped1_out = fopen("Offline_Pedestrian1.txt","a");
				fprintf(ped1_out,"\n");
				fclose(ped1_out);

			sprintf(ID1_status, "ID1: %.2f", current_ID.used_ID[1]);
		
			sprintf(ID2_status, "ID2: %.2f", current_ID.used_ID[2]);
		
			sprintf(ID3_status, "ID3: %.2f", current_ID.used_ID[3]);
		
			sprintf(ID4_status, "ID4: %.2f", current_ID.used_ID[4]);
		
			sprintf(ID5_status, "ID5: %.2f", current_ID.used_ID[5]);
		
				
	
			sprintf(x_id1,"x: %.2f",ID1_x);
			sprintf(y_id1,"y: %.2f",ID1_y);
			//sprintf(size_id1,"ID1 Size: %.2f",ID1_size);
			sprintf(x_id2,"x: %.2f",ID2_x);
			sprintf(y_id2,"y: %.2f",ID2_y);
			sprintf(x_id3,"x: %.2f",ID3_x);
			sprintf(y_id3,"y: %.2f",ID3_y);
			sprintf(x_id4,"x: %.2f",ID4_x);
			sprintf(y_id4,"y: %.2f",ID4_y);
			sprintf(x_id5,"x: %.2f",ID5_x);
			sprintf(y_id5,"y: %.2f",ID5_y);

			//ddlg->SetDlgItemText(IDC_EDIT2,(LPCSTR)idstr1);
			//ddlg->SetDlgItemText(IDC_EDIT3,(LPCSTR)idstr2);

			ddlg->SetDlgItemText(IDC_EDIT4,(LPCSTR)x_id1);
			ddlg->SetDlgItemText(IDC_EDIT5,(LPCSTR)y_id1);

			ddlg->SetDlgItemText(IDC_EDIT6,(LPCSTR)x_id2);
			ddlg->SetDlgItemText(IDC_EDIT7,(LPCSTR)y_id2);

			ddlg->SetDlgItemText(IDC_EDIT2,(LPCSTR)x_id3);
			ddlg->SetDlgItemText(IDC_EDIT3,(LPCSTR)y_id3);

			ddlg->SetDlgItemText(IDC_EDIT8,(LPCSTR)x_id4);
			ddlg->SetDlgItemText(IDC_EDIT14,(LPCSTR)y_id4);

			ddlg->SetDlgItemText(IDC_EDIT15,(LPCSTR)x_id5);
			ddlg->SetDlgItemText(IDC_EDIT16,(LPCSTR)y_id5);
			//ddlg->SetDlgItemText(IDC_EDIT8,(LPCSTR)size_id1);

			ddlg->SetDlgItemText(IDC_EDIT9,(LPCSTR)ID1_status);
			ddlg->SetDlgItemText(IDC_EDIT10,(LPCSTR)ID2_status);
			ddlg->SetDlgItemText(IDC_EDIT11,(LPCSTR)ID3_status);
			ddlg->SetDlgItemText(IDC_EDIT12,(LPCSTR)ID4_status);
			ddlg->SetDlgItemText(IDC_EDIT13,(LPCSTR)ID5_status);

			datmo.UpdateMap();
		}

		dt = dt+1000;
		//scan_time = timeGetTime()-ctime;
		ctime = timeGetTime()-ctime;
		scan_time = ctime;
		//ddlg->SetDlgItemInt(IDC_EDIT1,scan_time);
		ddlg->SetDlgItemInt(IDC_EDIT1,PRData.gtime);

		/*if(ctime < 500)
			Sleep(500 - ctime);*/
	}

	//delete ddlg;
	//delete idlg;

	//ExitThread(0);

	return 0;

}

DWORD WINAPI NAVI_THREAD(LPVOID lpt){

	
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



DWORD WINAPI NAVI2_THREAD(LPVOID lpt){

	
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


DWORD WINAPI NAVI3_THREAD(LPVOID lpt){

	
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
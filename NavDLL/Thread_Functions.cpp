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
#include "VisableArea.h"
#include "NavDlg.h"
#include "NaviDraw.h"

#include "DATMO.h"
#include "DATMODlg.h"


using namespace RobotTra;
using namespace Geom2D;

Point ggoal;
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

		PRData.SetSensorData(range,rad,rawpose);
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
	FILE * lout;
	Pose ICPPose;
	for (j = 0; j < 6; j++)
		current_ID.used_ID[j] = false;

	//lout = fopen("ICP.txt","w");
	ggoal.x = 0;
	ggoal.y = 0;
	while(IsThread){	   
		
		ctime = timeGetTime();
		tracked_object = false;
		exist = false;
		PRData.GetSensorData(range,rad,Nowrawpose);//Get sensor data from PRData
		PRData.GetSensorData(strange,strad,Nowrawpose);//Get sensor data from PRData

		if(range.size()>0){


            //////////loop/////////////
			//idlg->CallDoICP(range,rad,Nowrawpose);
            //Lastrawpose = Nowrawpose;
#if 1
			for(j=0;j<1;++j){
				ICPPose = idlg->CallComputeICP(strange,strad,Nowrawpose);			
				datmo.DetectMOs(ICPPose,range,rad);//Do it after there are enough scans?
				datmo.getStaticReadings(strange,strad);
			}
#endif
			////////////////////////////
			idlg->CallDoICP(range,rad,Nowrawpose);
            Lastrawpose = Nowrawpose;
			//ICPPose = idlg->getICPpose();
#if 0
			lout = fopen("ICP.txt","a");
			fprintf(lout,"%.3f %.3f %.3f ",ICPPose.p.x,ICPPose.p.y,ICPPose.phi);
			for (j = 0; j < range.size(); j++){
				fprintf(lout,"%.3f ",range[j]);
			}
			fprintf(lout,"\n");
			fclose(lout);
#endif

			ddlg->DrawDATMO(tracked_pedestrian, current_ID);
			if (tracked_pedestrian.size() != 0) {
				for (j = 0;j < tracked_pedestrian.size(); j++){
					if (tracked_pedestrian[j].id == 1){
						//ID1_x = tracked_pedestrian[j].x;
						//ID1_y = tracked_pedestrian[j].y;
						ID1_x = tracked_pedestrian[j].appear_time;
						ID1_y = ICPPose.p.y;
						ID1_size = tracked_pedestrian[j].size;
						ggoal.x = tracked_pedestrian[j].x;
						ggoal.y = tracked_pedestrian[j].y;
						//ggoal.x = tracked_pedestrian[j].Tmean[0];
						//ggoal.y = tracked_pedestrian[j].Tmean[1];
						tracked_object = true;
						//ggoal.x = 0;
						//ggoal.y = 0;
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
				ggoal.x = 0;
				ggoal.y = 0;
			}
			
			datmo.UpdateMap();
			
		}


		ctime = timeGetTime()-ctime;

		ddlg->SetDlgItemInt(IDC_EDIT1,ctime);

		if(ctime < 500)
			Sleep(500 - ctime);
	}

	delete ddlg;
	delete idlg;

	ExitThread(0);

	return 0;

}



/*
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
	FILE * lout;
	Pose ICPPose;
	bool ped_check;
	int a = 0;
	for (j = 0; j < 6; j++)
		current_ID.used_ID[j] = false;

	while(IsThread && PRData.GetLaserDataFromText(lin)){	   

		ctime = timeGetTime();

		/////////////////////////////////////////////////////////
//		PRData.GetSensorData(range,rad,Nowrawpose);//Get sensor data from PRData
		/////////////////////////////////////////////////////////

		PRData.GetSensorData(range,rad,Nowrawpose);//Get sensor data from PRData
		PRData.GetSensorData(strange,strad,Nowrawpose);//Get sensor data from PRData

		if(range.size()>0){
			ped_check = false;
			//////////loop/////////////
			//idlg->CallDoICP(range,rad,Nowrawpose);
            //Lastrawpose = Nowrawpose;
			for(j=0;j<1;++j){
				ICPPose = idlg->CallComputeICP(strange,strad,Nowrawpose);			
				datmo.DetectMOs(ICPPose,range,rad);//Do it after there are enough scans?
				datmo.getStaticReadings(strange,strad);
			}			
			
			idlg->CallDoICP(strange,strad,Nowrawpose);
			Lastrawpose = Nowrawpose;
			
#if 1
			lout = fopen("ICP.txt","a");
			fprintf(lout,"%.3f %.3f %.3f ",ICPPose.p.x,ICPPose.p.y,ICPPose.phi);
			for (j = 0; j < range.size(); j++){
				fprintf(lout,"%.3f ",range[j]);
			}
			fprintf(lout,"\n");
			fclose(lout);
#endif
			ICPPose = idlg->getICPpose();

			datmo.DetectMOs(idlg->getICPpose(),range,rad);//Do it after there are enough scans?

			ddlg->DrawDATMO(tracked_pedestrian, current_ID);


			if (tracked_pedestrian.size() != 0) {
				for (j = 0;j < tracked_pedestrian.size(); j++){
					if (tracked_pedestrian[j].id == 1){
						ID1_x = tracked_pedestrian[j].x;
						ID1_y = tracked_pedestrian[j].y;
						#if 1
						if (ped_check == false){
							lout = fopen("Pedestrian.txt","a");
							fprintf(lout,"%.3f %.3f ",tracked_pedestrian[j].x,tracked_pedestrian[j].y);
							//fprintf(lout,"\n");
							fclose(lout);
							ped_check = true;
						}
						#endif
						//ID1_size = tracked_pedestrian[j].size;
					}
					else if (tracked_pedestrian[j].id == 2){
						ID2_x = tracked_pedestrian[j].x;
						ID2_y = tracked_pedestrian[j].y;
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
				lout = fopen("Pedestrian.txt","a");
				fprintf(lout,"%.3f %.3f ",0,0);
				fclose(lout);
			}
				lout = fopen("Pedestrian.txt","a");
				fprintf(lout,"\n");
				fclose(lout);

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


		ctime = timeGetTime()-ctime;

		ddlg->SetDlgItemInt(IDC_EDIT1,ctime);

		if(ctime < 500)
			Sleep(500 - ctime);
	}

	//delete ddlg;
	//delete idlg;

	//ExitThread(0);

	return 0;

}

*/

#if 0
DWORD WINAPI NAVI_THREAD(LPVOID lpt){

	
	RobotDlg* rdlg = (RobotDlg*)lpt;

	bool &IsThread = rdlg->IsThread3;

	P3robot &probot = rdlg->probot;
	
	PubRobotData& PRData = rdlg->PRData;

	NavDlg* ndlg = rdlg->ndlg;
	MFC_VIEWER &viewer = ndlg->viewer;


    DWAstar dywstar;
	ObsArray Lobs;

	double v;
	double w;
	Pose rpose;
	//Point ggoal;
	Point lgoal;
	//ggoal.x = 40;
	//ggoal.y = -5;
	Transform2D t(rpose);


	ifstream in1("DWAp.txt");
    ifstream in2("obj_weight.txt");
    ifstream in3("DWAstarp.txt");
    double at;
    double sa;
    double sm;
    int ma;
    int hur;

    int Sleeptime=(int)(dywstar.dyw.delta_t*1000.0);
    in3>>sa>>at>>sm>>ma>>hur;
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
	double goal_x = 0;
	double goal_y = 0;
	double theta = 0;
	double pseudo_theta = 0;
	double pseudo_goal_x = 0;
	double pseudo_goal_y = 0;
	double slope = 0;
	double dist;
	FILE * lout;
	//ggoal.x = 2.63;
	//ggoal.y = 1.41;
	while(IsThread){
		//Get Laser Data
		
		if (ggoal.x != 0 && ggoal.y != 0){	
		PRData.GetSensorData(range,rad,rpose);//Get sensor data from PRData

		Lobs.set_obs_laser_array(range,rad);
		
		t.SetBase(rpose);
		lgoal = ggoal;
		t.transform_to_relative(lgoal);
		//slope = lgoal.y / lgoal.x;
		slope = lgoal.x / lgoal.y;
		//pseudo_goal_y = lgoal.y*3;
		//pseudo_goal_x = pseudo_goal_y*slope;
		dist = sqrt(lgoal.x*lgoal.x + lgoal.y*lgoal.y);
		//goal_x = ggoal.x;
		//goal_x = slope;
		//goal_y = ggoal.y;
		//sprintf(x_goal,"x: %.2f",goal_x);
		//sprintf(y_goal,"y: %.2f",goal_y);
		if (dist < 1){
			probot.robot.setVel(0);
		    probot.robot.setRotVel(0);
		}
		else if (abs(slope) < 0.7){
			theta = atan(lgoal.x / lgoal.y)*180/3.1415;
			if (slope > 0){
				probot.robot.setVel(0*1000);
				probot.robot.setRotVel((70 - abs(theta)));
				//probot.robot.setRotVel((3.1415/2)*180.0/3.1415);
			}
			else{
				probot.robot.setVel(0*1000);
				probot.robot.setRotVel(-(70 - abs(theta)));
				//probot.robot.setRotVel((-3.1415/2)*180.0/3.1415);
			}
		}
		else{
		v = probot.getV();
		w = probot.getW();
		//ndlg->SetDlgItemText(IDC_EDIT2,(LPCSTR)x_goal);
		//ndlg->SetDlgItemText(IDC_EDIT3,(LPCSTR)y_goal);
		theta = atan(lgoal.x / lgoal.y)*180/3.1415;
		goal_x = theta;
		sprintf(x_goal,"x: %.2f",goal_x);
		ndlg->SetDlgItemText(IDC_EDIT2,(LPCSTR)x_goal);
#if 1
			lout = fopen("theta.txt","a");
			fprintf(lout,"%.3f ",theta);
			//fprintf(lout,"\n");
			fclose(lout);
#endif
		if (theta >= 35 & theta < 53){
			if (theta < 44){
				pseudo_theta = 35;
				pseudo_goal_y = lgoal.y*3;
				pseudo_goal_x = pseudo_goal_y*0.7;
				DoDWAstar(dywstar, Lobs, v, w, pseudo_goal_x, pseudo_goal_y);
				probot.robot.setVel(v*1000);
				probot.robot.setRotVel(w*180.0/3.1415);
			}
			else{
				pseudo_theta = 53;
				pseudo_goal_y = lgoal.y*3;
				pseudo_goal_x = pseudo_goal_y*1.35;
				DoDWAstar(dywstar, Lobs, v, w, pseudo_goal_x, pseudo_goal_y);
				probot.robot.setVel(v*1000);
				probot.robot.setRotVel(w*180.0/3.1415);
			}
		}
		else if (theta >= 53 & theta < 70){
			if (theta < 61){
				pseudo_theta = 53;
				pseudo_goal_y = lgoal.y*3;
				pseudo_goal_x = pseudo_goal_y*1.35;
				DoDWAstar(dywstar, Lobs, v, w, pseudo_goal_x, pseudo_goal_y);
				probot.robot.setVel(v*1000);
				probot.robot.setRotVel(w*180.0/3.1415);
			}
			else{
				pseudo_theta = 70;
				pseudo_goal_y = lgoal.y*3;
				pseudo_goal_x = pseudo_goal_y*2.75;
				DoDWAstar(dywstar, Lobs, v, w, pseudo_goal_x, pseudo_goal_y);
				probot.robot.setVel(v*1000);
				probot.robot.setRotVel(w*180.0/3.1415);
			}
		}
		else if (theta >= 70 & theta < 90){
			if (theta < 80){
				pseudo_theta = 70;
				pseudo_goal_y = lgoal.y*3;
				pseudo_goal_x = pseudo_goal_y*2.75;
				DoDWAstar(dywstar, Lobs, v, w, pseudo_goal_x, pseudo_goal_y);
				probot.robot.setVel(v*1000);
				probot.robot.setRotVel(w*180.0/3.1415);
			}
			else{
				pseudo_theta = 90;
				pseudo_goal_y = lgoal.y*3;
				pseudo_goal_x = 0.1;
				DoDWAstar(dywstar, Lobs, v, w, pseudo_goal_x, pseudo_goal_y);
				probot.robot.setVel(v*1000);
				probot.robot.setRotVel(w*180.0/3.1415);
			}
		}
		else if (theta <= -35 & theta > -53){
			if (theta > -44){
				pseudo_theta = -35;
				pseudo_goal_y = lgoal.y*3;
				pseudo_goal_x = pseudo_goal_y*-0.7;
				DoDWAstar(dywstar, Lobs, v, w, pseudo_goal_x, pseudo_goal_y);
				probot.robot.setVel(v*1000);
				probot.robot.setRotVel(w*180.0/3.1415);
			}
			else{
				pseudo_theta = -53;
				pseudo_goal_y = lgoal.y*3;
				pseudo_goal_x = pseudo_goal_y*-1.35;
				DoDWAstar(dywstar, Lobs, v, w, pseudo_goal_x, pseudo_goal_y);
				probot.robot.setVel(v*1000);
				probot.robot.setRotVel(w*180.0/3.1415);
			}
		}
		else if (theta <= -53 & theta > -70){
			if (theta > -61){
				pseudo_theta = -53;
				pseudo_goal_y = lgoal.y*3;
				pseudo_goal_x = pseudo_goal_y*-1.35;
				DoDWAstar(dywstar, Lobs, v, w, pseudo_goal_x, pseudo_goal_y);
				probot.robot.setVel(v*1000);
				probot.robot.setRotVel(w*180.0/3.1415);
			}
			else{
				pseudo_theta = -70;
				pseudo_goal_y = lgoal.y*3;
				pseudo_goal_x = pseudo_goal_y*-2.75;
				DoDWAstar(dywstar, Lobs, v, w, pseudo_goal_x, pseudo_goal_y);
				probot.robot.setVel(v*1000);
				probot.robot.setRotVel(w*180.0/3.1415);
			}
		}
		else if (theta <= -70 & theta > -90){
			if (theta > -80){
				pseudo_theta = -70;
				pseudo_goal_y = lgoal.y*3;
				pseudo_goal_x = pseudo_goal_y*-2.75;
				DoDWAstar(dywstar, Lobs, v, w, pseudo_goal_x, pseudo_goal_y);
				probot.robot.setVel(v*1000);
				probot.robot.setRotVel(w*180.0/3.1415);
			}
			else{
				pseudo_theta = -90;
				pseudo_goal_y = lgoal.y*3;
				pseudo_goal_x = -0.1;
				DoDWAstar(dywstar, Lobs, v, w, pseudo_goal_x, pseudo_goal_y);
				probot.robot.setVel(v*1000);
				probot.robot.setRotVel(w*180.0/3.1415);
			}
		}
#if 1
			lout = fopen("pseudo_theta.txt","a");
			fprintf(lout,"%.3f ",pseudo_theta);
			//fprintf(lout,"\n");
			fclose(lout);
#endif
		viewer.FillAll();
		DrawSensorReading( viewer, range, rad, scale);
		DrawDWATreeOnView(viewer,dywstar,0,0,0,ratio);
		ndlg->OnPaint();
		}
		}
		else{
			probot.robot.setVel(0);
		    probot.robot.setRotVel(0);
		}
	
		Sleep(200);
	}


	return 0;
}
#endif



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


////////////////////////////////////////////////
    FILE* lin = fopen("LASER.txt","r");
	PRData.GetLaserDataFromText(lin);
	PRData.GetSensorData(range,rad,Lastrawpose);//Get sensor data from PRData
////////////////////////////////////////////////////////
	Lastrawpose.p.x -= 10.0; //For initialization

	//DATMO
	DATMO &datmo = ddlg->datmo;
	vector<Geom2D::Pedestrian> tracked_pedestrian;
	Occupied_ID current_ID;


	//Computing time
	DWORD ctime;
	DWORD ctime2;
	char textstr[64];

	while(IsThread && PRData.GetLaserDataFromText(lin)){	   

		ctime = timeGetTime();
		ctime2 = ctime;

		/////////////////////////////////////////////////////////
//		PRData.GetSensorData(range,rad,Nowrawpose);//Get sensor data from PRData
		/////////////////////////////////////////////////////////

		PRData.GetSensorData(range,rad,Nowrawpose);//Get sensor data from PRData
		PRData.GetSensorData(strange,strad,Nowrawpose);//Get sensor data from PRData

		if(range.size()>0){
			//////////loop/////////////
			//idlg->CallDoICP(range,rad,Nowrawpose);
            //Lastrawpose = Nowrawpose;
			for(j=0;j<3;++j){
				ICPpose = idlg->CallComputeICP(strange,strad,Nowrawpose);			
				datmo.DetectMOs(ICPpose,range,rad);//Do it after there are enough scans?
				datmo.getStaticReadings(strange,strad);
			}			
			
			idlg->CallDoICP(strange,strad,Nowrawpose);
			Lastrawpose = Nowrawpose;

			

			ICPpose = idlg->getICPpose();

			datmo.DetectMOs(idlg->getICPpose(),range,rad);//Do it after there are enough scans?

			ddlg->DrawDATMO(tracked_pedestrian, current_ID);

			ctime = timeGetTime()-ctime;

			datmo.UpdateMap();
		}


		
		ctime2 = timeGetTime()-ctime2;

		sprintf(textstr,"%dms,%dms",ctime,ctime2);

		ddlg->SetDlgItemText(IDC_EDIT1,(LPCSTR)textstr);
/*
		if(ctime < 500)
			Sleep(500 - ctime);*/
	}

	return 0;

}


#if 1
DWORD WINAPI NAVI_THREAD(LPVOID lpt){

	
	RobotDlg* rdlg = (RobotDlg*)lpt;

	bool &IsThread = rdlg->IsThread3;

	P3robot &probot = rdlg->probot;
	
	PubRobotData& PRData = rdlg->PRData;

	NavDlg* ndlg = rdlg->ndlg;
	MFC_VIEWER &viewer = ndlg->viewer;


    DWAstar dywstar;
	ObsArray Lobs;

	double v;
	double w;
	Pose rpose;
	//Point ggoal;
	Point lgoal;
	lgoal.x = 20;
	lgoal.y = 0;
	//ggoal.x = 40;
	//ggoal.y = -5;
	Transform2D t(rpose);


	ifstream in1("DWAp.txt");
    ifstream in2("obj_weight.txt");
    ifstream in3("DWAstarp.txt");
    double at;
    double sa;
    double sm;
    int ma;
    int hur;

	DWORD ctime = 0;

    int Sleeptime=(int)(dywstar.dyw.delta_t*1000.0);
    in3>>sa>>at>>sm>>ma>>hur;
    dywstar.initDWA(in1,in2);
	dywstar.initDWAstar(at,sm,0,ma,hur);

    int i = -1; // Loop counter for readings

	vector<double>range;
	vector<double>rad;

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

		dywstar.ndptr->ResetReg();
		dywstar.ndptr->Set_All_Intervals(range,rad);
		DoDWAstar(dywstar, Lobs, v, w, lgoal.x, lgoal.y); //Why??

		//DoDWAstar_Tracking(dywstar, Lobs, v, w,TT);
		sprintf(velstr,"(%.2f,%.2f)",v,w);
        ndlg->SetDlgItemText(IDC_EDIT1,(LPCSTR)velstr);

		probot.robot.setVel(v*1000);
	    probot.robot.setRotVel(w*180.0/3.1415);

		viewer.FillAll();
		DrawSensorReading( viewer, range, rad, scale);
		DrawDWATreeOnView(viewer,dywstar,0,0,0,(int)ratio);
		//DrawVAsOnView(viewer, VA, ratio);
		ndlg->OnPaint();

		ctime = timeGetTime() - ctime;

		if(Sleeptime - (int)ctime > 50)
			Sleep(Sleeptime - (int)ctime);

		//Sleep(200);
	}
	

	return 0;
}
#endif
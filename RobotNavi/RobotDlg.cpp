// RobotDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RobotNavi.h"
#include "RobotDlg.h"
#include "Function_Pointers.h"
#include "Thread_Functions.h"

// RobotDlg dialog

IMPLEMENT_DYNAMIC(RobotDlg, CDialog)


RobotDlg::RobotDlg(CWnd* pParent /*=NULL*/)
	: CDialog(RobotDlg::IDD, pParent)
{
	jvmax = 500;
	jwmax = 60;

	robotpt = &probot;
	laserpt = NULL;
	jdlg = NULL;
	idlg = NULL;
	ldlg = NULL;
	mdlg = NULL;
	mapping = true;
}

RobotDlg::~RobotDlg()
{

}

void RobotDlg::EndDlg(){

	EndThread(hThread,IsThread);
	EndThread(hThread2,IsThread2);

	EndThread(hThread3,IsThread3);
	EndThread(hThread4,IsThread4);
	EndThread(hThread5,IsThread5);
    /*
	if(idlg)
		idlg->EndDialog(0);

	if(mdlg)
		mdlg->EndDialog(0);

	if(ndlg)
		ndlg->EndDialog(0);*/

	if(laserpt)
		this->StopLaser(laserpt); //problems

	probot.DisConnect();

	exit(0);

}

void RobotDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

inline void DZT(double &x, double max, double dz){//dz:0~1
	if(fabs(x)<max*dz)
		x = 0;
	else
		if(x>0)
			x = (x-max*dz)/(1-dz);

	else
		if(x<0)
			x = (x+max*dz)/(1-dz);
}

void RobotDlg::JoyKeyDown(int key){

	if(key == 1 && lout!=NULL)
		MeasurementOutput(robotpt,laserpt,lout);

	if(key == 0)
		this->SetDlgItemText(IDC_EDIT1,(LPCSTR)"I love nanoha");
}

void RobotDlg::JoyVel(int x, int y){
	double v;
	double w;

	double deadzone = 0.1;

	ArRobot &robot = probot.robot;	
	char velstate[32];
	char rpose[64];

	v = jvmax * 2 * (double)(32767-y)/65536;
	w = jwmax * 2 * (double)(32767-x)/65536;
	DZT(v,jvmax,deadzone);
	DZT(w,jwmax,deadzone);

	probot.robot.setVel(v);
	probot.robot.setRotVel(w);

	sprintf(velstate,"(%.2f mm/s,%.2f deg/s)",v,w);
	SetDlgItemText(IDC_EDIT1,(LPCTSTR)velstate);

	sprintf(rpose,"(%.2f,%.2f,%.2f,%.2f,%.2f)", probot.getX(), probot.getY(), probot.getTh(),robot.getVel(),robot.getRotVel());
	SetDlgItemText(IDC_EDIT2,(LPCTSTR)rpose);

}


//Laser functions
void RobotDlg::StartLaserThread(){

	EndThread(hThread2,IsThread2);

	IsThread2 = true;
	hThread2=CreateThread(NULL,0, LASER_THREAD,(LPVOID)this, 0,&dwThreadID2);//thread start
}

void RobotDlg::StartSLAMThread(){

	EndThread(hThread4,IsThread4);

	idlg = new ICPDlg();
	idlg->Create(IDD_ICP);
	idlg->ShowWindow(1);

	mdlg = new MapDlg(200,200);
	mdlg->Create(IDD_MAP);
	mdlg->ShowWindow(1);	

	IsThread4 = true;
	hThread4=CreateThread(NULL,0, SLAM_THREAD,(LPVOID)this, 0,&dwThreadID4);//thread start

}

void RobotDlg::StartNaviThread(){

	EndThread(hThread3,IsThread3);
	IsThread3 = true;

	ndlg = new NavDlg();
	ndlg->Create(IDD_NAVDLG);
	ndlg->ShowWindow(1);

	hThread3=CreateThread(NULL,0, NAVI_THREAD,(LPVOID)this, 0,&dwThreadID3);//thread start

}

void RobotDlg::SLAM(Geom2D::Pose &rpose,std::vector<double> &nrange,std::vector<double> &nrad){

	Pose dpose = rpose;
	Transform2D t(this->icpose);
	this->icpose = rpose;
	if(mapping)//Do mapping
		mdlg->UpdateGridMap(rpose,nrange,nrad);
	else{
		//Compute relative movement
		t.transform_to_relative(dpose);
		mdlg->DoMCL(dpose,nrange,nrad);//You should input the "dpose"
	}
}


BEGIN_MESSAGE_MAP(RobotDlg, CDialog)
	ON_WM_PAINT()
	ON_WM_CREATE()
	ON_BN_CLICKED(IDOK, &RobotDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &RobotDlg::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_CONNECT, &RobotDlg::OnBnClickedConnect)
	ON_BN_CLICKED(IDC_STICK, &RobotDlg::OnBnClickedStick)
	ON_BN_CLICKED(IDC_SaveLaser, &RobotDlg::OnBnClickedSavelaser)
	ON_BN_CLICKED(IDC_REODE, &RobotDlg::OnBnClickedReode)
	ON_BN_CLICKED(IDC_MCLSTART, &RobotDlg::OnBnClickedMclstart)
	ON_BN_CLICKED(IDC_Navi, &RobotDlg::OnBnClickedNavi)
	ON_BN_CLICKED(IDC_LASER, &RobotDlg::OnBnClickedLaser)
	ON_BN_CLICKED(IDC_DATMO, &RobotDlg::OnBnClickedDatmo)
	ON_BN_CLICKED(IDC_SLAM, &RobotDlg::OnBnClickedSlam)
	ON_BN_CLICKED(IDC_BUTTON1, &RobotDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_Navi2, &RobotDlg::OnBnClickedNavi2)
	ON_BN_CLICKED(IDC_Navi3, &RobotDlg::OnBnClickedNavi3)
END_MESSAGE_MAP()


// RobotDlg message handlers

void RobotDlg::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here
	EndDlg();

	
}

void RobotDlg::OnBnClickedCancel()
{
	// TODO: Add your control notification handler code here
	EndDlg();
	OnCancel();
}

void RobotDlg::OnBnClickedConnect()
{
	// TODO: Add your control notification handler code here
	if(!probot.Connected()){//to be connected
		probot.Connect();
		GetDlgItem(IDC_CONNECT)->SetWindowText((LPCTSTR)"Stop");
	}
	else{
		probot.DisConnect();
		GetDlgItem(IDC_CONNECT)->SetWindowText((LPCTSTR)"Connect");
	}

}


void RobotDlg::OnBnClickedStick()
{
	// TODO: Add your control notification handler code here
	DWORD ExitCode;
    GetExitCodeThread(hThread,&ExitCode);
 
	if(ExitCode == STILL_ACTIVE){
		EndThread(hThread,IsThread);
        GetDlgItem(IDC_STICK)->SetWindowText((LPCTSTR)"Stick Control");
	}
	else{

		if(!jdlg){
			jdlg = new JoyDlg();
			jdlg->Create(IDD_JOYDLG);
			jdlg->ShowWindow(1);
		}
	
		GetDlgItem(IDC_STICK)->SetWindowText((LPCTSTR)"Stop");
	    IsThread = true;
	    hThread=CreateThread(NULL,0, JOYCTRL_THREAD,(LPVOID)this, 0,&dwThreadID);//thread start
	}


}


void RobotDlg::OnBnClickedSavelaser()
{
	// TODO: Add your control notification handler code here
	probot.SimLaserOutput(lout);
}


void RobotDlg::OnBnClickedReode()
{
	// TODO: Add your control notification handler code here
	probot.SetOdemetry(0,0,0);
}

void RobotDlg::OnBnClickedMclstart()
{
	// TODO: Add your control notification handler code here
	Pose mean = idlg->getICPpose();
	Pose sigma;
	sigma.p.x = 1;
	sigma.p.y = 1;
	sigma.phi = PI/10;

	this->icpose = mean;

	this->mapping = false;
	mdlg->MCLInit(mean,sigma,500);
}

void RobotDlg::OnBnClickedNavi()
{
	// TODO: Add your control notification handler code here
	this->StartNaviThread();
}

void RobotDlg::OnBnClickedLaser()
{
	int sensor;
	// TODO: Add your control notification handler code here
	if(IsDlgButtonChecked(IDC_RADIOS1))//Sim Laser
		sensor = 1;
	else
	if(IsDlgButtonChecked(IDC_RADIOS2))//LMS100 Laser
		sensor = 2;

	
	DWORD ExitCode;
    GetExitCodeThread(hThread2,&ExitCode);
 
	if(ExitCode == STILL_ACTIVE){
		IsThread2 = false;
		fclose(lout);
        GetDlgItem(IDC_LASER)->SetWindowText((LPCTSTR)"Laser Start");

		return;
	}
	

	if(sensor == 1){
		this->laserpt = &(this->probot.sick);//pointer assignment
		this->StartLaser = StartSimLaser;
		this->getLaser = getSimLaser;
		this->StopLaser = StopSimLaser;
		this->MeasurementOutput = SaveSimLaser;
		lout = fopen("SimLASER.txt","w");
	}
	
	
	if(sensor == 2){
		if(!ldlg)
			ldlg = new LaserDlg();
		
		this->laserpt = ldlg;//pointer assignment
		this->StartLaser = StartLMS100Laser;
		this->getLaser = getLMS100Laser;
		this->StopLaser = StopLMS100Laser;
		this->MeasurementOutput = SaveLMS100Laser;
		lout = fopen("LMS100LASER.txt","w");


	}
	
	IsThread2 = true;		
	GetDlgItem(IDC_LASER)->SetWindowText((LPCTSTR)"Stop");


	this->StartLaserThread();

}


void RobotDlg::OnBnClickedDatmo()
{
}

void RobotDlg::OnBnClickedSlam()
{

	// TODO: Add your control notification handler code here
	this->StartSLAMThread();
}

void RobotDlg::OnBnClickedButton1()
{
	// TODO: Add your control notification handler code here
	// TODO: Add your control notification handler code here

	idlg = new ICPDlg();
	idlg->Create(IDD_ICP);
	idlg->ShowWindow(1);


	//EndThread(hThread5,IsThread5);
	//IsThread5 = true;
	//hThread5=CreateThread(NULL,0, DATMO_THREAD_OFFLINE,(LPVOID)this, 0,&dwThreadID5);//thread start

}

void RobotDlg::OnBnClickedNavi2()
{
	// TODO: Add your control notification handler code here

	EndThread(hThread3,IsThread3);
	IsThread3 = true;

	ndlg = new NavDlg();
	ndlg->Create(IDD_NAVDLG);
	ndlg->ShowWindow(1);

	hThread3=CreateThread(NULL,0, NAVI2_THREAD,(LPVOID)this, 0,&dwThreadID3);//thread start
}

void RobotDlg::OnBnClickedNavi3()
{
	// TODO: Add your control notification handler code here
	EndThread(hThread3,IsThread3);
	IsThread3 = true;

	ndlg = new NavDlg();
	ndlg->Create(IDD_NAVDLG);
	ndlg->ShowWindow(1);

	hThread3=CreateThread(NULL,0, NAVI3_THREAD,(LPVOID)this, 0,&dwThreadID3);//thread start
}

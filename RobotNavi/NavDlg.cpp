// NavDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RobotNavi.h"
#include "NavDlg.h"
#include "IO_Functions.h"
#include <mmsystem.h>
#include <fstream>
#include "NaviDraw.h"



using namespace std;
using namespace Geom2D;
using namespace RobotTra;

// NavDlg dialog

IMPLEMENT_DYNAMIC(NavDlg, CDialog)

NavDlg::NavDlg(CWnd* pParent /*=NULL*/)
	: CDialog(NavDlg::IDD, pParent)
{
	double vlim[] = {0.5,0,1,-1,0.5,1,0.05,0.1,0.5};
	VelItv* vitv = new VelItv(1,80);
	this->dwa = new DWA(vlim,*vitv);

	lin = NULL;


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


	double* alpha = DWAtree.alpha;
	ifstream in4("alpha.txt");
	in4>>alpha[0]>>alpha[1]>>alpha[2];

}

NavDlg::~NavDlg()
{
	this->hmp.~HMOMDP();
	this->DWAtree.~TraTree();
}

void NavDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}



void DrawItv(MFC_VIEWER& viewer, VelItv& vitv, double scale){

	int s = vitv.getsize();
	double radius;

	for(int i=0;i<s;i+=1){
		viewer.DrawArcT(0,0,(int)(vitv.IdxtoRad(i)/scale),(int)(vitv.getdist(i)/scale),1,0,0,255);
	}
}

void DrawCandidateVel(MFC_VIEWER& viewer, DWA& dwa, double scale){
	vector<VEL>& cand = dwa.candidates;
	int s = cand.size();
	double radius;
	double dist;

	for(int i=0;i<s;i+=1){

		radius = dwa.vitv->IdxtoRad(cand[i].i);
		//dist = cand[i].v;
		dist = dwa.vitv->getdist(cand[i].i);

		viewer.DrawArcT(0,0,(int)(radius/scale),(int)(dist/scale),1,0,0,255);
	}

}

void DrawBestPath(MFC_VIEWER& viewer, TraTree& DWAtree, double scale){

	vector<VELnode>& bestpath = DWAtree.GetBestPath();
	vector<VELnode>& allnodes = DWAtree.GetTree();
    int s = allnodes.size();
    int i=0;
	VELnode n;
	VELnode p;
	for(i=0;i<s;++i){
		n = allnodes[i];
		p = allnodes[n.parent];
		viewer.DrawLineT((int)(n.x/scale),(int)(n.y/scale),(int)(p.x/scale),(int)(p.y/scale),1,0,0,255);
	}


	s = bestpath.size()-1;

	for(i=0;i<s;++i){
		viewer.DrawLineT((int)(bestpath[i].x/scale),(int)(bestpath[i].y/scale),(int)(bestpath[i+1].x/scale),(int)(bestpath[i+1].y/scale),2,255,0,0);
	}
        viewer.DrawLineT((int)(bestpath[i].x/scale),(int)(bestpath[i].y/scale),0,0,2,255,0,0);
}

void DrawTargetPath(MFC_VIEWER& view, vector<GoalPoint>& TT, double scale){

    double px;//parent node
	double py;
	double nx;//now node
	double ny;
	GoalPoint ng;
	GoalPoint pg;
	double ratio = 1/scale;

	double rx=0;
	double ry=0;
	double rth=0;
    
	ng = TT[0];
	nx = (double)ratio*(cos(rth)*ng.x - sin(rth)*ng.y + rx);
	ny = (double)ratio*(sin(rth)*ng.x + cos(rth)*ng.y + ry);
	view.DrawCircle(view.Width()/2-(int)(ny),view.Height()/2-(int)(nx),10,2,0,255,0);

	for(int i=0;i<TT.size()-1;++i){
		ng = TT[i+1];
		pg = TT[i];
		nx = (double)ratio*(cos(rth)*ng.x - sin(rth)*ng.y + rx);
		ny = (double)ratio*(sin(rth)*ng.x + cos(rth)*ng.y + ry);
	    px = (double)ratio*(cos(rth)*pg.x - sin(rth)*pg.y + rx);
		py = (double)ratio*(sin(rth)*pg.x + cos(rth)*pg.y + ry);
		view.DrawLine(view.Width()/2-(int)(ny),view.Height()/2-(int)(nx),view.Width()/2-(int)(py),view.Height()/2-(int)(px),2,0,255,0);

	}



}


void NavDlg::Planning(double lgx,double lgy){

	// TODO: Add your control notification handler code here

	DWORD ctime = timeGetTime();

	char str[32];

	double scale = 0.03;

	double v;
	double w;

	//Get data
	PRData->GetSensorData(range,rad,rawpose);
	PRData->GetPoseData(rawpose,v,w);
	obsarr.set_obs_laser_array(range,rad);

	DWAtree.dwa = this->dwa;

	//v = 0.2;
	//w = 0;
	//Computing
	DWAtree.TraSearch(obsarr.obs, v, w, lgx, lgy, 300);

	ctime = timeGetTime() - ctime;

	sprintf(str,"(%.2f,%.2f),%d nodes,%dms",v,w,DWAtree.GetTree().size(),ctime);

	SetDlgItemText(IDC_EDIT1,(LPCSTR)str);

	vel_v = v;
	vel_w = w;

	
	//Drawing
	viewer.FillAll();//Cleaning
    DrawSensorReading( viewer, range, rad, scale);
    DrawBestPath(viewer,  DWAtree, scale);
	this->OnPaint();


}


BEGIN_MESSAGE_MAP(NavDlg, CDialog)
	ON_WM_PAINT()
	ON_WM_CREATE()
	ON_BN_CLICKED(IDC_Compute, &NavDlg::OnBnClickedCompute)
	ON_BN_CLICKED(IDC_Expand, &NavDlg::OnBnClickedExpand)
	ON_BN_CLICKED(IDC_OFFLINE, &NavDlg::OnBnClickedOffline)
	ON_BN_CLICKED(IDC_SaveImg, &NavDlg::OnBnClickedSaveimg)
	ON_BN_CLICKED(IDC_OFFLINE2, &NavDlg::OnBnClickedOffline2)
	ON_BN_CLICKED(IDC_POMDP, &NavDlg::OnBnClickedPomdp)
	ON_BN_CLICKED(IDOK, &NavDlg::OnBnClickedOk)
END_MESSAGE_MAP()


// NavDlg message handlers

void NavDlg::OnBnClickedCompute()
{
	// TODO: Add your control notification handler code here
	double scale = 0.02;

	double v;
	double w;

	//Get data
	PRData->GetSensorData(range,rad,rawpose);
	PRData->GetPoseData(rawpose,v,w);
	obsarr.set_obs_laser_array(range,rad);

	//Computing
	dwa->Update_Vel_State(obsarr.obs,v,w);

	//Drawing
	viewer.FillAll();//Cleaning
    DrawSensorReading( viewer, range, rad, scale);
    DrawCandidateVel( viewer,  *dwa,  scale);
	this->OnPaint();

}


void NavDlg::OnBnClickedExpand()
{
	// TODO: Add your control notification handler code here

	DWORD ctime = timeGetTime();

	char str[32];

	double scale = 0.03;

	double v;
	double w;

	//Get data
	PRData->GetSensorData(range,rad,rawpose);
	PRData->GetPoseData(rawpose,v,w);
	obsarr.set_obs_laser_array(range,rad);

	DWAtree.dwa = this->dwa;

	//v = 0.2;
	//w = 0;
	//Computing
	DWAtree.TraSearch(obsarr.obs, v, w, 20, 0, 400);

	ctime = timeGetTime() - ctime;

	sprintf(str,"(%.2f,%.2f),%d nodes,%dms",v,w,DWAtree.GetTree().size(),ctime);

	SetDlgItemText(IDC_EDIT1,(LPCSTR)str);

	vel_v = v;
	vel_w = w;

	
	//Drawing
	viewer.FillAll();//Cleaning
    DrawSensorReading( viewer, range, rad, scale);
    DrawBestPath(viewer,  DWAtree, scale);
	this->OnPaint();

}


/* For TraTree
void NavDlg::OnBnClickedOffline()
{
	// TODO: Add your control notification handler code here

	Pose nowpose;

	DWORD ctime = timeGetTime();

	char str[32];

	double scale = 0.03;

	double v = 0;
	double w = 0;


	if(!lin)
		lin = fopen("LASER.txt","r");

	if(!ReadRawLaserData(lin, nowpose.p.x, nowpose.p.y, nowpose.phi,rad, range))
		return;

	obsarr.set_obs_laser_array(range,rad);

// Compute
	//v = 0.2;
	//w = 0;
	//Computing

	DWAtree.dwa = this->dwa;

	DWAtree.TraSearch(obsarr.obs, v, w, 20, 0, 100);

	ctime = timeGetTime() - ctime;

	sprintf(str,"(%.2f,%.2f),%d nodes,%dms",v,w,DWAtree.GetTree().size(),ctime);

	SetDlgItemText(IDC_EDIT1,(LPCSTR)str);

//

	//Drawing
	viewer.FillAll();//Cleaning
    DrawSensorReading( viewer, range, rad, scale);
    DrawBestPath(viewer,  DWAtree, scale);
	this->OnPaint();


}
*/


void NavDlg::OnBnClickedOffline()
{
	// TODO: Add your control notification handler code here

	Pose nowpose;

	DWORD ctime = 0;

	char str[64];

	double scale = 0.05;

	double v = 0;
	double w = 0;

	////////////DWA tracking
	vector<GoalPoint> TT;
    GoalPoint ngp;

	ifstream in("inTra.txt");
	ifstream ing("ingoal.txt");

	TT.clear();

	while(in>>ngp.x>>ngp.y>>ngp.t){
		TT.push_back(ngp);
	}
	////////////DWA tracking

	if(!lin){
		lin = fopen("LASER.txt","r");
		counter = 0;
	}

	if(!ReadRawLaserData(lin, nowpose.p.x, nowpose.p.y, nowpose.phi,rad, range))
		return;
	++counter;

    ctime = timeGetTime();

	obsarr.set_obs_laser_array(range,rad);

// Compute
		dywstar.ndptr->ResetReg();
	    dywstar.ndptr->Set_All_Intervals(range,rad);
		DoDWAstar_Tracking(dywstar, obsarr, v, w,TT);
		ctime = timeGetTime() - ctime;

		sprintf(str,"(%.2f,%.2f) %dms data%d, %d door, %d gaps",v,w,ctime,counter,(dywstar.ndptr)->regions.size(),(dywstar.ndptr)->gaps.size());


        SetDlgItemText(IDC_EDIT1,(LPCSTR)str);

//

	//Drawing
	viewer.FillAll();//Cleaning
	DrawSensorReading( viewer, range, rad, scale);
	DrawDWATreeOnView(viewer,dywstar,0,0,0,(int)(1/scale));
	//DrawNDOnView(viewer,*(dywstar.ndptr),VFHsize,(int)(1/scale));
	//DrawVAsOnView(viewer, dywstar.VAs[dywstar.VAs.size()-1], (int)(1/scale));
	this->OnPaint();


}
void NavDlg::OnBnClickedSaveimg()
{
	// TODO: Add your control notification handler code here
		// TODO: Add your control notification handler code here
	char strFilter[] = { "BMP Files (*.bmp)|*.bmp|JPG Files (*.jpg)|*.jpg|All Files (*.*)|*.*||" };

	CFileDialog FileDlg(FALSE, ".bmp", NULL, 0, strFilter);

	if(FileDlg.DoModal()==IDOK)
		viewer.SavePicture(FileDlg.GetPathName());
}

void NavDlg::OnBnClickedOffline2()
{
	// TODO: Add your control notification handler code here
	// TODO: Add your control notification handler code here

	Pose nowpose;

	DWORD ctime = 0;

	char str[64];

	double scale = 0.025;

	double v = 0;
	double w = 0;

	////////////DWA tracking
	vector<GoalPoint> TT;
    GoalPoint ngp;

	ifstream in("inTra.txt");
	ifstream ing("ingoal.txt");

	TT.clear();

	while(in>>ngp.x>>ngp.y>>ngp.t){
		TT.push_back(ngp);
	}
	////////////DWA tracking

	if(!lin){
		lin = fopen("LASER.txt","r");
		counter = 0;
	}

	if(!ReadRawLaserData(lin, nowpose.p.x, nowpose.p.y, nowpose.phi,rad, range))
		return;
	++counter;

    ctime = timeGetTime();

	obsarr.set_obs_laser_array(range,rad);

// Compute
	/*
		dywstar.ndptr->ResetReg();
	    dywstar.ndptr->Set_All_Intervals(range,rad);
		DoDWAstar_Tracking(dywstar, obsarr, v, w,TT);
		*/

	   DWAtree.ndptr = &(this->nd);

		DWAtree.dwa = this->dwa;

		nd.ResetAll();
		nd.Set_All_Intervals(range,rad);
		nd.Find_Gap(0.8);
		nd.Find_REGION();
		nd.Find_Door(0.49);

		//DWAtree.TraSearch(obsarr.obs, v, w, 20, 0, 100);
        //DoNavi(DWAtree, obsarr, v, w, 20,0);//initobs: (0,0,0)
		DoTracking(DWAtree, obsarr, v, w,  TT);
		ctime = timeGetTime() - ctime;

		//sprintf(str,"(%.2f,%.2f) %dms data%d, %d door, %d gaps",v,w,ctime,counter,(dywstar.ndptr)->regions.size(),(dywstar.ndptr)->gaps.size());
		sprintf(str,"(%.2f,%.2f) %dms data%d, %d door, %d gaps",v,w,ctime,counter,nd.regions.size(), nd.gaps.size());

        SetDlgItemText(IDC_EDIT1,(LPCSTR)str);

//

	//Drawing
	viewer.FillAll();//Cleaning
	DrawSensorReading( viewer, range, rad, scale);
	DrawBestPath( viewer,  DWAtree, scale);
	DrawTargetPath(viewer, TT, scale);
	//DrawNDOnView(viewer,nd,VFHsize,(int)(1/scale));
	DrawVAsOnView(viewer, DWAtree.VAs[DWAtree.VAs.size()-1], (int)(1/scale));
	this->OnPaint();
}

void NavDlg::OnBnClickedPomdp()
{
	// TODO: Add your control notification handler code here
	CString str;
	char solution[64];
	double Uv;
	double Lv;
	this->GetDlgItemText(IDC_EDIT1,str);
	double p = atof((LPCSTR)str);
	planning_TS(p,hmp);
	hmp.BELTreeOutput();

	sprintf(solution,"%.2f, %.2f, action:%d",hmp.Uv,hmp.Lv,*(int*)(hmp.best_action));
	this->SetDlgItemText(IDC_EDIT2,solution);
}

void NavDlg::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here
	OnOK();
}

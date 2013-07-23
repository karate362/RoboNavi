// LaserDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RobotNavi.h"
#include "LaserDlg.h"
#include "math.h"
#include<time.h>
#include <mmsystem.h>
#pragma   comment(lib,"winmm.lib") 
// LaserDlg dialog

IMPLEMENT_DYNAMIC(LaserDlg, CDialog)


DWORD WINAPI READ_THREAD (LPVOID lpt){

	LaserDlg* ldlg = (LaserDlg*) lpt;
    int i = 0;
    double scale = 2.5; //(cm/pixel)
	double ang = 0;
    double range = 0;
	int x;
	int y;
   
    DWORD ctime;

	int counter = 0;

	while(ldlg->IsReadCont){


       counter = (counter+1)%5;
		
	   ctime = timeGetTime();

		//ldlg->OnBnClickedSingle();
		ldlg->getLaser();
		ctime = timeGetTime() - ctime;
		ldlg->SetDlgItemInt(IDC_EDIT1,ctime);

		if(	ldlg->IsDlgButtonChecked(IDC_RECORD) && counter == 0)//
			ldlg->OnBnClickedSavelaser();


		Sleep(100);
	}

	return 0;

}


LaserDlg::LaserDlg(CWnd* pParent /*=NULL*/)
	: CDialog(LaserDlg::IDD, pParent)
{
	pLaser = NULL;
	IsReadCont = false;
	lout = fopen("LASER.txt","w");
}

LaserDlg::~LaserDlg()
{
  StopLaser();
}

void LaserDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(LaserDlg, CDialog)
	ON_WM_PAINT()
	ON_WM_CREATE()
	ON_BN_CLICKED(IDC_SINGLE, &LaserDlg::OnBnClickedSingle)
	ON_BN_CLICKED(IDC_CONNNECT, &LaserDlg::OnBnClickedConnnect)
	ON_BN_CLICKED(IDCLOSE, &LaserDlg::OnBnClickedClose)
	ON_BN_CLICKED(IDC_CONT, &LaserDlg::OnBnClickedCont)
	ON_BN_CLICKED(IDC_SaveLaser, &LaserDlg::OnBnClickedSavelaser)
END_MESSAGE_MAP()


// LaserDlg message handlers


bool LaserDlg::StartLaser(){

	//Create Mutex
	lmutex = CreateMutex(NULL,false,(LPCSTR)"LaserSync");  
	
	if(!(pLaser))
		pLaser =  new SickLms100Laser();

	return pLaser->Open(DEVICE_ADDR);

}

void LaserDlg::getLaser(vector<double>& range, vector<double>& rad){

	range.clear();
	rad.clear();
	this->LRange.clear();
	this->LRad.clear();

	double dist;
	double ang;

	int idx1 = 90;
	int idx2 = DATA_LENGTH - 90;

	//Get mutex;
	WaitForSingleObject(lmutex,INFINITE);

	if(ReadLaser()){//only use 0~180
		for (int i = idx1; i < idx2; i++) {
			dist = (double)aScan[i]/1000;
			ang = (double)(i-270)*0.5*3.1415926/180;

			if(dist<0.05)
				dist = 3000;

			range.push_back(dist);//m
			rad.push_back(ang); 
			this->LRange.push_back(dist);
            this->LRad.push_back(ang);
			

		}
	}

	//release mutex
    ReleaseMutex(lmutex);

}

void LaserDlg::getLaser(){

	this->LRange.clear();
	this->LRad.clear();

	double dist;
	double ang;

	int idx1 = 90;
	int idx2 = DATA_LENGTH - 90;

	//Get mutex;
	WaitForSingleObject(lmutex,INFINITE);

	if(ReadLaser()){
		for (int i = idx1; i < idx2; i++) {
			dist = (double)aScan[i]/1000;
			ang = (double)(i-270)*0.5*3.1415926/180;
			this->LRange.push_back(dist);
            this->LRad.push_back(ang);
		}
	}
	//release mutex
    ReleaseMutex(lmutex);

}

void LaserDlg::StopLaser(){
	if(pLaser){
	  pLaser->Close();
	  delete pLaser;	
	  pLaser = NULL;
	}

	CloseHandle(lmutex);
}



bool LaserDlg::ReadLaser(){

		if(!pLaser){//pLaser is a NULL pointer
            OnConnectErr();
		    return false;
		}

		if(!(pLaser->Scan(aScan))){
            OnReadErr();
			return false;
		}

		return true;

}

void LaserDlg::getRange(vector<double>& dist){
	//Get mutex;
	WaitForSingleObject(lmutex,INFINITE);

	dist.resize(LRange.size());
	copy(LRange.begin(),LRange.end(),dist.begin());

    //release mutex
    ReleaseMutex(lmutex);

}
void LaserDlg::getRad(vector<double>& dist){
	//Get mutex;
	WaitForSingleObject(lmutex,INFINITE);

	dist.resize(LRad.size());
	copy(LRad.begin(),LRad.end(),dist.begin());

    //release mutex
    ReleaseMutex(lmutex);

}

void LaserDlg::SaveLaser(FILE* out){

	//Get mutex;
	WaitForSingleObject(lmutex,INFINITE);

	int s = LRange.size();

	for(int i=0;i<s;++i){
		fprintf(out,"%f ",LRange[i]);
	}

    //release mutex
    ReleaseMutex(lmutex);

    fprintf(out,"\n");

}


int LaserDlg::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
   CClientDC dc(this);
   lview.init(dc,320,240);
   lview.FillAll(0,0,0);

   return 0;

}

void LaserDlg::OnPaint()
{

   CClientDC dc(this); 

   lview.PasteOn(dc,0,0);

   CDialog::OnPaint();

}



void LaserDlg::OnBnClickedSingle()
{   
        int i = 0;
        double scale = 2.5; //(cm/pixel)
		double ang = 0;
		double range = 0;
		int x;
		int y;

        //w in bmp = wo - y, h in bmp = ho - x;

		LRange.clear();
		LRad.clear();

        lview.FillAll(255,255,255);

		lview.DrawCircleT(0,0,(int)(30/scale),1,0,0,0);

		if(ReadLaser()){
		    for (int i = 0; i < DATA_LENGTH; i++) {
			  range = (double)aScan[i]/10;//cm
			  ang = (double)(i-270)*0.5*3.1415926/180;

			  LRange.push_back(range);
			  LRad.push_back(ang);

			  x = (int)(range*cos(ang)/scale);
			  y = (int)(range*sin(ang)/scale);
			  lview.DrawCircleT(x,y,1,2,255,0,0);
		    }
		}

		OnPaint();

}
void LaserDlg::OnBnClickedConnnect()
{
	if(!(pLaser))
		pLaser =  new SickLms100Laser();

	if (!(pLaser->Open(DEVICE_ADDR)))
		OnConnectErr();
	else
		OnSuccess();
}

void LaserDlg::OnBnClickedClose()
{
	// TODO: Add your control notification handler code here
	StopLaser();

}



void LaserDlg::OnBnClickedCont()
{
	// TODO: Add your control notification handler code here
	if(IsReadCont){
		IsReadCont = false;
		GetDlgItem(IDC_CONT)->SetWindowText((LPCTSTR)"Cont");
	}
	else{
		IsReadCont = true;
		GetDlgItem(IDC_CONT)->SetWindowText((LPCTSTR)"STOP");
		hReadThread = CreateThread(NULL,0, READ_THREAD,(LPVOID)this, 0,&dwReadThreadID);
	}

}


void LaserDlg::OnBnClickedSavelaser()
{
	// TODO: Add your control notification handler code here
	this->SaveLaser(lout);
}

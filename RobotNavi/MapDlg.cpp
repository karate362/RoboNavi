// MapDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RobotNavi.h"
#include "MapDlg.h"
#include "IO_Functions.h"
#include "mmsystem.h"
#include <fstream>

using namespace Geom2D;
using namespace std;
// MapDlg dialog

IMPLEMENT_DYNAMIC(MapDlg, CDialog)

MapDlg::MapDlg(int m_width, int m_height,CWnd* pParent /*=NULL*/)
	: CDialog(MapDlg::IDD, pParent)
{
	width = m_width;
	height = m_height;
	buffersize = 4*width*height;
	buffer = new BYTE[buffersize];

	fin = NULL;

}

MapDlg::~MapDlg()
{
	this->Release();
}

void MapDlg::Release(){

	delete[] buffer;

	if(mcl)
		delete mcl;

	for(int i=0;i<localmaps.size();++i)
		delete localmaps[i];


}

void MapDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


void MapDlg::DrawParticles(){

    CClientDC dc(this); 

	Pose np;
	int s = mcl->size();

	int w;
	int h;


	viewer.CopyFrom(buffer,buffersize);//OK!  Notice that local variable should not be too big!!

	for(int i=0;i<s;++i){
		np = mcl->getParticle(i);
		gmap->XYToGrid(np.p.x,np.p.y,w,h);
		viewer.DrawCircle(w,h,1,1,255,0,0);
	}

	
}


void MapDlg::DrawBestParticle(Geom2D::Pose rpose, const std::vector<double>& prange, const std::vector<double>& prad){

	int i=0;
	int s=prange.size();

	int zw,zh,rw,rh;

    gmap->XYToGrid(rpose.p.x,rpose.p.y,rw,rh);

	for(i=0;i<s;i+=2){
		gmap->XYToGrid(rpose.p.x + prange[i]*cos(rpose.phi+prad[i]),rpose.p.y + prange[i]*sin(rpose.phi+prad[i]),zw,zh);
		viewer.DrawLine(zw,zh,rw,rh,1,0,0,255);
	}

	viewer.DrawCircle(rw,rh,5,2,0,0,255);
}

void MapDlg::DrawLocalGoal(){

	int gw,gh;

	gmap->XYToGrid(pgoal.x,pgoal.y,gw,gh);

	viewer.DrawCircle(gw,gh,10,3,0,255,0);
}


void MapDlg::UpdateGridMap(const Geom2D::Pose& rpose, const std::vector<double>& nrange, const std::vector<double>& nrad){
	
	//Whether it is needed to create another map;
	Geom2D::Pose mpose;

	if(localmaps.empty())
		this->CreateNewGridMap(rpose.p.x,rpose.p.y,rpose.phi);

	mpose = gmap->getPose();
	if( dist(rpose.p,mpose.p) > 4)
		this->CreateNewGridMap(rpose.p.x,rpose.p.y,rpose.phi);
	
	gmap->Grid_Mapping(rpose,nrange,nrad);
	
	this->DrawfromGMAP(gmap->getGMAP(),gmap->size());
	viewer.CopyFrom(buffer,buffersize);//OK!  Notice that local variable should not be too big!!
	OnPaint();
	

}

void MapDlg::CreateNewGridMap(double x,double y,double ang){
	double offset = 3.9;
	Point lgoal;

	//Decide local goal
	lgoal.x = x; 
	lgoal.y = y;

	gmap = new GridMap(x+offset*cos(ang),y+offset*sin(ang),ang);
	localmaps.push_back(gmap);
	localgoals.push_back(lgoal);

}


void MapDlg::MCLInit(Geom2D::Pose mean,Geom2D::Pose sigma,int pnum){

	mcl = new MCL(pnum,gmap);

	mcl->SetInitGuess(mean,sigma,pnum);

	mcl->SetGridMapSet(localmaps);

	mclpose = mean;//

	DrawParticles();
	this->OnPaint();
}


void MapDlg::DoMCL(Geom2D::Pose dpose, const std::vector<double>& nrange, const std::vector<double>& nrad){
//Decide the map to localize

	double md = 0;
	double dmin = dist( (localmaps[0]->getPose()).p,this->mclpose.p);//distance between mappose and robot pose

	Pose fpose = dpose;
	Transform2D t(this->mclpose);//front 3m...?
	t.transform_to_global(fpose);
    
	for(int i=0;i<localmaps.size();++i){
		md = dist((localmaps[i]->getPose()).p,fpose.p);
		if(md<dmin)
			dmin = md,gmap = localmaps[i],pgoal = localgoals[i];
	}
	mcl->SetGridMap(gmap);


	mcl->Predict(dpose);
	mcl->Weighting(nrange,nrad);
	mcl->Resample();

	this->mclpose = mcl->getMaxiPartcle();

	DrawfromGMAP(gmap->getGMAP(),gmap->size());
	viewer.CopyFrom(buffer,buffersize);//OK!  Notice that local variable should not be too big!!
	DrawParticles();
	DrawBestParticle(mclpose,nrange,nrad);
	DrawLocalGoal();
	this->OnPaint();
}



Geom2D::Point MapDlg::getLocalGoalT(){
	Point tgoal = pgoal;
	Transform2D t(mclpose);
	t.transform_to_relative(tgoal);

	double goaldis = sqrt( sqr(tgoal.x) + sqr(tgoal.y) );

	if(goaldis > 1)
		return tgoal;
	else{
		tgoal = pgoal;
		tgoal.x += (tgoal.x - gmap->getPose().p.x);
		tgoal.y += (tgoal.y - gmap->getPose().p.y);
		t.transform_to_relative(tgoal);
		return tgoal;
	}

}


BEGIN_MESSAGE_MAP(MapDlg, CDialog)
	ON_BN_CLICKED(IDC_DRAWMAP, &MapDlg::OnBnClickedDrawmap)
	ON_WM_PAINT()
	ON_WM_CREATE()
	ON_BN_CLICKED(IDC_BUTTON1, &MapDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_LMAP, &MapDlg::OnBnClickedLmap)
	ON_BN_CLICKED(IDC_SaveGMAP, &MapDlg::OnBnClickedSavegmap)
	ON_BN_CLICKED(IDC_LoadGMAP, &MapDlg::OnBnClickedLoadgmap)
	ON_BN_CLICKED(IDC_MCLINIT, &MapDlg::OnBnClickedMclinit)
	ON_BN_CLICKED(IDC_DOMCL, &MapDlg::OnBnClickedDomcl)
	ON_BN_CLICKED(IDC_SaveImg, &MapDlg::OnBnClickedSaveimg)
	ON_BN_CLICKED(IDOK, &MapDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &MapDlg::OnBnClickedCancel)
END_MESSAGE_MAP()


// MapDlg message handlers

void MapDlg::OnBnClickedDrawmap()
{
	// TODO: Add your control notification handler code here

	Pose rpose;

	DWORD ctime;
	char str[16];

	if(!fin)
		fin = fopen("ICPLASER.txt","r");

	if(!ReadICPLaserData(fin, rpose.p.x, rpose.p.y, rpose.phi,rad, range)){
		this->SetDlgItemText(IDC_EDIT1,(LPCTSTR)"No data");
	    return ;
	}
	else{		
		if(localmaps.empty())//No Gridmaps
			this->CreateNewGridMap(rpose.p.x,rpose.p.y,rpose.phi);

		ctime = timeGetTime();

		UpdateGridMap(rpose, range, rad);//gmap->Grid_Mapping(rpose,range,rad);
		ctime = timeGetTime()-ctime;

		sprintf(str,"%dms",ctime);
		this->SetDlgItemText(IDC_EDIT1,(LPCTSTR)str);

	}

}

void MapDlg::OnBnClickedButton1()
{
	// TODO: Add your control notification handler code here
	this->OnPaint();
}

void MapDlg::OnBnClickedLmap()
{
	// TODO: Add your control notification handler code here
  
	char str[64];
	DWORD dtime;
	DWORD ctime;
	ctime = timeGetTime();

	for(int i=0;i<localmaps.size();++i)
		localmaps[i]->LikelihoodMap(0.5);//0.5m limit

	ctime = timeGetTime() - ctime;

	dtime = timeGetTime();

	DrawfromLMAP(gmap->getLMAP(),gmap->size(),0.5);

	viewer.CopyFrom(buffer,buffersize);//OK!  Notice that local variable should not be too big!!

	OnPaint();

	dtime = timeGetTime() - dtime;

	sprintf(str,"computing: %d ms, Draw: %d ms",ctime,dtime);

	this->SetDlgItemText(IDC_EDIT1,(LPCTSTR)str);
}

void MapDlg::OnBnClickedSavegmap()
{
	// TODO: Add your control notification handler code here
	FILE* fout = fopen("GMAP.txt","w");

	gmap->SaveGMAP(fout);

	fclose(fout);
	
}

void MapDlg::OnBnClickedLoadgmap()
{
	// TODO: Add your control notification handler code here
	FILE* fin = fopen("GMAP.txt","r");

	if(localmaps.empty()){//No Gridmaps
		gmap = new GridMap(0,0,0);
		localmaps.push_back(gmap);
	}
	
	gmap->LoadGMAP(fin);

	fclose(fin);

	this->DrawfromGMAP(gmap->getGMAP(),gmap->size());
	viewer.CopyFrom(buffer,buffersize);//OK!  Notice that local variable should not be too big!!

	OnPaint();

}

void MapDlg::OnBnClickedMclinit()
{
	// TODO: Add your control notification handler code here

	int N;
	Pose mean;
	Pose lim;

	ifstream finit("MCLInit.txt");
	fin = fopen("LASER.txt","r");

	if(!ReadRawLaserData(fin, rawpose.p.x, rawpose.p.y, rawpose.phi,rad, range)){
		this->SetDlgItemText(IDC_EDIT1,(LPCTSTR)"No data");
	    return ;
	}

	finit>>mean.p.x>>mean.p.y>>mean.phi>>lim.p.x>>lim.p.y>>lim.phi>>N;


	mean.phi *= Geom2D::PI/180.0;
	lim.phi *= Geom2D::PI/180.0;

	this->MCLInit(mean,lim,N);

	mcl->Weighting(range,rad);
	mcl->Resample();

	DrawParticles();
	this->OnPaint();
}

void MapDlg::OnBnClickedDomcl()
{
	// TODO: Add your control notification handler code here
	//read data and moving amount
	DWORD ctime;

	Transform2D t(rawpose);
	Pose dpose;

	if(!ReadRawLaserData(fin, rawpose.p.x, rawpose.p.y, rawpose.phi,rad, range)){
		this->SetDlgItemText(IDC_EDIT1,(LPCTSTR)"No data");
	    return ;
	}

	ctime = timeGetTime();

	dpose = rawpose;
	t.transform_to_relative(dpose);

	this->DoMCL(dpose,range,rad);

	ctime = timeGetTime()-ctime;

	this->SetDlgItemInt(IDC_EDIT1,ctime);

}

void MapDlg::OnBnClickedSaveimg()
{
	// TODO: Add your control notification handler code here
	char strFilter[] = { "BMP Files (*.bmp)|*.bmp|JPG Files (*.jpg)|*.jpg|All Files (*.*)|*.*||" };

	CFileDialog FileDlg(FALSE, ".bmp", NULL, 0, strFilter);

	if(FileDlg.DoModal()==IDOK)
		viewer.SavePicture(FileDlg.GetPathName());

}
void MapDlg::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here
	Release();
	OnOK();

}

void MapDlg::OnBnClickedCancel()
{
	// TODO: Add your control notification handler code here
	Release();
	OnCancel();
}

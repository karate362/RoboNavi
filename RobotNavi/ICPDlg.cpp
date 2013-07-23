// ICPDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RobotNavi.h"
#include "ICPDlg.h"
#include "IO_Functions.h"
#include <mmsystem.h>
#include <math.h>

// ICPDlg dialog

IMPLEMENT_DYNAMIC(ICPDlg, CDialog)

ICPDlg::ICPDlg(CWnd* pParent /*=NULL*/)
	: CDialog(ICPDlg::IDD, pParent)
{
	lin = fopen("LASER.txt","r");
	IcpOut = fopen("ICPLASER.txt","w");
	viewer.FillAll();
}

ICPDlg::~ICPDlg()
{
	fclose(lin);
}

Pose ICPDlg::getICPpose(){
	return this->icpose;
}

void ICPDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

void ICPDlg::DrawLaser(vector <Point>& data){

    double scale = 0.05; //(m/pixel)
	double x;
	double y;
	std::vector<Point>::const_iterator it; 

	for (it = data.begin(); it != data.end(); it++){ // Loop through readings
		x = (int)(it->x/scale);
		y = (int)(it->y/scale);
		viewer.DrawCircleT(x,y,1,2,255,0,0);
		}
		OnPaint();

}


BEGIN_MESSAGE_MAP(ICPDlg, CDialog)
	ON_WM_PAINT()
	ON_WM_CREATE()
	ON_BN_CLICKED(IDC_Read, &ICPDlg::OnBnClickedRead)
	ON_BN_CLICKED(IDC_Refresh, &ICPDlg::OnBnClickedRefresh)
	ON_BN_CLICKED(IDC_DOICP, &ICPDlg::OnBnClickedDoicp)
	ON_BN_CLICKED(IDC_SaveImg, &ICPDlg::OnBnClickedSaveimg)
	ON_BN_CLICKED(IDC_DOICP_CONT, &ICPDlg::OnBnClickedDoicpCont)
	ON_BN_CLICKED(IDC_DOMICP, &ICPDlg::OnBnClickedDomicp)
	ON_BN_CLICKED(IDC_DOODO_CONT, &ICPDlg::OnBnClickedDoodoCont)
	ON_BN_CLICKED(IDC_DOMICP_CONT, &ICPDlg::OnBnClickedDomicpCont)
END_MESSAGE_MAP()


// ICPDlg message handlers

bool ICPDlg::UpdateData(){//data1<--data2, data2<--newReading, data2 in (r,th) form

	int s = 0;
	int i = 0;
	Point np;
	Pose newpose;

	if(!ReadRawLaserData(lin, newpose.p.x, newpose.p.y, newpose.phi,rad, range))
		return false;
/*
	if( dist(rawpose.p,newpose.p) <= 0.5 && fabs(rawpose.phi - newpose.phi) <= PI/6 )
		return true;*/

	rawpose = newpose;

	
	if(data1.size()!=data2.size())
		data1.resize(data2.size());
		
	copy(data2.begin(),data2.end(),data1.begin());
	data2.clear();

	s = range.size();
	for(i=0;i<s;++i){
		np.x = range[i];
		np.y = rad[i];
		data2.push_back(np);
	}

	return true;
 

}

void ICPDlg::OnBnClickedRead()
{
	// TODO: Add your control notification handler code here
   
	UpdateData();
	icpod.ICP_Init(data2,rawpose);
	icpod.GetTransReading(data3);
	DrawLaser(data3);	   

}

void ICPDlg::DoICP(){

	icpod.DoICPOdemetry(data2,rawpose);
	icpose = icpod.GetIcpPose();
	icpod.GetTransReading(data3);

}


void ICPDlg::DoMICP(){

	micpod.DoICPOdemetry(data2,rawpose);
	//micpod.DoVoteICPOdemetry(data2,rawpose);
	icpose = micpod.GetIcpPose();
	micpod.GetTransReading(data3);
}

void ICPDlg::OnBnClickedDoicp()
{
	char icp_result[64];
	DWORD ctime = timeGetTime();

	if(!UpdateData())
		SetDlgItemText(IDC_EDIT1,(LPCTSTR)"file end");

	DoICP();

	sprintf(icp_result,"%.2f, %.2f, %.2f, %dms",icpose.p.x,icpose.p.y,icpose.phi,timeGetTime()-ctime);
	SetDlgItemText(IDC_EDIT1,(LPCTSTR)icp_result);
	WriteMeasurementData(IcpOut,icpose.p.x,icpose.p.y,icpose.phi,range);
    
	DrawLaser(data3);	    
}




void ICPDlg::OnBnClickedSaveimg()
{
	// TODO: Add your control notification handler code here
	char strFilter[] = { "BMP Files (*.bmp)|*.bmp|JPG Files (*.jpg)|*.jpg|All Files (*.*)|*.*||" };

	CFileDialog FileDlg(FALSE, ".bmp", NULL, 0, strFilter);

	if(FileDlg.DoModal()==IDOK)
		viewer.SavePicture(FileDlg.GetPathName());
}

void ICPDlg::OnBnClickedRefresh()
{
	// TODO: Add your control notification handler code here
	viewer.FillAll();
	OnPaint();
}



void ICPDlg::OnBnClickedDoicpCont()
{
	char icp_result[64];
	DWORD ctime = timeGetTime();


	while(UpdateData()){

	DoICP();

	sprintf(icp_result,"%.2f, %.2f, %.2f, %dms",icpose.p.x,icpose.p.y,icpose.phi,timeGetTime()-ctime);
	SetDlgItemText(IDC_EDIT1,(LPCTSTR)icp_result);
	WriteMeasurementData(IcpOut,icpose.p.x,icpose.p.y,icpose.phi,range);
    
	DrawLaser(data3);
	}
		SetDlgItemText(IDC_EDIT1,(LPCTSTR)"file end");
}

void ICPDlg::OnBnClickedDomicp()
{
	// TODO: Add your control notification handler code here
	char icp_result[64];
	DWORD ctime = timeGetTime();

	if(!UpdateData())
		SetDlgItemText(IDC_EDIT1,(LPCTSTR)"file end");

	DoMICP();

	sprintf(icp_result,"%.2f, %.2f, %.2f, %dms",icpose.p.x,icpose.p.y,icpose.phi,timeGetTime()-ctime);
	SetDlgItemText(IDC_EDIT1,(LPCTSTR)icp_result);
	WriteMeasurementData(IcpOut,icpose.p.x,icpose.p.y,icpose.phi,range);
    
	DrawLaser(data3);	 
}

void ICPDlg::OnBnClickedDoodoCont()
{
	// TODO: Add your control notification handler code here
	char icp_result[64];
	DWORD ctime = timeGetTime();


	while(UpdateData()){

	//DoMICP();
	sprintf(icp_result,"%.2f, %.2f, %.2f, %dms",rawpose.p.x,rawpose.p.y,rawpose.phi,timeGetTime()-ctime);
	SetDlgItemText(IDC_EDIT1,(LPCTSTR)icp_result);

    DataPolarToXY(data2);
	VectorTrans(data2, this->rawpose);
	DrawLaser(data2);
	}
		SetDlgItemText(IDC_EDIT1,(LPCTSTR)"file end");
}

void ICPDlg::OnBnClickedDomicpCont()
{
		char icp_result[64];
	DWORD ctime = timeGetTime();


	while(UpdateData()){

	DoMICP();

	sprintf(icp_result,"%.2f, %.2f, %.2f, %dms",icpose.p.x,icpose.p.y,icpose.phi,timeGetTime()-ctime);
	SetDlgItemText(IDC_EDIT1,(LPCTSTR)icp_result);
	WriteMeasurementData(IcpOut,icpose.p.x,icpose.p.y,icpose.phi,range);
    
	DrawLaser(data3);
	}
		SetDlgItemText(IDC_EDIT1,(LPCTSTR)"file end");
}

// DATMODlg.cpp : implementation file
//
#include "stdafx.h"
#include "RobotNavi.h"
#include "DATMODlg.h"

using namespace Geom2D;
// DATMODlg dialog

IMPLEMENT_DYNAMIC(DATMODlg, CDialog)

DATMODlg::DATMODlg(CWnd* pParent /*=NULL*/)
	: CDialog(DATMODlg::IDD, pParent)
{
	gmap = this->datmo.getGMAP();
	buffersize = 4*gmap->Width()*gmap->Height();
	buffer = new BYTE[buffersize];
	nmap = new GridMap();

}

DATMODlg::~DATMODlg()
{
}

void DATMODlg::KFilter(vector<Geom2D::Pedestrian>& current_pedestrian,DWORD scan_time){
	int i = 0;
	//double predicted_x = 0;
	//double predicted_y = 0;
	int s = 0;
	double z[2];
	s = current_pedestrian.size();
	//if (s != 0)
	for (i = 0;i < s; i++){
			if (current_pedestrian[i].present == 1) {
				if (current_pedestrian[i].appear_time == 0)
					kfupdate.KFInitialize(current_pedestrian[i]);
				else{
				z[0] = current_pedestrian[i].x;
				z[1] = current_pedestrian[i].y;
				kfupdate.measurementUpdate(current_pedestrian[i], z);
				kfupdate.timeUpdate(current_pedestrian[i], scan_time);
				}
			}
			else if (current_pedestrian[i].present == 0){
				kfupdate.timeUpdate(current_pedestrian[i], scan_time);
			}
	}
		
}

void DATMODlg::DrawDATMO(vector<Geom2D::Pedestrian>& tracked_pedestrian, Geom2D::Occupied_ID & current_ID, DWORD scan_time){

	int s=0;
	int i=0;
	int j = 0;
	int w=0;
	int h=0;
	int w1=0;
	int h1=0;
	int p_w = 0;
	int p_h = 0;
	int p_w1 = 0;
	int p_h1 = 0;
	int p_w2 = 0;
	int p_h2 = 0;
	int p_w3 = 0;
	int p_h3 = 0;
	int p_w4 = 0;
	int p_h4 = 0;
	int count = 0;
	int tracked_count = 0;
	double slope = 0;

	vector<Geom2D::Point> MOs;
    vector<Geom2D::Point> CMOs;

	vector<Geom2D::Pedestrian> current_pedestrian;

	datmo.getMOs(MOs);
	datmo.ClusterMOs(MOs);
	datmo.getCMOs(CMOs);
	datmo.DetectPedestrian(CMOs, current_pedestrian);
	datmo.TrackingPedestrian(tracked_pedestrian, current_pedestrian,current_ID);
	//filter_velocity(current_pedestrian);

	KFilter(current_pedestrian, scan_time);
	//filter_velocity(current_pedestrian);
	
	s = MOs.size();
	count = current_pedestrian.size();
#if 0
	if (count != 0)
	for (i = 0;i < count; i++){
		for (j = 0;j < tracked_count;j++){
			if (current_pedestrian[i].id == tracked_pedestrian[j].id)

	
	}
#endif
	
	//if (count != 0)
		
	//	datmo.writePosition(current_pedestrian);

	gmap = this->datmo.getGMAP();

	GMAPtoBuffer();
	viewer.CopyFrom(this->buffer,this->buffersize);

	s = CMOs.size();
#if 0
	for(i=0;i<s;++i){
		
		if (i+1 < s)
		if (dist(CMOs[i],CMOs[i+1]) < 0.3)
		{
			gmap->XYToGrid(CMOs[i].x,CMOs[i].y,w,h);
			viewer.DrawCircle(w,h,5,3,0,0,255);
			gmap->XYToGrid(CMOs[i+1].x,CMOs[i+1].y,w,h);
			viewer.DrawCircle(w,h,5,3,0,0,255);
			
		}
		else
		{
			gmap->XYToGrid(CMOs[i].x,CMOs[i].y,w,h);
			viewer.DrawCircle(w,h,5,3,0,255,0);
			
		}

	}
#endif
	Point center;
	Point robot;
	center.x = 0;
	center.y = 0;
	datmo.getCenter(center);
	slope = (center.y - robot.y) / (center.x - robot.x);
	double temp_x = 0;
	double temp_y = 0;

#if 1
	for(i=0;i<count;++i){
		gmap->XYToGrid(center.x,center.y,w,h);
		//viewer.DrawCircle(w,h,2,4,255,105,180);
		//viewer.DrawCircle(w,h,2,4,000,191,255);
		viewer.DrawCircle(w,h,2,4,000,000,255);
		//viewer.DrawLine(w,h,w1,h1,5,255,105,180);
#if 1
		if (current_pedestrian[i].id == 0){
			gmap->XYToGrid(current_pedestrian[i].x,current_pedestrian[i].y,w,h);
			viewer.DrawCircle(w,h,5,3,169,169,169);
			gmap->XYToGrid(current_pedestrian[i].Tmean[0],current_pedestrian[i].Tmean[1],p_w,p_h);
			viewer.DrawCircle(p_w,p_h,1.5,4,255,000,255);

		}
		else if (current_pedestrian[i].id == 1){
			if (current_pedestrian[i].present == 1){
			gmap->XYToGrid(current_pedestrian[i].x,current_pedestrian[i].y,w,h);
			viewer.DrawCircle(w,h,5,3,255,255,0);
			gmap->XYToGrid(current_pedestrian[i].Tmean[0],current_pedestrian[i].Tmean[1],p_w,p_h);
			viewer.DrawCircle(p_w,p_h,1.5,4,255,000,255);
#if 1
			temp_x = current_pedestrian[i].Tmean[0]+current_pedestrian[i].Tmean[2];
			temp_y = current_pedestrian[i].Tmean[1]+current_pedestrian[i].Tmean[3];
			gmap->XYToGrid(temp_x,temp_y,p_w1,p_h1);
			viewer.DrawCircle(p_w1,p_h1,1.5,4,255,000,255);
			temp_x = temp_x + current_pedestrian[i].Tmean[2];
			temp_y = temp_y + current_pedestrian[i].Tmean[3];
			gmap->XYToGrid(temp_x, temp_y,p_w2,p_h2);
			viewer.DrawCircle(p_w2,p_h2,1.5,4,255,000,255);
			temp_x = temp_x + current_pedestrian[i].Tmean[2];
			temp_y = temp_y + current_pedestrian[i].Tmean[3];
			gmap->XYToGrid(temp_x, temp_y,p_w3,p_h3);
			viewer.DrawCircle(p_w3,p_h3,1.5,4,255,000,255);
			temp_x = temp_x + current_pedestrian[i].Tmean[2];
			temp_y = temp_y + current_pedestrian[i].Tmean[3];
			gmap->XYToGrid(temp_x, temp_y,p_w4,p_h4);
			viewer.DrawCircle(p_w4,p_h4,1.5,4,255,000,255);

			viewer.DrawLine(p_w, p_h, p_w1, p_h1, 2,255,000,000);
			viewer.DrawLine(p_w1, p_h1, p_w2, p_h2, 2,255,000,000);
			viewer.DrawLine(p_w2, p_h2, p_w3, p_h3, 2,255,000,000);
			viewer.DrawLine(p_w3, p_h3, p_w4, p_h4, 2,255,000,000);
#endif	
			}
			else if (current_pedestrian[i].present == 0){
				gmap->XYToGrid(current_pedestrian[i].Tmean[0],current_pedestrian[i].Tmean[1],p_w,p_h);
				viewer.DrawCircle(p_w,p_h,1.5,4,255,000,255);
			}
		}
		else if (current_pedestrian[i].id == 2){
			if (current_pedestrian[i].present == 1){
			gmap->XYToGrid(current_pedestrian[i].x,current_pedestrian[i].y,w,h);
			viewer.DrawCircle(w,h,5,3,255,000,000);
			gmap->XYToGrid(current_pedestrian[i].Tmean[0],current_pedestrian[i].Tmean[1],p_w,p_h);
			viewer.DrawCircle(p_w,p_h,1.5,4,255,000,255);
			}
			else if (current_pedestrian[i].present == 0){
				gmap->XYToGrid(current_pedestrian[i].Tmean[0],current_pedestrian[i].Tmean[1],p_w,p_h);
				viewer.DrawCircle(p_w,p_h,1.5,4,255,000,255);
			}
		}
		else if (current_pedestrian[i].id == 3){
			if (current_pedestrian[i].present == 1){
			gmap->XYToGrid(current_pedestrian[i].x,current_pedestrian[i].y,w,h);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
			viewer.DrawCircle(w,h,5,3,0,255,0);
			gmap->XYToGrid(current_pedestrian[i].Tmean[0],current_pedestrian[i].Tmean[1],p_w,p_h);
			viewer.DrawCircle(p_w,p_h,1.5,4,255,000,255);
			}
			else if (current_pedestrian[i].present == 0){
				gmap->XYToGrid(current_pedestrian[i].Tmean[0],current_pedestrian[i].Tmean[1],p_w,p_h);
				viewer.DrawCircle(p_w,p_h,1.5,4,255,000,255);
			}
		}
		else if (current_pedestrian[i].id == 4){
			if (current_pedestrian[i].present == 1){
			gmap->XYToGrid(current_pedestrian[i].x,current_pedestrian[i].y,w,h);
			viewer.DrawCircle(w,h,5,3,0,0,255);
			gmap->XYToGrid(current_pedestrian[i].Tmean[0],current_pedestrian[i].Tmean[1],p_w,p_h);
			viewer.DrawCircle(p_w,p_h,1.5,4,255,000,255);
			}
			else if (current_pedestrian[i].present == 0){
				gmap->XYToGrid(current_pedestrian[i].Tmean[0],current_pedestrian[i].Tmean[1],p_w,p_h);
				viewer.DrawCircle(p_w,p_h,1.5,4,255,000,255);
			}

		}
		else if (current_pedestrian[i].id >= 5){
			if (current_pedestrian[i].present == 1){
			gmap->XYToGrid(current_pedestrian[i].x,current_pedestrian[i].y,w,h);
			viewer.DrawCircle(w,h,5,3,0,255,255);
			gmap->XYToGrid(current_pedestrian[i].Tmean[0],current_pedestrian[i].Tmean[1],p_w,p_h);
			viewer.DrawCircle(p_w,p_h,1.5,4,255,000,255);
			}
			else if (current_pedestrian[i].present == 0){
				gmap->XYToGrid(current_pedestrian[i].Tmean[0],current_pedestrian[i].Tmean[1],p_w,p_h);
				viewer.DrawCircle(p_w,p_h,1.5,4,255,000,255);
			}
		}
#endif
	}
#endif
	ID_Update(current_pedestrian,current_ID);

	if (count != 0){
	tracked_pedestrian.resize(count);
	copy(current_pedestrian.begin(),current_pedestrian.end(),tracked_pedestrian.begin());
	}
	//else
	//	tracked_pedestrian.clear();
	this->OnPaint();
}
void DATMODlg::filter_velocity(vector<Geom2D::Pedestrian>& current_pedestrian){
	int i = 0;
	for (i = 0; i < current_pedestrian.size(); i++){
		if (current_pedestrian[i].appear_time < 3 & fabs(current_pedestrian[i].Tmean[2]) < 0.15 & fabs(current_pedestrian[i].Tmean[3]) < 0.15)
			current_pedestrian.erase(current_pedestrian.begin()+i);
	
	}

}
void DATMODlg::ID_Update(vector<Geom2D::Pedestrian>& current_pedestrian, Geom2D::Occupied_ID & current_ID){
	int i = 0;
	int count = 0;
	double ID[6];
	bool ID1_done = false;
	bool ID2_done = false;
	bool ID3_done = false;
	bool ID4_done = false;
	bool ID5_done = false;
	count = current_pedestrian.size();
	for (i = 0; i < 6; i++)
		ID[i] = 0;
	for (i = 0;i < count;i++){
		if (current_pedestrian[i].id == 1 & ID1_done == false){
			ID[1] = current_ID.used_ID[1] + 1;
			ID1_done = true;
		}
		else if (current_pedestrian[i].id == 2 & ID2_done == false){
			ID[2] = current_ID.used_ID[2] + 1;
			ID2_done = true;
		}
		else if (current_pedestrian[i].id == 3 & ID3_done == false){
			ID[3] = current_ID.used_ID[3] + 1;
			ID3_done = true;
		}
		else if (current_pedestrian[i].id == 4 & ID4_done == false){
			ID[4] = current_ID.used_ID[4] + 1;
			ID4_done = true;
		}
		else if (current_pedestrian[i].id == 5 & ID5_done == false){
			ID[5] = current_ID.used_ID[5] + 1;
			ID5_done = true;
		}
	}

	for (i = 0; i < 6; i++)
		current_ID.used_ID[i] = ID[i];

}
void DATMODlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


bool DATMODlg::UpdateData(){


	return true;
 


}


BEGIN_MESSAGE_MAP(DATMODlg, CDialog)
	ON_WM_PAINT()
	ON_WM_CREATE()
	ON_BN_CLICKED(IDOK, &DATMODlg::OnBnClickedOk)
	ON_BN_CLICKED(ID_DODATMO, &DATMODlg::OnBnClickedDodatmo)
	ON_EN_CHANGE(IDC_EDIT2, &DATMODlg::OnEnChangeEdit2)
END_MESSAGE_MAP()


// DATMODlg message handlers

void DATMODlg::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here
	//OnOK();
//Draw the transformed map
	vector<Geom2D::Pedestrian> database;
	Geom2D::Occupied_ID current_ID;
	DWORD scan_time = 500;
	Pose np;
	np.p.x = -2;
	np.p.y = 4;
	np.phi = Geom2D::PI/2;
	gmap->TransformMap(np,*nmap);
	gmap = nmap;
	this->DrawDATMO(database, current_ID,scan_time);
	gmap = this->datmo.getGMAP();

}




void DATMODlg::OnBnClickedDodatmo()
{
	// TODO: Add your control notification handler code here
}

void DATMODlg::OnEnChangeEdit2()
{
	// TODO:  如果這是 RICHEDIT 控制項，控制項將不會
	// 傳送此告知，除非您覆寫 CDialog::OnInitDialog()
	// 函式和呼叫 CRichEditCtrl().SetEventMask()
	// 讓具有 ENM_CHANGE 旗標 ORed 加入遮罩。

	// TODO:  在此加入控制項告知處理常式程式碼
}

#pragma once

#include "MFC_VIEWER.h"
#include <vector>
//#include "geometry2D.hpp"
#include "PubRobotData.h"
#include "DWA.h"
#include "TraTree.h"
#include "DWAstar.h"
#include "TwoStateEx.h"

// NavDlg dialog

class NavDlg : public CDialog
{
	DECLARE_DYNAMIC(NavDlg)

	friend DWORD WINAPI NAVI_THREAD(LPVOID lpt);
	
	friend DWORD WINAPI NAVI2_THREAD(LPVOID lpt);

	friend DWORD WINAPI NAVI3_THREAD(LPVOID lpt);

public:
	NavDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~NavDlg();

	void Planning(double lgx,double lgy);

	void SetData(RobotTra::DWA* dwapt,PubRobotData* PRDatapt){
		dwa = dwapt;
		PRData = PRDatapt;
	}

	void GetVel(double &v, double &w){
		v = vel_v, w=vel_w;
	}

// Dialog Data
	enum { IDD = IDD_NAVDLG };


private:
	MFC_VIEWER viewer;
	RobotTra::DWA* dwa;
	PubRobotData* PRData;
	ObsArray obsarr;
	RobotTra::TraTree DWAtree;
	ND nd;

	double vel_v;
	double vel_w;
	
	DWAstar dywstar;

	std::vector<double>range;
	std::vector<double>rad;
	Geom2D::Pose rawpose;

	FILE* lin;

	int counter;

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()

public:

	afx_msg void OnPaint(){
		CClientDC dc(this); 
		viewer.PasteOn(dc,0,0);
		CDialog::OnPaint();		
	}
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct){
		CClientDC dc(this);
		viewer.init(dc,320,240);
		viewer.FillAll(0,0,0);
		return 0;
	}
	afx_msg void OnBnClickedCompute();
	afx_msg void OnBnClickedExpand();
	afx_msg void OnBnClickedOffline();
	afx_msg void OnBnClickedSaveimg();
	afx_msg void OnBnClickedOffline2();
	afx_msg void OnBnClickedOk();
};

void DrawSensorReading(MFC_VIEWER& viewer, std::vector<double> &range, std::vector<double> &rad, double scale);
void DrawItv(MFC_VIEWER& viewer, RobotTra::VelItv& vitv, double scale);
void DrawCandidateVel(MFC_VIEWER& viewer, RobotTra::DWA& dwa, double scale);
void DrawBestPath(MFC_VIEWER& viewer, RobotTra::TraTree& DWAtree, double scale);
void DrawTargetPath(MFC_VIEWER& view, vector<GoalPoint>& TT, double scale);
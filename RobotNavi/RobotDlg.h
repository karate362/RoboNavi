#pragma once

#include "P3robot.h"
#include "ActiveSocket.h"       // Include header for active socket object definition
#include "MFC_VIEWER.h"
#include "PubRobotData.h"
#include <math.h>

#include "JoyStick.h"
#include "LaserDlg.h"
#include "ICPDlg.h"
#include "MapDlg.h"
#include "JoyDlg.h"
#include "NavDlg.h"
#include "DATMODlg.h"

// RobotDlg dialog

#define TEST_PACKET "I love to be scorned by nanoha"
#define MAX_PACKET  1024

DWORD WINAPI JOYCTRL_THREAD(LPVOID lpt);
DWORD WINAPI LASER_THREAD(LPVOID lpt);
DWORD WINAPI NAVI_THREAD(LPVOID lpt);
DWORD WINAPI NAVI2_THREAD(LPVOID lpt);
DWORD WINAPI NAVI3_THREAD(LPVOID lpt);
DWORD WINAPI SLAM_THREAD(LPVOID lpt);
DWORD WINAPI DATMO_THREAD(LPVOID lpt);

class RobotDlg : public CDialog
{
	DECLARE_DYNAMIC(RobotDlg)
	friend DWORD WINAPI JOYCTRL_THREAD(LPVOID lpt);
	friend DWORD WINAPI LASER_THREAD(LPVOID lpt);
	friend DWORD WINAPI NAVI_THREAD(LPVOID lpt);
	friend DWORD WINAPI NAVI2_THREAD(LPVOID lpt);
	friend DWORD WINAPI NAVI3_THREAD(LPVOID lpt);
	friend DWORD WINAPI SLAM_THREAD(LPVOID lpt);
	friend DWORD WINAPI DATMO_THREAD(LPVOID lpt);
	friend DWORD WINAPI DATMO_THREAD_OFFLINE(LPVOID lpt);
public:
	RobotDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~RobotDlg();

	void EndDlg();//Disconnection, release...

	void StartLaserThread();
	void StartSLAMThread();
	void StartNaviThread();

	void GetRawPose(double &x, double &y, double &rad){
		x = probot.getX();
		y = probot.getY();
		rad = probot.getThRad();
	}

	void SLAM(Geom2D::Pose &rpose,std::vector<double> &nrange,std::vector<double> &nrad);//rpose will be set as localized...

// Dialog Data
	enum { IDD = IDD_ROBOT };

public:
	P3robot probot;


protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCancel();
	afx_msg void OnBnClickedConnect();
	afx_msg void OnBnClickedStick();

private:
	//void JoyProcess(const char* data);
    void JoyKeyDown(int action);
	void JoyVel(int x,int y);

	void (*MeasurementOutput)(void* robot,void* sensor,void* out);//Function pointer, used to output data
	bool (*StartLaser)(void* laser);
	void (*getLaser)(void* laser,std::vector<double> &range,std::vector<double> &rad);
	void (*StopLaser)(void* laser);

		

private:
	MFC_VIEWER viewer;

	HANDLE hThread; //JoyStick thread
    DWORD dwThreadID;
	bool IsThread;
	double jvmax;
	double jwmax;
	JoyDlg* jdlg;


	HANDLE hThread2; //Laser thread (Sim or Real)
    DWORD dwThreadID2;
	bool IsThread2;
	LaserDlg* ldlg;

	HANDLE hThread3; //Navi thread 
    DWORD dwThreadID3;
	bool IsThread3;
	NavDlg* ndlg;

	HANDLE hThread4; //SLAM thread 
    DWORD dwThreadID4;
	bool IsThread4;

	HANDLE hThread5; //DATMO thread 
    DWORD dwThreadID5;
	bool IsThread5;

	//void* for function pointers
	void* robotpt;
	void* laserpt;

	//SLAM objects
	ICPDlg* idlg;
	MapDlg* mdlg;
	DATMODlg* ddlg;
	FILE* lout;
	bool mapping;
	Geom2D::Pose icpose;

	//Public data
	PubRobotData PRData;



public:
	afx_msg void OnBnClickedSimlaser();

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

	afx_msg void OnBnClickedSavelaser();
	afx_msg void OnBnClickedLms100laser();
	afx_msg void OnBnClickedReode();
	afx_msg void OnBnClickedMclstart();
	afx_msg void OnBnClickedNavi();
	afx_msg void OnBnClickedLaser();
	afx_msg void OnBnClickedDatmo();
	afx_msg void OnBnClickedSlam();
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedNavi2();
	afx_msg void OnBnClickedNavi3();
};


#if !defined(AFX_RNAVDLG_H__DBCC2FAC_945E_4992_95F2_03A0CFCD17C2__INCLUDED_)
#define AFX_RNAVDLG_H__DBCC2FAC_945E_4992_95F2_03A0CFCD17C2__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// RNavDlg.h : header file
//
#include "StdAfx.h"

#include "MFC_VIEWER.h"
#include "Aria.h"
#include<iostream>
#include<fstream>
#include<math.h>
#include<time.h>
#include <direct.h>

#include "ObsArray.h"
#include "DyWin.h"
#include "ND.h"
#include "ASTAR.h"

#include "LaserThread.h"

#pragma   comment(lib,"winmm.lib") 

using namespace std;
/////////////////////////////////////////////////////////////////////////////
// RNavDlg dialog

class RNavDlg : public CDialog
{
// Construction
public:
	RNavDlg(CWnd* pParent = NULL);   // standard constructor
    ~RNavDlg()
	{
		exit(0);
	}
	void Ctrl_START();

	void RePaint(){this->OnPaint();}
// Dialog Data
	//{{AFX_DATA(RNavDlg)
	enum { IDD = IDD_PCtrl_DIALOG };
		// NOTE: the ClassWizard will add data members here
	//}}AFX_DATA

public:
	///////////////ROBOT////////////////////////////
    // robot
    ArRobot robot;
    // a laser in case one is used
    ArSick sick;
    ArSimpleConnector* simpleConnector;
    // a key handler so we can do our key handling
    ArKeyHandler keyHandler;
    // sonar, must be added to the robot, for teleop and wander
    ArSonarDevice sonarDev;
    ArAnalogGyro* gyro;
	///////////////ROBOT////////////////////////////

    //////////////LRF//////////////////////////////

    LaserThread Lthread;
	//////////////////////////////////////////////


    HANDLE hThread0; //robot setup 
    DWORD dwThreadID0;

    //////////////Algorithm////////////////////////
	int algorithm;
	int sensor;
	int sdepth;
	//////////////////////////////////////////////


	/////record///////////////////////////////////
	bool is_mkdir;
	bool is_path;
	bool is_obs;
	bool is_itv;
	bool is_reg;
	bool is_tree;
	/////////////////////////////////////////////

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(RNavDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(RNavDlg)
   afx_msg void OnPaint();
   afx_msg void OnCreate(LPCREATESTRUCT lpCreateStruct);
   virtual void OnOK();
   BOOL OnInitDialog();
   //void OnInitDialog();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};




//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.



#endif // !defined(AFX_RNAVDLG_H__DBCC2FAC_945E_4992_95F2_03A0CFCD17C2__INCLUDED_)

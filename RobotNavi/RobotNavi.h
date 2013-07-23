// RobotNavi.h : main header file for the RobotNavi application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols


// CRobotNaviApp:
// See RobotNavi.cpp for the implementation of this class
//

class CRobotNaviApp : public CWinApp
{
public:
	CRobotNaviApp();


// Overrides
public:
	virtual BOOL InitInstance();

// Implementation
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
	afx_msg void OnFileRobotstart();
	afx_msg void OnFileLaserstart();
	afx_msg void OnFileIcpstart();
	afx_msg void OnFileDrawmap();
	afx_msg void OnFileJoystickstart();
	afx_msg void OnFileNavistart();
};

extern CRobotNaviApp theApp;
#pragma once

#include "MFC_VIEWER.h"
#include "SickLms100Laser.h"
#include <stdio.h>
#include <vector>
using namespace std;
using namespace Robotics;
// LaserDlg dialog

#define DATA_LENGTH 541
#define DEVICE_ADDR "192.168.0.2"

DWORD WINAPI READ_THREAD (LPVOID lpt);

class LaserDlg : public CDialog
{
	DECLARE_DYNAMIC(LaserDlg)
	friend DWORD WINAPI READ_THREAD (LPVOID lpt);

private:
	//viewer
	MFC_VIEWER lview;

	//Laser reading
    SickLms100Laser* pLaser;
	int aScan[DATA_LENGTH];

	//Thread
    HANDLE hReadThread; //read thread
    DWORD dwReadThreadID;
	bool IsReadCont;


	vector<double> LRange;
	vector<double> LRad;

	HANDLE lmutex;
	FILE* lout;


public:
	LaserDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~LaserDlg();
	bool ReadLaser();

	const vector<double>& getRange(){return LRange;}
	const vector<double>& getRad(){return LRad;}

	void getRange(vector<double>& dist);
	void getRad(vector<double>& dist);

	bool StartLaser();
	void getLaser(vector<double>& range, vector<double>& rad);
	void getLaser();
	void StopLaser();
	void SaveLaser(FILE* out);



public:

	bool IsThreadAlive(){//If the thread is running
	    DWORD ExitCode;
		GetExitCodeThread(hReadThread,&ExitCode);
		if(ExitCode == STILL_ACTIVE)
			return true;
		else
			return false;
	
	}
	void OnSuccess(){
		    SetDlgItemText(IDC_EDIT1,(LPCTSTR)"Success!");
	        HBITMAP hbmp;
			hbmp = (HBITMAP)LoadImage(AfxGetInstanceHandle(),MAKEINTRESOURCE(IDB_BITMAP3),IMAGE_BITMAP,0,0,LR_LOADMAP3DCOLORS);
			CStatic* PIC =(CStatic*)GetDlgItem(IDC_STATEPIC);
			PIC->ModifyStyle(0xF,SS_BITMAP);
			PIC->SetBitmap(hbmp);
	}
	void OnConnectErr(){
	        HBITMAP hbmp;
			hbmp = (HBITMAP)LoadImage(AfxGetInstanceHandle(),MAKEINTRESOURCE(IDB_BITMAP1),IMAGE_BITMAP,0,0,LR_LOADMAP3DCOLORS);
			CStatic* PIC =(CStatic*)GetDlgItem(IDC_STATEPIC);
			PIC->ModifyStyle(0xF,SS_BITMAP);
			PIC->SetBitmap(hbmp);
	}
	void OnReadErr(){
	        HBITMAP hbmp;
			hbmp = (HBITMAP)LoadImage(AfxGetInstanceHandle(),MAKEINTRESOURCE(IDB_BITMAP2),IMAGE_BITMAP,0,0,LR_LOADMAP3DCOLORS);
			CStatic* PIC =(CStatic*)GetDlgItem(IDC_STATEPIC);
			PIC->ModifyStyle(0xF,SS_BITMAP);
			PIC->SetBitmap(hbmp);
	}
// Dialog Data
	enum { IDD = IDD_LASER };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
    afx_msg void OnPaint();
    afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);



	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedSingle();
	afx_msg void OnStnClickedPic1();
	afx_msg void OnBnClickedConnnect();
	afx_msg void OnBnClickedClose();
	afx_msg void OnBnClickedCont();
	afx_msg void OnBnClickedSavelaser();
};

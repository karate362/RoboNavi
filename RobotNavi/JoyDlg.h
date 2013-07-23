#pragma once

#include "MFC_VIEWER.h"
#include <queue>

using namespace std;

// JoyDlg dialog

class JoyDlg : public CDialog
{
	DECLARE_DYNAMIC(JoyDlg)

public:
	JoyDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~JoyDlg();

	void GetPosition(int& x, int& y){
		x=jx;
		y=jy;
	}

	int GetAction(){//-1 if no actions

		int rval = -1;

		if(!keyqueue.empty()){
			rval = keyqueue.front();
			keyqueue.pop();
		}
		
		return rval;
	}


// Dialog Data
	enum { IDD = IDD_JOYDLG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	BOOL OnInitDialog();
	afx_msg void OnPaint();

public:
	afx_msg void OnBnClickedCancel();
	afx_msg void OnTimer(UINT_PTR nIDEvent);

public:
	MFC_VIEWER jview;

private:
	queue<int> keyqueue;//action queue
	int jx;
	int jy;//0~65535
	char Keyold[32];

	DECLARE_MESSAGE_MAP()

public:
	afx_msg void OnBnClickedOk();
};

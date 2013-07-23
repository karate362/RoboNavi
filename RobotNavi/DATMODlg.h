#pragma once

#include "MFC_VIEWER.h"
#include "GridMap.h"
#include "DATMO.h"
#include "KFUpdate.h"

// DATMODlg dialog

class DATMODlg : public CDialog
{
	DECLARE_DYNAMIC(DATMODlg)

public:
	DATMODlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~DATMODlg();

	afx_msg void OnPaint(){
		CClientDC dc(this); 
		viewer.PasteOn(dc,0,0);
		CDialog::OnPaint();		
	}

	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct){
		CClientDC dc(this);
		viewer.init(dc,gmap->Width(),gmap->Height());

		viewer.FillAll(0,0,0);
		return 0;
	}

	void GMAPtoBuffer(){
		double l;
		double d;
		BYTE b=0;

		double* src = this->gmap->getGMAP();
		int s = this->gmap->size();

		for(int i=0;i<s;++i){

			l = exp(src[i]);
			d = l/(1+l);//probability
			d = 255*d;

			b = (BYTE)d;

			buffer[4*i] = b;
			buffer[4*i+1] = b;
			buffer[4*i+2] = b;
			buffer[4*i+3] = 0x00;
		}

	}
	void KFilter(vector<Geom2D::Pedestrian>& current_pedestrian,DWORD scan_time);
	void DrawDATMO(vector<Geom2D::Pedestrian>& tracked_pedestrian, Geom2D::Occupied_ID & current_ID, DWORD scan_time);
	void filter_velocity(vector<Geom2D::Pedestrian>& current_pedestrian);
	void DATMODlg::ID_Update(vector<Geom2D::Pedestrian>& current_pedestrian, Geom2D::Occupied_ID & current_ID);
	bool UpdateData();

// Dialog Data
	enum { IDD = IDD_DATMO };


public:
	DATMO datmo;
public:
	KFUpdate kfupdate;

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()


private:
	MFC_VIEWER viewer;
	BYTE* buffer;//1byte-->4byte
	int buffersize;
	GridMap* gmap;// Present map
	GridMap* nmap;

public:
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedDodatmo();
	afx_msg void OnEnChangeEdit2();
};

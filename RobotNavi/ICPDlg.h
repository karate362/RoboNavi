#pragma once
#include <stdio.h>
#include "MFC_VIEWER.h"
#include <math.h>
#include "ICP_Odemetry.h"
#include "MICP_Odometry.h"

using namespace Geom2D;
// ICPDlg dialog

class ICPDlg : public CDialog
{
	DECLARE_DYNAMIC(ICPDlg)

public:
	ICPDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~ICPDlg();

	Pose getICPpose();

	void SetData(vector <Point>& data, Pose rpose){//data1<--data2, data2<--new Data

		if(data1.size()!=data2.size())
			data1.resize(data2.size());

		copy(data2.begin(),data2.end(),data1.begin());
        
		if(data2.size()!=data.size())
			data2.resize(data.size());

		copy(data.begin(),data.end(),data2.begin());

		this->rawpose = rpose;
	}

	void CallDoICP(vector <Point>& data, Pose rpose){
		SetData(data,rpose);
		DoICP();
		DrawLaser(data3);
	}

	Pose CallComputeICP(vector <double>& range,vector <double>& rad, Pose rpose){

		Point np;
		databuf.clear();

		for(int i=0;i<range.size();++i){
			//ICP   
		    np.x = range[i];
		    np.y = rad[i];
		    databuf.push_back(np);
		}

		return icpod.ComputeICPOdemetry(databuf, rpose);
		
	}

	void CallDoICP(vector <double>& range,vector <double>& rad, Pose rpose){

		Point np;
		databuf.clear();

		for(int i=0;i<range.size();++i){
			//ICP   
		    np.x = range[i];
		    np.y = rad[i];
		    databuf.push_back(np);
		}

		SetData(databuf,rpose);
		DoICP();
		DrawLaser(data3);
	}



	Pose CallComputeMICP(vector <double>& range,vector <double>& rad, Pose rpose){

		Point np;
		databuf.clear();

		for(int i=0;i<range.size();++i){
			//ICP   
		    np.x = range[i];
		    np.y = rad[i];
		    databuf.push_back(np);
		}

		return micpod.ComputeICPOdemetry(databuf, rpose);
		
	}


	void CallDoMICP(vector <double>& range,vector <double>& rad, Pose rpose){

		Point np;
		databuf.clear();

		for(int i=0;i<range.size();++i){
			//ICP   
		    np.x = range[i];
		    np.y = rad[i];
		    databuf.push_back(np);
		}

		SetData(databuf,rpose);
		DoMICP();
		DrawLaser(data3);
	}



// Dialog Data
	enum { IDD = IDD_ICP };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()


private:
		MFC_VIEWER viewer;
		FILE* lin;
		FILE* IcpOut;
		char LaserStr[8196];


		//ICP parameters
		vector <double> rad;
		vector <double> range;
		vector <Point> data1;//Last Polar data
        vector <Point> data2;//Now Polar data
		vector <Point> data3;//(x,y) data in global coordinate
		vector <Point> databuf;//
		Pose rawpose;
		Pose icpose;

		ICP_Odemetry icpod;
		MICP_Odometry micpod;

public:
	afx_msg void OnPaint(){
		CClientDC dc(this); 
		viewer.PasteOn(dc,0,0);
		CDialog::OnPaint();		
	}
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct){
		CClientDC dc(this);
		viewer.init(dc,320,240);
		viewer.FillAll();
		return 0;
	}
	afx_msg void OnBnClickedRead();

private:
	void DrawLaser(vector <Point>& data);
	bool UpdateData();
	void DoICP();
	void DoMICP();
public:
	afx_msg void OnBnClickedRefresh();
	afx_msg void OnBnClickedDoicp();
	afx_msg void OnBnClickedSaveimg();
	afx_msg void OnBnClickedDoicpCont();
	afx_msg void OnBnClickedDomicp();
	afx_msg void OnBnClickedDoodoCont();
	afx_msg void OnBnClickedDomicpCont();
};

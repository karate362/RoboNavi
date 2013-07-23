#pragma once

#include"MFC_VIEWER.h"
#include "GridMap.h"
#include "MCL.h"

// MapDlg dialog

class MapDlg : public CDialog
{
	DECLARE_DYNAMIC(MapDlg)

public:
	MapDlg(int m_width, int m_height,CWnd* pParent = NULL);   // standard constructor
	virtual ~MapDlg();

	//called from other object, draw on gmap
	void UpdateGridMap(const Geom2D::Pose& rpose, const std::vector<double>& range, const std::vector<double>& nrad);

	//Create a new map and push it into localmaps, gmap always points the last member in localmaps
	void CreateNewGridMap(double x,double y,double ang);

	void MCLInit(Geom2D::Pose mean,Geom2D::Pose sigma,int pnum);

	void DoMCL(Geom2D::Pose newpose, const std::vector<double>& range, const std::vector<double>& nrad);

	Geom2D::Pose getMCLpose(){return mclpose;}
	Geom2D::Point getLocalGoal(){return pgoal;}
	Geom2D::Point getLocalGoalT();

	void Release();//Release all objects

//Drawing commands
public:

	void DrawfromByteBuffer(BYTE* src,int size){
		int s = min(size,height*width);
		
		for(int i=0;i<s;++i){
			buffer[4*i] = src[i];
			buffer[4*i+1] = src[i];
			buffer[4*i+2] = src[i];
			buffer[4*i+3] = 0x00;
		}
	}

	void DrawfromGMAP(double* src,int size){
		int s = min(size,height*width);
		double l;
		double d;
		BYTE b=0;

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

	void DrawfromLMAP(double* src,int size,double limit){
		int s = min(size,height*width);
		double d;
		BYTE b=0;

		for(int i=0;i<s;++i){

			d = src[i]/limit;
			d = max(0,d);
			d = min(1,d);
			d = 255*d;

			b = (BYTE)d;

			buffer[4*i] = b;
			buffer[4*i+1] = b;
			buffer[4*i+2] = b;
			buffer[4*i+3] = 0x00;
		}

	}


	void DrawParticles();
	void DrawBestParticle(Geom2D::Pose rpose, const std::vector<double>& nrange, const std::vector<double>& nrad);
	void DrawLocalGoal();


	afx_msg void OnPaint(){
		CClientDC dc(this); 
		viewer.PasteOn(dc,0,0);
		CDialog::OnPaint();		
	}

	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct){
		CClientDC dc(this);
		viewer.init(dc,width,height);

		viewer.FillAll(0,0,0);
		return 0;
	}

// Dialog Data
	enum { IDD = IDD_MAP };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support


private:

	int width;
	int height;
	int buffersize;

	MFC_VIEWER viewer;
	BYTE* buffer;//1byte-->4byte
	GridMap* gmap;// Present map
	GridMap* lmap;//Gridmap used for localization
	Geom2D::Point pgoal;//present goal
	vector <GridMap*> localmaps;//Save local maps
	vector <Geom2D::Point> localgoals;//When the robot want to return, save the local goal
	MCL* mcl;
	FILE* fin;



	Geom2D::Pose rawpose;//Read txt and do MCL
	Geom2D::Pose mclpose;
	std::vector<double> range;
	std::vector<double> rad;

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedDrawmap();
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedLmap();
	afx_msg void OnBnClickedSavegmap();
	afx_msg void OnBnClickedLoadgmap();
	afx_msg void OnBnClickedMclinit();
	afx_msg void OnBnClickedDomcl();
	afx_msg void OnBnClickedSaveimg();
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCancel();
};

// DyWin.h: interface for the DyWin class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_DYWIN_H__1CA4A271_D4BA_4203_8185_AE473A10AEED__INCLUDED_)
#define AFX_DYWIN_H__1CA4A271_D4BA_4203_8185_AE473A10AEED__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include<math.h>
#include<vector>
#include<fstream>
#include "ObsArray.h"
#include "VelReg.h"
using namespace std;

class _declspec(dllexport) DyWin  
{
public:
	DyWin();
	void init(int w, int h, double vmax, double vmin, double wmax, double wmin, double vd, double wd,double dt,vector<OBS>* ob);
   	void init(ifstream& pars, ifstream& obsw,vector<OBS>* ob);
	bool Low_Safety(double savety,double goal_ang, double& rv, double& rw);//環境在低安全性時傳回true
	bool NON_REG();//是否只剩左右回轉的選項
	
	int DyWin_search(double v, double w, double rx,double ry,double rth,double gx,double gy, double& rv, double& rw);
	int DyWin_Reg_search(double v, double w, double lgx,double lgy,double Safety);
	//reward function
	double heading(double v,double w, double dt, double rx,double ry,double rth,double gx,double gy);
	
	double target_heading(double rx,double ry,double rth,double gx,double gy);

    double dist(double v,double w,double dt);

    double margin(double v,double w,double dt);
	
	double margin2(double v,double w,double dt);

	void compute_obf(double v, double w, double dt,double lgx,double lgy, double rv, double rw);//compute objective function

	void Regcompute_obf(double v, double w, double dt, double rv, double rw);//compute objective function


	virtual ~DyWin();
	//queue <OBS> obs;

	void Region_Analysis(double threshold,double lgx,double lgy);

	int VeltoITV(double v,double w)
	{
     return VR.DegtoITV(atan2(v,w)*180/PI);
	}

	DWA_VALLEY* A_star(double lgx,double lgy,double dt);

	void SaveITV(ofstream& outfile);

private:

public:


	int MAX_sonar_range;

	/////////////////DWA/////////////////////

	int sw;
	int sh;//size of reward space
    int dw;
    int dh;
	int obsize;//OBS中的障礙物個數

	double a[5];//weight vector
	double b[5];//objective vector
	double c[5];//region weight vector

	double V_MAX;
    double V_MIN;
	double W_MAX;
    double W_MIN;
	double delta_v;
	double delta_w;
	double dw_width;
	double dw_height;
	double va;
	double wa;
	double delta_t;
	double vc;
	double wc;

	double dis[2];
	//double dm;//the distance between the robot and the nearest obstacle

    double robot_radius;

	//vector <OBS> obs;
	vector <OBS> *obspt;
	///////////////////////////////////////////////////////////
	//velocity space interval
    VelReg VR;

	ITV* intervals;

};

void GPtoLP(double rx,double ry,double rth,double gx,double gy,double& lgx,double& lgy);


#endif // !defined(AFX_DYWIN_H__1CA4A271_D4BA_4203_8185_AE473A10AEED__INCLUDED_)

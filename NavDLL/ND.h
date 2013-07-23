// ND.h: interface for the ND class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_ND_H__EE84284E_19A8_4072_8191_E3785790C8CA__INCLUDED_)
#define AFX_ND_H__EE84284E_19A8_4072_8191_E3785790C8CA__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include<math.h>
#include<vector>
#include "ObsArray.h"
#include "Reg.h"
using namespace std;

#define VFHsize 181

enum nregtype{NDHSWR=1,NDHSNR=2,NDHSGR=3,NDLS1=4,NDLS2=5,NDNONE=6,FALL=7};

struct ND_VALLEY {
int sr;//the rightest sector
int sl;
int ITVg;
double obsdis;
bool rise_r;
bool rise_l;
nregtype type;//1:HSWR,2:HSNR,3:HSGR,4:LS1,5:LS2
bool navigable;
double dx;
double dy;
double h;
};

class _declspec(dllexport) ND  
{
public:
	ND();
	virtual ~ND();

    int DegtoVFH(double th);
	double VFHtoDeg(int index);

	void ResetAll()
	{
     for(int i = 0; i<VFHsize; ++i)//���]interval
	 {
       intervals[i].dmax = D_MAX;
       intervals[i].regindex = -1;
	 }
	 gaps.clear();
	 regions.clear();

	}
	void ResetVFH()
	{
     for(int i = 0; i<VFHsize; ++i)//���]interval
	 {
       intervals[i].dmax = D_MAX;
       intervals[i].regindex = -1;
	 }
	}

	void ResetReg()
	{
	ResetVFH();
	 gaps.clear();
	 regions.clear();
	}

    void Set_Intervals(OBS* o);//���obstacle o, ��Ҧ�������interval���]�w
    void Set_All_Intervals(vector<OBS>& obsarr);//���obstacle o, ��Ҧ�������interval���]�w
	void Set_All_Intervals(vector<double>& range, vector<double>& rad);//���vector range, ��Ҧ�������interval���]�w
	void Find_Gap(double threshold);
    void Set_Intervals(double d,double deg);//���obstacle o, ��Ҧ�������interval���]�w
	void Find_Door(double lgx,double lgy,double radius);
	bool In_door(double x,double y);//�O�_�w���X���~
	double hval(double x,double y,double lgx,double lgy);//��Xhurestic
	void Find_REGION(double lgx,double lgy);
	void Save_Reg(ofstream& out);

	void Find_REGION();
	void Find_Door(double radius);
	double hval2(double x,double y,double lgx,double lgy);//��Xhurestic

public:
    double D_MAX;

	//velocity space interval
	ITV intervals[VFHsize];
    //Selected ITV
	int sr;
	int sl;

	//Regions
	vector <GAP> gaps;//�s1 ���0�P1������gap... �G�q0~interval_size
    vector <ND_VALLEY> regions;
};

#endif // !defined(AFX_ND_H__EE84284E_19A8_4072_8191_E3785790C8CA__INCLUDED_)

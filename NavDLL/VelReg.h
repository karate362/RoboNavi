// VelReg.h: interface for the VelReg class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_VELREG_H__330EE70E_7F71_44DB_AE46_858E58067103__INCLUDED_)
#define AFX_VELREG_H__330EE70E_7F71_44DB_AE46_858E58067103__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include<math.h>
#include<vector>
#include "ObsArray.h"
#include "Reg.h"
using namespace std;

#define ITVsize 181

enum vregtype{STOP=0,HSWR=1,HSNR=2,HSGR=3,LS1=4,LS2=5,BUG=6,NONE=7};

struct _declspec(dllexport) DWA_VALLEY {
int sr;//the rightest sector
int sl;
bool rise_r;
bool rise_l;
vregtype type;//1:HSWR,2:HSNR,3:HSGR,4:LS1,5:LS2

int ITVg;
bool navigable;
double v;
double w;
double reward_max;
};

class _declspec(dllexport) VelReg  
{
public:
	VelReg();
	virtual ~VelReg();

    int DegtoITV(double th);
    double ITVtoRad(int index);

	void ResetAll()
	{
     for(int i = 0; i<ITVsize; ++i)//重設interval
	 {
       intervals[i].dmax = D_MAX;
       intervals[i].regindex = -1;
	 }
	 gaps.clear();
	 regions.clear();

	}
    void Set_Intervals(OBS* o);//對於obstacle o, 對所有相關的interval做設定
	void Set_VFH(OBS* o);
    void Set_All_Intervals(vector<OBS>& obsarr);//對於obstacle o, 對所有相關的interval做設定
	void Find_Gap(double threshold);
	void Find_REGION(double lgx,double lgy);
	void Set_Situation(DWA_VALLEY& reg,double lgx,double lgy);

	inline void PutNewRegion(DWA_VALLEY& reg);

	//void SplitReg(DWA_VALLEY& reg,double lgx,double lgy);
	void SplitReg(DWA_VALLEY& reg);
	void Save_Reg(ofstream& out);

	double Regheading(double v,double w);

    double Regmargin(double v,double w,double dt);

public:
    double D_MAX;

	//velocity space interval
	ITV intervals[ITVsize];
    //Selected ITV
	int sr;
	int sl;

	//Regions
	vector <GAP> gaps;//存1 表示0與1中間有gap... 故從0~interval_size
    vector <DWA_VALLEY> regions;

};

#endif // !defined(AFX_VELREG_H__330EE70E_7F71_44DB_AE46_858E58067103__INCLUDED_)

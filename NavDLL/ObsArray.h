// ObsArray.h: interface for the ObsArray class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_OBSARRAY_H__8C897EBF_F109_41AD_BD2D_5258F25AC710__INCLUDED_)
#define AFX_OBSARRAY_H__8C897EBF_F109_41AD_BD2D_5258F25AC710__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <math.h>
#include <vector>
#include <fstream>
using namespace std;

struct OBS {
double x;
double y;
double r;
};

class _declspec(dllexport) ObsArray  
{
public:
	ObsArray();
	ObsArray(int max_r,int size,double enlarge_r);
	virtual ~ObsArray();
	void init(int max_r,int size,double enlarge_r);
	void init(ObsArray& initobs);
	void copyobs(ObsArray& initobs);
    void tran_obs(double dx, double dy, double dth);
    void tran_obs2(double nx, double ny, double nth);//根據新位置來轉換...
    //機器人有了移動(dx,dy,dth) 改動obs中的障礙物位置
    //void set_obs_sonar(double d,double deg,double ed);
    void set_obs_sonar(double x,double y,double ed);
    void set_obs_laser(double d,double deg,double ed);
	void set_obs_pose(double px,double py,double pth);

	void set_obs_laser_array(vector<double>& range, vector<double>& rad);

	void setLastReadingTime(DWORD rt){LastReadingTime = rt;}
	void SaveObs(ofstream& out);

public://attributes
	int MAX_sonar_range;
	int obsize;//OBS中的障礙物個數
    double robot_radius;
	double x;//robot X when last reading comes
	double y;//robot Y when last reading comes
	double th;//robot Th when last reading comes
	vector <OBS> obs;
	int write_count;//新增obs時由此index寫入
	DWORD LastReadingTime;

};

#endif // !defined(AFX_OBSARRAY_H__8C897EBF_F109_41AD_BD2D_5258F25AC710__INCLUDED_)

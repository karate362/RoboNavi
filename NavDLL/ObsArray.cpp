// ObsArray.cpp: implementation of the ObsArray class.
//
//////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "ObsArray.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

double SONARCONFIG[8] = {90,50,30,10,-10,-30,-50,-90};
double SONARCONFIGX[8] = {115,155,190,210,210,190,155,115};
double SONARCONFIGY[8] = {130,115,80,25,-25,-80,-115,-130};

ObsArray::ObsArray()
{

}

ObsArray::ObsArray(int max_r,int size,double enlarge_r)
{
    this->init(max_r,size,enlarge_r);
}

ObsArray::~ObsArray()
{

}

void ObsArray::init(int max_r,int size,double enlarge_r)
{
	int s = 0;
    MAX_sonar_range = max_r;
	obsize = size;//OBS中的障礙物個數
    robot_radius = enlarge_r;

    write_count=0;
	obs.resize(size);

	s = obs.size();
	for(int i=0;i<s;++i)
	{
		obs[i].x = 1000.0;
		obs[i].y = 1000.0;
		obs[i].r = 0;
	}

	x = 0;
	y = 0;
	th = 0;
	LastReadingTime = -1;
}

void ObsArray::init(ObsArray& initobs)
{
	this->init(initobs.MAX_sonar_range,initobs.obsize,initobs.robot_radius);
}

void ObsArray::copyobs(ObsArray& initobs)
{
	int i = 0;
	int s = initobs.obs.size();
	if(obs.capacity()<s)
		obs.reserve(s);

	obs.clear();
	for(i=0;i<s;++i)
		obs.push_back(initobs.obs[i]);

}

void ObsArray::set_obs_pose(double px,double py,double pth)
{
	x=px;
	y=py;
	th=pth;

}

void ObsArray::set_obs_laser_array(vector<double>& range, vector<double>& rad){//range: m
	OBS nobs;
	obs.clear();
	
	this->robot_radius = 0.23;

	int i=0;
	int s=range.size();

	for(i=0;i<s;++i){
		nobs.x = range[i]*cos(rad[i]);
		nobs.y = range[i]*sin(rad[i]);
		nobs.r = robot_radius + range[i]*0.01;
		obs.push_back(nobs);
	}
}

void ObsArray::set_obs_laser(double d,double deg,double ed)// d unit->mm, deg unit->deg
{
OBS nobs;
double th = deg*3.1415/180;
d = d/1000;
nobs.x = d*cos(th);
nobs.y = d*sin(th);
nobs.r = robot_radius+ed;
obs.push_back(nobs);
}

void ObsArray::set_obs_sonar(double x,double y,double ed)// d unit->mm, deg unit->deg
{
/*
OBS nobs;
nobs.x = x;
nobs.y = y;
nobs.r = robot_radius+ed;
obs.push_back(nobs);*/
//加上刪除動作
int s = obs.size();
int i = 0;
double th0 = atan2(y,x);
double th1 = 0;
double dis0 = sqrt(x*x+y*y);
double dis1 = 0;
double threshold1 = 5*3.1415/180.0;
double threshold2 = 0.3;

for(i=0;i<s;++i)
{
	th1 = atan2(obs[i].y,obs[i].x);
    dis1 = sqrt( obs[i].x*obs[i].x +obs[i].y*obs[i].y );

	if( fabs(th1-th0)<threshold1 && (dis0-dis1)>threshold2  )//刪除障礙
	{
		obs[i].x=1000.0;
		obs[i].y=1000.0;
		obs[i].r=0;
	}

	if( dis1 < obs[i].r )//刪除障礙
	{
		obs[i].r = dis1;
	}

}


obs[write_count].x = x;
obs[write_count].y = y;
obs[write_count].r = robot_radius+ed;
write_count = (write_count+1)%(this->obsize);
}


void ObsArray::tran_obs(double dx, double dy, double dth){//設原在(0,0,0), 移至(dx,dy,dth)後, 轉換障礙物位置

 int i = 0;
 int s = obs.size();
 OBS* obsp;//iterator

 double nx;
 double ny;

 //x' = R(-th)*(x-t)
 //R(th) = [cos(th)  -sin(th)]
 //        [sin(th)   cos(th)]



 for(i = 0; i<s; ++i){
    obsp = &(obs.front());
	obsp = obsp+i;
    nx = obsp->x - dx;
	ny = obsp->y - dy;
    obsp->x = cos(dth)*nx + sin(dth)*ny;
	obsp->y = cos(dth)*ny - sin(dth)*nx;


	}



}

void ObsArray::tran_obs2(double nx, double ny, double nth){//設原在(x,y,th), 移至(nx,ny,nth)後, 轉換障礙物位置

 int i = 0;
 int s = obs.size();
 OBS* obsp;//iterator

 double xb;
 double yb;

 double dx;
 double dy;
 double dth;

 //x' = R(-th)*(x-t)
 //R(th) = [cos(th)  -sin(th)]
 //        [sin(th)   cos(th)]

 ///////////對obs做矯正//////////
 dx = cos(th)*(nx - x) + sin(th)*(ny - y);
 dy = cos(th)*(ny - y) - sin(th)*(nx - x);
 dth = nth - th;

 x = nx;
 y = ny;
 th = nth;

 for(i = 0; i<s; ++i){
    obsp = &(obs.front());
	obsp = obsp+i;
    xb = obsp->x - dx;
	yb = obsp->y - dy;
    obsp->x = cos(dth)*xb + sin(dth)*yb;
	obsp->y = cos(dth)*yb - sin(dth)*xb;


	}
}


void ObsArray::SaveObs(ofstream& out)
{
 int i = 0;
 int s = obs.size();
 OBS* obsp;//iterator

 for(i = 0; i<s; ++i){
    obsp = &(obs.front());
	obsp = obsp+i;
	out<<obsp->x<<" "<<obsp->y<<" "<<obsp->r<<endl;
	}

}
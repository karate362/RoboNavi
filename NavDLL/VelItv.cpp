#include "StdAfx.h"
#include "VelItv.h"

using namespace std;

namespace RobotTra{

VelItv::VelItv(double ang_resolution,double dist_max)
{
	angrsl = ang_resolution;
	dmax = dist_max;

	ITV_size = (int)(180.0/angrsl) + 1;
	dist.resize(ITV_size);
	fill(dist.begin(),dist.end(),dmax);
}

VelItv::~VelItv(void)
{
}

inline int VelItv::AngtoIdx(double ang){

	return (int)((double)(ITV_size-1)*ang/PI + 0.5);
}

inline int VelItv::VeltoIdx(double v,double w){

	double rad = atan2(v,w);
	return (int)((double)(ITV_size-1)*rad/PI + 0.5); 

}


inline double VelItv::IdxtoRad(int i){
	
	return tan( ( (double)i/(double)(ITV_size-1)) * PI);
}


inline void VelItv::SetITV_Single(OBS& obs){

double &xo = obs.x;
double &yo = obs.y;
double &ro = obs.r;

double th1=0;
double th2=0;
double buf=0;//�p���K��

double R=0;
double th = 0;
double d;

int i1=0;
int i2=0;
int i=0;

//��X�v�T��intervals
buf = (xo*xo + yo*yo - ro*ro)/2;

buf = max(buf,0);

th1 = atan2( buf , (yo+ro) );//�|�b0~180����, �]buf���j��0
th2 = atan2( buf , (yo-ro) );

i1 = AngtoIdx(th1);
i2 = AngtoIdx(th2);

if(i2<i1)//�Oindex i1<i2
swap(i1,i2);

i1 = max(0,i1);
i2 = min(i2,ITV_size-1);

//�p��I���Z��
th = atan2( (xo*xo + yo*yo)/2 , yo );//velocity space�W������

if(th>89.5*PI/180 && th<90.5*PI/180)//�i�H�������u�F
{
	if(xo<=0)
		d = this->dmax;
	else
        d = max(xo - ro,0);
}
else//�ëD���u
{
	R=tan(th);//�^��b�|
	if(R>=0)
		th=atan2(xo,R-yo);
	else
		th=atan2(xo,yo-R);
	if(th<0)//�O�b0~360����
		th+=2*PI;

	d = max(fabs(R)*th - ro,0);
}

//Set Intervals
for(i=i1;i<=i2;++i)
dist[i] = min(dist[i],d);

}

void VelItv::SetITV_Multi(vector<OBS>& obsarr){

	fill(dist.begin(),dist.end(),dmax);
	int s = obsarr.size();
	for(int i=0;i<s;++i)
		this->SetITV_Single(obsarr[i]);
}

}
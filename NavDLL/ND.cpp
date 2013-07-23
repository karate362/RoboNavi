// ND.cpp: implementation of the ND class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ND.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ND::ND()
{
D_MAX = 80.0;
}

ND::~ND()
{

}

void ND::Save_Reg(ofstream& out)
{
int i;
int s = regions.size();
ND_VALLEY* pr;
for(i=0;i<s;++i)
{
	pr = &regions[i];
	out<<pr->sr<<' '<<pr->sl<<' '<<pr->type<<endl;
}

}

int ND::DegtoVFH(double th)//theta->VFH index, theta = 0~180.0
{
//size為n, 每份大小為180/(n-1), 總角度為180*n/(n-1)
//從-90-180/(2*(n-1))至90+180/(2*(n-1))
//index為k, 則佔有 180*k/(n-1)-90-180/(2*(n-1)) ~ 180*k/(n-1)-90+180/(2*(n-1))
int index = (int)((double)(VFHsize-1)*th/180.0 + 0.5);
	
return index;
}

double ND::VFHtoDeg(int index)
{

return (double)index*180.0/(double)(VFHsize-1) - 90.0;
}

void ND::Set_Intervals(double d,double deg)
{
	int index = DegtoVFH(deg+90.0);

	if(index>=0 && index<=VFHsize-1)//
	{
		if(intervals[index].dmax > d)
			intervals[index].dmax = d;
	}
}


void ND::Set_Intervals(OBS* o)//D_MAX: initial value
{
double &xo = o->x;
double &yo = o->y;
double &ro = o->r;

double th = 0;
double d = 0;
double dth = 0;
double buf=0;//計算方便用


int i1=0;
int i2=0;
int i=0;

d = sqrt(xo*xo + yo*yo) - ro;
if(d<0)
d=0;
if(d>D_MAX)
d = D_MAX;

th = atan2(yo,xo);//PI~-PI

if(d<ro)
dth = PI/2;
else
dth = asin(fabs(ro/d));

i1 = DegtoVFH((th + PI/2 - dth)*180.0/PI);
i2 = DegtoVFH((th + PI/2 + dth)*180.0/PI);

if(i1<0)
i1 = 0;
if(i2>=VFHsize)
i2 = VFHsize - 1;

//Set Intervals
for(i=i1;i<=i2;++i)
{
	if(d<intervals[i].dmax)
		intervals[i].dmax = d;
}

}


void ND::Set_All_Intervals(vector<OBS>& obsarr)
{
    vector<OBS> &obs = obsarr;
	int i=0;
	int s=obsarr.size();
	for(i=0;i<s;++i)
		Set_Intervals(&obs[i]);
}

void ND::Set_All_Intervals(vector<double>& range, vector<double>& rad)
{
	int i=0;
	int s=range.size();
	for(i=0;i<s;++i)
		Set_Intervals(range[i],rad[i]*180.0/3.1415);
}


void ND::Find_Gap(double threshold)
{

int i = 0;
	double dis = 0;
	GAP ng;

	gaps.clear();

	ng.i = 0;
	ng.max = true;//
	gaps.push_back(ng);

	for(i=1;i<VFHsize;++i)
	{
		dis = intervals[i].dmax - intervals[i-1].dmax;

        if(dis<0)
			ng.max = true;
		else
			ng.max = false;

		dis = fabs(dis);

		if( dis > threshold )
		{
			ng.i = i;
			gaps.push_back(ng);
		}
	}
	
	ng.i = VFHsize;
	ng.max = false;// 
	gaps.push_back(ng);

}

void ND::Find_REGION(){

int i = 0;
int j = 0;
int s = gaps.size();
int s2 = 0;

ND_VALLEY nv;
regions.clear();

GAP* gl;
GAP* gr;
for(i=1; i<s; ++i)//比較i和i-1之間
{
	gr = &gaps[i-1];
	gl = &gaps[i];

    nv.type = NDNONE;
	nv.rise_r = (bool)(1 - gr->max);
	nv.rise_l = gl->max;
	nv.sr = gr->i;
	nv.sl = gl->i - 1;//i從0~ITVsize 但第一個gap必定是 i = 0;應不用擔心index問題


	if(nv.rise_r || nv.rise_l )//有至少一邊是rising
	{  
		//if(abs(nv.sr-nv.sl) >= 3 ){ //wide enough
		
			s2=regions.size();
		for(j=nv.sr;j<=nv.sl;++j)
          intervals[j].regindex = s2;

		
		regions.push_back(nv);
		//}
	}
	

}

}


void ND::Find_REGION(double lgx,double lgy)
{
int i = 0;
int j = 0;
int s = gaps.size();
int s2 = 0;
double gth = atan2(lgy,lgx);
int VFHg = DegtoVFH((gth + PI/2)*180.0/PI);
double gdis = sqrt(lgx*lgx+lgy*lgy);

ND_VALLEY nv;
regions.clear();

GAP* gl;
GAP* gr;
for(i=1; i<s; ++i)//比較i和i-1之間
{
	gr = &gaps[i-1];
	gl = &gaps[i];

    nv.type = NDNONE;
	nv.rise_r = (bool)(1 - gr->max);
	nv.rise_l = gl->max;
	nv.sr = gr->i;
	nv.sl = gl->i - 1;//i從0~ITVsize 但第一個gap必定是 i = 0;應不用擔心index問題


	if(nv.sr<=VFHg && nv.sl>=VFHg)//GOAL IN REGION
    {
		if(intervals[VFHg].dmax >= gdis)
          nv.type = NDHSGR;
	}

	if(nv.rise_r || nv.rise_l || nv.type==NDHSGR)//有至少一邊是rising
	{  
		s2=regions.size();
		for(j=nv.sr;j<=nv.sl;++j)
          intervals[j].regindex = s2;
		regions.push_back(nv);
	}
	

}

}


void ND::Find_Door(double width)
{

double dth1,dis1;
double dth2,dis2;
double dwidth;//門寬
int i = 0;
int s = regions.size();
ND_VALLEY* vpt = 0;

double weight = 0;

/////找出開口來/////////////
for(i=0;i<s;++i)//決定 selected region
{

vpt = &(regions[i]);
dth1 = VFHtoDeg(vpt->sr)*PI/180.0;
dth2 = VFHtoDeg(vpt->sl)*PI/180.0;

////Door types

if(vpt->rise_l && vpt->rise_r){
	dis1 = intervals[vpt->sr - 1].dmax;
	dis2 = intervals[vpt->sl + 1].dmax;
	dth1 = VFHtoDeg(vpt->sr)*PI/180.0;
	dth2 = VFHtoDeg(vpt->sl)*PI/180.0;
}
else
if(vpt->rise_l){//left gap
	//dis1 = intervals[vpt->sl].dmax;
	dis2 = intervals[vpt->sl + 1].dmax;
	dis1 = dis2 + 0.5;
	dth1 = VFHtoDeg(vpt->sl)*PI/180.0;
	dth2 = VFHtoDeg(vpt->sl)*PI/180.0;
}
else{//right gap
	dis1 = intervals[vpt->sr - 1].dmax;
	//dis2 = intervals[vpt->sr].dmax;
	dis2 = dis1 + 0.5;
	dth1 = VFHtoDeg(vpt->sr)*PI/180.0;
	dth2 = VFHtoDeg(vpt->sr)*PI/180.0;

}


/*
if(vpt->sr == 0)//最邊緣
dis1 = intervals[vpt->sl + 1].dmax;
else{

	if(vpt->rise_r)
		dis1 = intervals[vpt->sr - 1].dmax;
	else
		dis1 = intervals[vpt->sr].dmax;
}

if(vpt->sl == VFHsize-1)//最邊緣
dis2 = intervals[vpt->sr - 1].dmax;
else{

	if(vpt->rise_l)
		dis2 = intervals[vpt->sl + 1].dmax;
	else
		dis2 = intervals[vpt->sl].dmax;
}
*/


//門寬是否可通過
dwidth = sqrt( dis1*dis1 + dis2*dis2  -2*dis1*dis2*cos(fabs(dth1-dth2)) );

if(dwidth>=width)
vpt->navigable = true;
else
vpt->navigable = false,vpt->type=FALL;

//門的位置

weight = 0.5;
vpt->dx = (1-weight)*dis1*cos(dth1) + weight*dis2*cos(dth2);
vpt->dy = (1-weight)*dis1*sin(dth1) + weight*dis2*sin(dth2);

/*
if(dis1<dis2){
vpt->dx = dis1*cos(dth1);
vpt->dy = dis1*sin(dth1);

}
else{
vpt->dx = dis2*cos(dth2);
vpt->dy = dis2*sin(dth2);

}*/

}
///////////////////////////

}


void ND::Find_Door(double lgx,double lgy,double width)
{


double gth = atan2(lgy,lgx);
int VFHg = DegtoVFH((gth + PI/2)*180.0/PI);
double gdis = sqrt(lgx*lgx+lgy*lgy);
double dth1,dis1;
double dth2,dis2;
double dwidth;//門寬
int i = 0;
int s = regions.size();
ND_VALLEY* vpt = 0;

double weight = 0;

/////找出開口來/////////////
for(i=0;i<s;++i)//決定 selected region
{

vpt = &(regions[i]);
dth1 = VFHtoDeg(vpt->sr)*PI/180.0;
dth2 = VFHtoDeg(vpt->sl)*PI/180.0;

if(vpt->sr == 0)//最邊緣
dis1 = intervals[vpt->sl + 1].dmax;
else
dis1 = intervals[vpt->sr - 1].dmax;

if(vpt->sl == VFHsize-1)//最邊緣
dis2 = intervals[vpt->sr - 1].dmax;
else
dis2 = intervals[vpt->sl + 1].dmax;

//門寬是否可通過
dwidth = sqrt( dis1*dis1 + dis2*dis2  -2*dis1*dis2*cos(fabs(dth1-dth2)) );

if(dwidth>=width)
vpt->navigable = true;
else
vpt->navigable = false,vpt->type=FALL;

//門的位置
weight = (double)(VFHg - vpt->sr)/fabs((double)(vpt->sl - vpt->sr));
weight = min(1.0,weight);
weight = max(0,weight);//限制在0~1之間
vpt->dx = (1-weight)*dis1*cos(dth1) + weight*dis2*cos(dth2);
vpt->dy = (1-weight)*dis1*sin(dth1) + weight*dis2*sin(dth2);

//決定到達終點之距離
vpt->h = sqrt((lgx-vpt->dx)*(lgx-vpt->dx) + (lgy-vpt->dy)*(lgy-vpt->dy));//從門到終點的距離

if(vpt->type == NDHSGR && gdis < vpt->h)//表示可以到達
{
		vpt->dx = lgx;
        vpt->dy = lgy;
		vpt->h = 0;
}

}
///////////////////////////


}

bool ND::In_door(double x,double y)
{

double th = atan2(y,x);
int VFHi = DegtoVFH((th + PI/2)*180.0/PI);//對應到的interval
double dist = sqrt(x*x+y*y);
int regi = 0;
ND_VALLEY* vpt = 0;

if(VFHi<0 || VFHi>VFHsize-1)//只考慮前方180度
return true;

if(dist > intervals[VFHi].dmax)//條件1 位在Laser打到的範圍之外
return false;

regi = intervals[VFHi].regindex;
if(regi>=0)
vpt = &regions[regi];
else
return true;

if(vpt->navigable && dist > sqrt( vpt->dx*vpt->dx + vpt->dx*vpt->dx ))
return false;//條件2 位於門外


return true;

}

double ND::hval(double x,double y,double lgx,double lgy)
{
double gd = sqrt( (lgx-x)*(lgx-x) + (lgy-y)*(lgy-y) );//直線距離
double hmin = -1;
double h;
int i=0;
int s=regions.size();
ND_VALLEY* vpt = 0;

if(!In_door(x,y))//已走出門外
return gd;

for(i=0;i<s;++i)
{
	vpt = &(regions[i]);
	
	if(vpt->navigable)//可通過
	{
		h = sqrt((vpt->dx-x)*(vpt->dx-x) + (vpt->dy-y)*(vpt->dy-y));//(x,y)到門的距離
		h += vpt->h;//加上門到終點的距離
		if(hmin<0)
			hmin = h;
		else
			hmin = min(h,hmin);
	}

}

if(hmin<0)//沒有可走的region
return gd;
else
return hmin;

}


double ND::hval2(double x,double y,double lgx,double lgy)// Goal point is not a constant
{
double gd = sqrt( (lgx-x)*(lgx-x) + (lgy-y)*(lgy-y) );//直線距離
double hmin = -1;
double h;
int i=0;
int s=regions.size();
ND_VALLEY* vpt = 0;

bool xin = In_door(x,y);
bool tin = In_door(lgx,lgy);

if(xin && tin)//已走出門外
return gd;

if(!xin && !tin)//已走出門外
return gd;


for(i=0;i<s;++i)
{
	vpt = &(regions[i]);
	
	if(vpt->navigable)//可通過
	{
		h = sqrt((vpt->dx-x)*(vpt->dx-x) + (vpt->dy-y)*(vpt->dy-y));//(x,y)到門的距離
		h += sqrt((lgx-vpt->dx)*(lgx-vpt->dx) + (lgy-vpt->dy)*(lgy-vpt->dy));//加上門到終點的距離
		if(hmin<0)
			hmin = h;
		else
			hmin = min(h,hmin);
	}

}

if(hmin<0)//沒有可走的region
return gd;
else
return hmin;

}
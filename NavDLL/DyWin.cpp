// DyWin.cpp: implementation of the DyWin class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "DyWin.h"



//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////



DyWin::DyWin()
{

a[0] = 1.2;
a[1] = 0.2;
a[2] = 0.2;
a[3] = 0.2;
a[4] = 0.05;

intervals = VR.intervals;
  }

DyWin::~DyWin()
{

}

void GPtoLP(double rx,double ry,double rth,double gx,double gy,double& lgx,double& lgy)
{
lgx = cos(rth)*(gx - rx) + sin(rth)*(gy - ry);
lgy = cos(rth)*(gy - ry) - sin(rth)*(gx - rx);

}

void DyWin::Region_Analysis(double threshold,double lgx,double lgy)
{
	int i=0;
	int s=0;
    //vector<OBS> &obs = *obspt;
    VR.ResetAll();
    VR.Set_All_Intervals(*obspt);
	VR.Find_Gap(threshold);
    VR.Find_REGION(lgx,lgy);
	s=VR.regions.size();
	for(i=0;i<s;++i)
		VR.Set_Situation(VR.regions[i],lgx,lgy);
}

void DyWin::init(ifstream& pars, ifstream& obsw,vector<OBS>* ob)
{
double vrange[2];
double wrange[2];
double sample_time,va,wa;
int obsize;
obsw>>a[0]>>a[1]>>a[2]>>a[3]>>a[4];
pars>>vrange[0]>>vrange[1]>>wrange[0]>>wrange[1]>>sample_time>>va>>wa>>obsize;
this->init(50,50,vrange[0],vrange[1],wrange[0],wrange[1],va,wa,sample_time,ob);

}

void DyWin::init(int ws, int hs, double vmax, double vmin, double wmax, double wmin, double vd, double wd,double dt, vector<OBS>* ob)
{
	obspt = ob;
    vector<OBS> &obs = *obspt;
	OBS nobs;
	nobs.x = 100;
	nobs.y = 100;
	nobs.r = 0;
	int i = 0;

	sw=ws;
	sh=hs;//size of reward space


	V_MAX = vmax;
    V_MIN = vmin;
	W_MAX = wmax;
    W_MIN = wmin;

	delta_v = (V_MAX-V_MIN)/(double)sh;
	delta_w = (W_MAX-W_MIN)/(double)sw;
    delta_t = dt;
	va = vd/delta_t;//加速度
	wa = wd/delta_t;//角加速度
	dw_width = vd;
	dw_height = wd;
	dh = (int)((double)sh * vd/(V_MAX-V_MIN));
    dw = (int)((double)sw * wd/(W_MAX-W_MIN));

    robot_radius = 0.3;

}

void DyWin::SaveITV(ofstream& outfile)
{
int i=0;
for(i=0;i<ITVsize;++i)
outfile<<intervals[i].dmax<<" "<<intervals[i].regindex<<endl;
}

int DyWin::DyWin_search(double v, double w, double rx,double ry,double rth,double gx,double gy, double& rv, double& rw)
{
double va = 1;
double vb = 1;
double wa = 0.5;
double wb = 0.5;//加速 減速

double hc = -1;
double dc = -1;
vc = v;
wc = w;

double vd;
double wd;
double reward = -100;//find max reward
double reward_max = -100;//find max reward

double f0 = -1;
double f1 = -1;
double f2 = -1;
double f3 = -1;
double f4 = -1;

double lgx;
double lgy;

vector<OBS> &obs = *obspt;

va = (V_MAX-V_MIN)*(double)(dh)/(double)(sh);


int di,dj;

int i1,i2;
int j1,j2;

double h = target_heading(rx,ry,rth,gx,gy);

int ITVi;
int REGi;


////是否在範圍內//////////////////
if(v > V_MAX) 
{
printf("out of range \n");
v = V_MAX;

}


if(v < V_MIN) 
{
printf("out of range \n");
v = V_MIN;

}

if(w > W_MAX) 
{
printf("out of range \n");
w = W_MAX;

}

if(w < W_MIN) 
{
printf("out of range \n");
w = W_MIN;

}
//////////////////////////////////


//////Interval setting//////////////////

GPtoLP(rx,ry,rth,gx,gy,lgx,lgy);//local position



//dynamic window中心
////決定dynamic window大小

dj = (int)( (double)(sh-1)*(v - V_MIN)/(V_MAX - V_MIN) );
di = (int)( (double)(sw-1)*(w - W_MIN)/(W_MAX - W_MIN) );

i1 = di - dw;
i2 = di + dw;
j1 = dj - dh;
j2 = dj + dh;

if(i1 < 0)
i1 = 0;
if(i2 >= sw)
i2 = sw-1;
if(j1 < 0)
j1 = 0;
if(j2 >= sh)
j2 = sh-1;

////對每一點計算reward value
for(di = i1; di <= i2; ++di)
for(dj = j1; dj <= j2; ++dj)
{
 vd = (double)dj*delta_v + V_MIN;
 wd = (double)di*delta_w + W_MIN;

 /////////////origin////////////////////////////////
 
 compute_obf(vd, wd, delta_t, lgx, lgy,v,w);
 //reward = a[0]*b[0]  + a[1]*b[1] + a[2]*b[2] + a[3]*b[3] + a[4]*b[4];
 reward = 1.2*b[0]  + 0.5*b[1] + 0.4*b[2] + 0*b[3] + 0*b[4];
 if(b[1] >= 0 )//確定不會碰撞
 {

 if(reward > reward_max)
 {
 reward_max = reward;
 rv = vd;
 rw = wd;

 f0 = b[0];
 f1 = b[1];
 f2 = b[2];
 f3 = b[3];
 f4 = b[4];
 }
 else
 if(reward == reward_max && fabs(wd)<fabs(rw))
 {
 reward_max = reward;
 rv = vd;
 rw = wd;
 }

 }

///////////////////////////////////////////////
}


if( reward_max < 0 )//WINDOW中所有速度均不可
{
rv = 0, rw = 0;

printf("NONE!!");


return 0;

}


return 1;


}


bool DyWin::NON_REG()
{
int s = VR.regions.size();
int i = 0;
int num = s; 

if(s == 0)//
return true;

if(VR.regions[0].sr == 0)//左轉
--num;

if(VR.regions[s-1].sl == ITVsize)//右轉
--num;

for(i=1;i<s-1;++i)
{
	if(VR.regions[i].type == STOP)
		--num;
}

if(num<=0)
return true;
else
return false;

}

int DyWin::DyWin_Reg_search(double v, double w,double lgx,double lgy,double Safety)//傳回policy
{
double va = 1;
double vb = 1;
double wa = 0.5;
double wb = 0.5;//加速 減速

double hc = -1;
double dc = -1;
vc = v;
wc = w;

double vd;
double wd;
double reward = -100;//find max reward
double reward_max = -100;//find max reward

double f0 = -1;
double f1 = -1;
double f2 = -1;
double f3 = -1;
double f4 = -1;


vector<OBS> &obs = *obspt;
DWA_VALLEY* REGd;

va = (V_MAX-V_MIN)*(double)(dh)/(double)(sh);


int di,dj;

int i1,i2;
int j1,j2;


int ITVi;
int REGi;


////是否在範圍內//////////////////
if(v > V_MAX) 
{
printf("out of range \n");
v = V_MAX;

}


if(v < V_MIN) 
{
printf("out of range \n");
v = V_MIN;

}

if(w > W_MAX) 
{
printf("out of range \n");
w = W_MAX;

}

if(w < W_MIN) 
{
printf("out of range \n");
w = W_MIN;

}


//dynamic window中心
////決定dynamic window大小

dj = (int)( (double)(sh-1)*(v - V_MIN)/(V_MAX - V_MIN) );
di = (int)( (double)(sw-1)*(w - W_MIN)/(W_MAX - W_MIN) );

i1 = di - dw;
i2 = di + dw;
j1 = dj - dh;
j2 = dj + dh;

if(i1 < 0)
i1 = 0;
if(i2 >= sw)
i2 = sw-1;
if(j1 < 0)
j1 = 0;
if(j2 >= sh)
j2 = sh-1;

double vs;
double ws;

DWA_VALLEY nv;
int ni = -1;
int i;
int s;

if(Low_Safety(Safety, 0, vs, ws))//低安全性
{
nv.type = BUG;
nv.v = vs;
nv.w = ws;
nv.navigable = true;
VR.regions.clear();
VR.regions.push_back(nv);
return 0;
}

ni = VR.regions.size();
nv.reward_max = 0;
nv.type = NONE;
nv.v = 0;
nv.w = 0;

s=VR.regions.size();//初始化

for(i=0;i<s;++i){
VR.regions[i].navigable = false;
VR.regions[i].reward_max = 0;
}

////對每一點計算reward value
for(di = i1; di <= i2; ++di)
for(dj = j1; dj <= j2; ++dj)
{
 vd = (double)dj*delta_v + V_MIN;
 wd = (double)di*delta_w + W_MIN;
 

///////////////////////Region/////////////////////////////////
 ITVi = VeltoITV(vd,wd);
 REGi = intervals[ITVi].regindex;

 if(REGi>=0)
 {
   Regcompute_obf(vd,wd,delta_t,v,w);
   reward = a[0]*b[0]  + a[1]*b[1] + a[2]*b[2] + a[3]*b[3] + a[4]*b[4];
 
   if(reward > VR.regions[REGi].reward_max)//要濾去沒有在DW內的region!
   {
	 VR.regions[REGi].reward_max = reward;
	 VR.regions[REGi].v = vd;
	 VR.regions[REGi].w = wd;
	 VR.regions[REGi].navigable = true;
   }
 }
 ///////////////////////////////////////////////////

}
/*
s=VR.regions.size();

for(i=0;i<s;++i)
{
	if(VR.regions[i].navigable == false)//有需停下才能進入的region
	{
		nv.reward_max = -100;
        nv.type = STOP;
        nv.v = 0;
        nv.w = 0;
		nv.navigable = true;
        VR.regions.push_back(nv);
		i = s;
	}
}*/


return 1;


}

DWA_VALLEY* DyWin::A_star(double lgx,double lgy,double dt)
{
DWA_VALLEY* vpt;
DWA_VALLEY* rv;
int i=0;
int s=VR.regions.size();
double px;
double py;

double hmin = 0;//原地不動
double h = 0;

vpt = &(VR.regions[0]);
rv = vpt;
px = (vpt->v)*dt*cos((vpt->w)*dt/2);
py = (vpt->v)*dt*sin((vpt->w)*dt/2);
hmin = sqrt((lgx-px)*(lgx-px) + (lgy-py)*(lgy-py));

for(i=1;i<s;++i)
{
vpt = &(VR.regions[i]);
px = (vpt->v)*dt*cos((vpt->w)*dt/2);
py = (vpt->v)*dt*sin((vpt->w)*dt/2);
h = sqrt((lgx-px)*(lgx-px) + (lgy-py)*(lgy-py));

if(h<hmin)
hmin=h, rv = vpt;
}

return rv;
}

double DyWin::target_heading(double rx,double ry,double rth,double gx,double gy){

double h;


h = atan2(gy-ry,gx-rx) - rth;

while(h > 3.1416)
h-=2*3.1416;

while(h < -3.1416)
h+=2*3.1416;


return h;
}


double DyWin::heading(double v,double w, double dt, double rx,double ry,double rth,double gx,double gy)
{

double px;
double py;
double pth;
double h;

px = rx + v*dt*cos(rth+w*dt/2);
py = ry + v*dt*sin(rth+w*dt/2);
pth = rth+w*dt;

h = atan2(gy-py,gx-px) - pth;

while(h > 3.1416)
h-=2*3.1416;

while(h < -3.1416)
h+=2*3.1416;

h = fabs(h);
h = fabs(180 - h*180/3.1416);

return h/180;
}


double DyWin::dist(double v,double w,double dt)
{
double dc = -1;
int ITVi = 0;
double vc = v;

ITVi = (int)(atan2(v,w)*180/PI + 0.5);//index in the intervals
dc = intervals[ITVi].dmax;

 if(vc>0)
 {

 if(v > sqrt(2*va*dc) || dc/vc<delta_t)//admissable velocity
 dc = -500;
 else
 if(dc/V_MAX < 1 )//碰撞時間
 dc = 0;
 else
 if(dc/V_MAX < 8)
 dc = (dc/V_MAX - 1)/8;
 else
 if(dc/V_MAX >=8)
 dc = 1;
 }
 else
 if(vc==0)
 dc = 1;

 return dc;
}

double DyWin::margin(double v,double w,double dt)
{

int i = 0;
int ITVi = 0;
int ri = -1;//robot右側margin
int li = -1;//robot左側margin 若到邊緣都OK 就設成是20吧

ITVi = (int)(atan2(v,w)*180/PI + 0.5);//index in the intervals

for(i = 1; i<=20; ++i)//檢查左右兩側 因為interval共有181個 我想到左右20都沒東西已經夠寬了吧
{
	if(ri == -1)//還未被設定才繼續
	{
		if(ITVi+i < ITVsize)
		{//
			if(intervals[ITVi+i].dmax <= v*dt)//會撞上
				ri = i; 
		}
		else
			ri = 20;//到最邊緣都沒東西
	}

	if(li == -1)//還未被設定才繼續
	{
		if(ITVi-i >= 0)
		{//
			if(intervals[ITVi-i].dmax <= v*dt)//會撞上
				li = i; 
		}
		else
			li = 20;//到最邊緣都沒東西
	}

}

if(ri == -1)
ri = 20;
if(li == -1)
li = 20;
//算出左右margin各有多少了...目前先取最小值吧
/*
if(ri>li)
return (double)(li*li)/400;
else
return (double)(ri*ri)/400;*/

  
return (double)(ri*li)/400;
}


double DyWin::margin2(double v,double w,double dt)
{
double dv = 0.05;
double dw = 0.1;
double nv = 0;
double nw  =0;
double ndist = 0;
double dval = 0;
double val = 0;
double valmax = 0;

for(int i=-2;i<=2;++i)
for(int j=-2;j<=2;++j){
	nv = v + dv*(double)i;
	nw = w + dw*(double)j;
	nv = max(nv,this->V_MIN);
	nv = min(nv,this->V_MAX);
	nw = max(nw,this->W_MIN);
	nw = min(nw,this->W_MAX);
	ndist = intervals[VeltoITV(nv,nw)].dmax;
  
	dval = (double)(3-abs(i)) * (double)(3-abs(j));

	if(ndist>3)//safe! 
		val += dval;
	valmax += dval;

}

///////////////////////Region/////////////////////////////////
 
return val/valmax;
}

void DyWin::compute_obf(double v, double w, double dt, double lgx,double lgy, double rv, double rw)//compute objective function
{

 b[0] = heading(v,w,dt,0,0,0,lgx,lgy);
 b[1] = dist(v,w,dt);
 b[2] = v/V_MAX;
 b[3] = margin(v,w,dt);
 //b[3] = VR.Regmargin(v,w,dt);
 b[4] = 0.5*(fabs(v-rv)/dw_width + fabs(w-rw)/dw_height);

}

void DyWin::Regcompute_obf(double v, double w, double dt, double rv, double rw)
{
 b[0] = VR.Regheading(v,w);
 b[1] = dist(v,w,dt);
 b[2] = v/V_MAX;
 b[3] = VR.Regmargin(v,w,dt);
 //b[3] = this->margin2(v,w,dt);
 //b[4] = 1 - fabs( PI/2 - atan2(v,w) )/(PI/2);
 //b[4] = 0.5*(fabs(v-rv)/dw_width + fabs(w-rw)/dw_height);
}



/////////////////////////////////BUG/////////////////////////////////////

bool DyWin::Low_Safety(double savety, double goal_ang, double& rv, double& rw)//環境在低安全性時傳回true
{
vector<OBS> &obs = *obspt;

double ds = 0;
double dmf = 100;
double dmb = 100;
int j = 0;
double thf = 0;
double thb = 0;
double dm;
double th;

int state = 0;//0: 前後均無, 1: 前有, 2:　後有,　3: 前後均有.

for(j = 0; j<obs.size(); ++j)//compute dm
{
  ds = sqrt( obs[j].x*obs[j].x + obs[j].y*obs[j].y ) - obs[j].r;

  if(ds < dmf && obs[j].x > 0)
  {
	  dmf = ds;
	  thf = atan2(obs[j].y,obs[j].x);
  }
  else
  if(ds < dmb && obs[j].x < 0)
  {
	  dmb = ds;
	  thb = atan2(obs[j].y,obs[j].x);
  }

}

if(dmf > savety && dmb >savety)
return 0;
else
if(dmf < savety && dmb >savety)
state = 1;
else
if(dmf > savety && dmb < savety)
state = 2;
else
state = 3;

if(state == 1)
{
dm = dmf;
th = thf;
}

if(state == 2)
{
dm = savety;
th = thb;
}

if(state == 3)
{
dm = dmf;
th = thf;
}

if(th < PI*(-0.5))
th = th + (PI);
else
if(th > PI*0.5)
th = th - (PI);
else
if(th < 0)
th = th + (PI/2);
else
if(th > 0)
th = th - (PI/2);


if(dm<savety)
{
rw = th;
rv = V_MAX*(dm/savety)*(1 - 2*fabs(th)/3.1415);

if(rv > V_MAX)
rv = V_MAX;
else
if(rv < V_MIN)
rv = V_MIN;


if(rw > W_MAX)
rw = W_MAX;
else
if(rw < W_MIN)
rw = W_MIN;

return 1;
}

return 0;

}

/////////////////////////////////BUG/////////////////////////////////////


#include "StdAfx.h"
#include <math.h>
#include "DWA.h"

using namespace std;

namespace RobotTra{

DWA::DWA(double* limits,VelItv& velitv)
{
	Vmax = limits[0];
	Vmin = limits[1];
	Wmax = limits[2];
	Wmin = limits[3];
	Vrsl = limits[6];
	Wrsl = limits[7];
	Vacc = limits[4];
    Wacc = limits[5];
	dt = limits[8];

	vitv = &velitv;

	int h = (int)((Vmax-Vmin)/Vrsl) + 1;
    int w =  (int)(Wmax/Wrsl);

	height = h;
	width = w*2 + 1;

	int i=0;
	int j=0;

	vels = new VEL*[height];

	for(i=0;i<height;++i)
		vels[i] = new VEL[width];

	for(i=0;i<h;++i)
		for(j=0;j<=w;++j){// initialize vels[i][w+j] & vels[i][w-j]
			vels[i][w-j].v = Vrsl*(double)i;
			vels[i][w-j].w = -1*Wrsl*(double)j;
			vels[i][w-j].i = vitv->VeltoIdx(vels[i][w-j].v,vels[i][w-j].w);
			vels[i][w-j].hi = i;
			vels[i][w-j].wi = w-j;

			vels[i][w+j].v = Vrsl*(double)i;
			vels[i][w+j].w = Wrsl*(double)j;
			vels[i][w+j].i = vitv->VeltoIdx(vels[i][w+j].v,vels[i][w+j].w);
			vels[i][w+j].hi = i;
			vels[i][w+j].wi = w+j;
		}

}

DWA::~DWA(void)
{
	for(int i=0;i<height;++i)
		delete[] vels[i];

}


double DWA::UnSafety(VEL &vel,double dv,double dw){
	double p = 0;
	VEL nvel;

	double vdel;
	double wdel;
	
	for(int i=0;i<height;++i)
		for(int j=0;j<width;++j){

			nvel = vels[i][j];
			vdel = fabs((nvel.v - vel.v)/dv);
			wdel = fabs((nvel.w - vel.w)/dw);
			
			if( !nvel.safe )
				if( vdel<1 && wdel<1 ) //in the search region
					p = max( p, exp(vdel*vdel*(-0.5))*exp(wdel*wdel*(-0.5)) );//0~1
				
		}

		return p;
}

bool DWA::Brakeable(VEL& vel){
	double& v = vel.v;
	double& w = vel.w;
	double d = vitv->getdist(vel.i);

	if( v > sqrt(2*Vacc*d) || 4*v*dt > d )
		return false;
	else
		return true;
}

bool DWA::Reachable(VEL &pvel, VEL &nvel){

	if( fabs(pvel.v-nvel.v)<Vacc*dt && fabs(pvel.w-nvel.w)<Wacc*dt)
		return true;
	else
		return false;

}

void DWA::Update_Vel_State(vector<OBS>& obsarr,double pv,double pw){

	int i=0;
	int j=0;
	int s=0;

	vitv->SetITV_Multi(obsarr);

	candidates.clear();

	VEL pvel;
	pvel.v = pv;
	pvel.w = pw;

	for(i=0;i<height;++i)
		for(j=0;j<width;++j){
			vels[i][j].safe = Brakeable(vels[i][j]);
			vels[i][j].reach = Reachable(pvel,vels[i][j]);

			if(vels[i][j].safe && vels[i][j].reach)
				candidates.push_back(vels[i][j]);
		}

	s = candidates.size();

	for(i=0;i<s;++i)
		candidates[i].unsafety = UnSafety(candidates[i],fabs(candidates[i].v) + 0.2, fabs(candidates[i].w*0.5) + 0.2);

}

}//namespace
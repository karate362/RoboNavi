// NaviDraw.cpp: implementation of the NaviDraw class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "PCtrl.h"
#include "NaviDraw.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

NaviDraw::NaviDraw()
{

}

NaviDraw::~NaviDraw()
{

}

void DrawRobotOnView(MFC_VIEWER& view,double v,double w,double gx,double gy,int policy,int ratio)
{
int r;
int g;
int b;
view.DrawCircle(view.Width()/2,view.Height()/2,(int)((double)ratio*0.2),1,0,0,0);
//注意若width > r 畫出的圓會以width為半徑...?
if(policy == 1)
r=0,g=255,b=0;
else
r=0,g=0,b=255;

if(w == 0 && v!=0)
view.DrawLine(view.Width()/2, view.Height(), view.Width()/2, 0, 2, r, g, b);
else
view.DrawCircle(view.Width()/2-(int)((double)ratio*v/w), view.Height()/2, (int)((double)ratio*fabs(v/w)), 2, r, g, b);

view.DrawLine(view.Width()/2,view.Height()/2,view.Width()/2-(int)(ratio*gy),view.Height()/2-(int)(ratio*gx),3,0,255,255);

}

void DrawOBSOnView(MFC_VIEWER& view,ObsArray& obsarr,int ratio)
{
    int i =0;
	int s = obsarr.obs.size();
    OBS* obsp;//iterator
    for(i = 0; i<s; ++i){
    obsp = &(obsarr.obs.front());
	obsp = obsp+i;
	view.DrawCircle(view.Width()/2-(int)((double)ratio*obsp->y),view.Height()/2-(int)((double)ratio*obsp->x),(int)((double)ratio*obsp->r)-1,2,255,0,0);
}
}

void DrawNDOnView(MFC_VIEWER& view,ND& nd,int size,int ratio)
{
int i = 0;
int j = 0;
int s = VFHsize;
double x;
double y;
double deg;
/*
for(i=0;i<s;++i)
{
	deg=nd.VFHtoDeg(i);
	x = (nd.intervals[i].dmax * cos(deg*PI/180));
	y = (nd.intervals[i].dmax * sin(deg*PI/180));
    view.DrawLine(view.Width()/2,view.Height()/2,view.Width()/2-(int)((double)ratio*y),view.Height()/2-(int)((double)ratio*x),1,0,0,0);
}
*/
s = nd.regions.size();

for(j=0;j<s;++j)
{

for(i=nd.regions[j].sr;i<nd.regions[j].sl;++i)
{
	deg=nd.VFHtoDeg(i);
	x = (double)ratio*(nd.intervals[i].dmax * cos(deg*PI/180));
	y = (double)ratio*(nd.intervals[i].dmax * sin(deg*PI/180));
    view.DrawLine(view.Width()/2,view.Height()/2,view.Width()/2-(int)(y),view.Height()/2-(int)(x),1,0,0,255);
	if(nd.regions[j].navigable)
	view.DrawCircle(view.Width()/2-(int)((double)ratio*nd.regions[j].dy),view.Height()/2-(int)((double)ratio*nd.regions[j].dx),(int)((double)ratio*0.5)-1,1,0,0,255);
}

}

}

void DrawDWATreeOnView(MFC_VIEWER& view,DWAstar& dstar,double rx,double ry,double rth,int ratio)
{
	int i=0;
	int s=dstar.DWAtree.size();
    double px;//parent node
	double py;
	double nx;//now node
	double ny;
	DWAnode* np;
	DWAnode* pp;

	for(i=s-1;i>0;--i)//index0必是initnode, 不用畫出
	{
		np = &(dstar.DWAtree[i]);//now node
		pp = &(dstar.DWAtree[dstar.DWAtree[i].parent]);//parent

		nx = (double)ratio*(cos(rth)*np->x - sin(rth)*np->y + rx);
		ny = (double)ratio*(sin(rth)*np->x + cos(rth)*np->y + ry);
	    px = (double)ratio*(cos(rth)*pp->x - sin(rth)*pp->y + rx);
		py = (double)ratio*(sin(rth)*pp->x + cos(rth)*pp->y + ry);

        view.DrawLine(view.Width()/2-(int)(ny),view.Height()/2-(int)(nx),view.Width()/2-(int)(py),view.Height()/2-(int)(px),1,0,0,255);

	}
}



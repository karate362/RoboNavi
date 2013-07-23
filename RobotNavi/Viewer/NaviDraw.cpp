// NaviDraw.cpp: implementation of the NaviDraw class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
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

s = nd.regions.size();

for(j=0;j<s;++j)
{
//view.DrawCircle(view.Width()/2-(int)((double)ratio*nd.regions[j].dy),view.Height()/2-(int)((double)ratio*nd.regions[j].dx),3,3,0,255,0);

for(i=nd.regions[j].sr;i<nd.regions[j].sl;++i)
{

	if(nd.regions[j].navigable){
	view.DrawCircle(view.Width()/2-(int)((double)ratio*nd.regions[j].dy),view.Height()/2-(int)((double)ratio*nd.regions[j].dx),10,4,0,255,0);
	deg=nd.VFHtoDeg(i);
	x = (double)ratio*(nd.intervals[i].dmax * cos(deg*PI/180));
	y = (double)ratio*(nd.intervals[i].dmax * sin(deg*PI/180));
    view.DrawLine(view.Width()/2,view.Height()/2,view.Width()/2-(int)(y),view.Height()/2-(int)(x),1,0,0,255);
	}
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
	GoalPoint ng;
	GoalPoint pg;

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

	for(i=1;i<dstar.BestPath.size();++i)//index0必是initnode, 不用畫出
	{
		np = &(dstar.BestPath[i-1]);//now node
		pp = &(dstar.BestPath[i]);//parent

		

		nx = (double)ratio*(cos(rth)*np->x - sin(rth)*np->y + rx);
		ny = (double)ratio*(sin(rth)*np->x + cos(rth)*np->y + ry);
	    px = (double)ratio*(cos(rth)*pp->x - sin(rth)*pp->y + rx);
		py = (double)ratio*(sin(rth)*pp->x + cos(rth)*pp->y + ry);

        view.DrawLine(view.Width()/2-(int)(ny),view.Height()/2-(int)(nx),view.Width()/2-(int)(py),view.Height()/2-(int)(px),2,255,0,0);

	}
	

	//Draw Target trajectory
	vector<GoalPoint>& TT = dstar.GoalTra;
	
	ng = TT[0];
	nx = (double)ratio*(cos(rth)*ng.x - sin(rth)*ng.y + rx);
	ny = (double)ratio*(sin(rth)*ng.x + cos(rth)*ng.y + ry);
	view.DrawCircle(view.Width()/2-(int)(ny),view.Height()/2-(int)(nx),10,2,0,255,0);
	for(i=1;i<TT.size();++i){
		ng = TT[i];
		pg = TT[i-1];
		nx = (double)ratio*(cos(rth)*ng.x - sin(rth)*ng.y + rx);
		ny = (double)ratio*(sin(rth)*ng.x + cos(rth)*ng.y + ry);
	    px = (double)ratio*(cos(rth)*pg.x - sin(rth)*pg.y + rx);
		py = (double)ratio*(sin(rth)*pg.x + cos(rth)*pg.y + ry);
		view.DrawLine(view.Width()/2-(int)(ny),view.Height()/2-(int)(nx),view.Width()/2-(int)(py),view.Height()/2-(int)(px),2,0,255,0);
	}

/*
	for(i=0;i<dstar.BestPath.size();++i)//
	{

		np = &(dstar.BestPath[i]);//parent

		if(TT.size()>=np->depth){
			pg = TT[np->depth];//now node
		

			nx = (double)ratio*(cos(rth)*np->x - sin(rth)*np->y + rx);
			ny = (double)ratio*(sin(rth)*np->x + cos(rth)*np->y + ry);
			px = (double)ratio*(cos(rth)*pg.x - sin(rth)*pg.y + rx);
			py = (double)ratio*(sin(rth)*pg.x + cos(rth)*pg.y + ry);

			view.DrawLine(view.Width()/2-(int)(ny),view.Height()/2-(int)(nx),view.Width()/2-(int)(py),view.Height()/2-(int)(px),2,0,0,0);

		}
	}
*/
}


void DrawVAsOnView(MFC_VIEWER& view,VisableArea& VA,double ratio){

	int i=0;
	double px;
	double py;
	double nx;
	double ny;
	double r;
	double a;

	VA.GetCenter(px,py);


	px = ratio*px;
	py = ratio*py;

	for(i=0;i<VITVsize;i+=4){
		VA.getITV(i,r,a);
		nx =  px + ratio*r*cos(a) ;
		ny =  py + ratio*r*sin(a);
		view.DrawLine(view.Width()/2-(int)(ny),view.Height()/2-(int)(nx),view.Width()/2-(int)(py),view.Height()/2-(int)(px),1,0,0,0);
	}
}


void DrawSensorReading(MFC_VIEWER& viewer, vector<double> &range, vector<double> &rad, double scale){

	double x;
	double y;

	viewer.DrawCircleT(0,0,(int)(0.25/scale),1,0,0,0);

	for(int i=0;i<range.size();++i){
		x = range[i]*cos(rad[i]);
		y = range[i]*sin(rad[i]);
		viewer.DrawCircleT((int)(x/scale),(int)(y/scale),1,2,255,0,0);

	}
}



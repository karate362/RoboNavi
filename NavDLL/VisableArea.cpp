#include "StdAfx.h"
#include "VisableArea.h"

VisableArea::VisableArea(void)
{
	this->ResetAll();
	cx=0;
	cy=0;
}

VisableArea::~VisableArea(void)
{
}


void VisableArea::setITV(int idx,double dist){
	if(idx < 0)
		idx += VITVsize;
	else
		if(idx >= VITVsize)
			idx -= VITVsize;

	ITVs[idx] = min(ITVs[idx],dist);
}

void VisableArea::getITV(int idx,double& r, double& a){

	if(idx < 0)
		idx += VITVsize;
	else
		if(idx >= VITVsize)
			idx -= VITVsize;

		r = ITVs[idx];
		a =  2*3.1415926 * ( (double)idx/(double)VITVsize - 0.5 );

}

bool VisableArea::GetVisableState(double rx,double ry,double rth,double& range, double& angle){// return true: visable, 
	double PI = 3.1415926;
	double dx;
	double dy;
	double dist;
	int idx;

	dx = rx - cx;
	dy = ry - cy;
	dist = sqrt(dx*dx + dy*dy);
	angle = atan2(dy,dx);
	idx = AngtoVFH(angle);
	getITV(idx,range,angle);

	angle = atan2(-1*dy,-1*dx) - rth;
	
	while(angle>=PI)
		angle = angle - 2*PI;

	while(angle<(-1)*PI)
		angle = angle + 2*PI;
	
	if(range < dist){//vision blocked
        range = dist;
		return false;
	}
	else{
        range = dist;
		return true;		
	}
}

void VisableArea::SetSingleVA(ObsArray& obsarr,double px,double py){// the target is in (px,py) 
	vector <OBS> &obs = obsarr.obs;
	int s = obs.size();
	double dx;
	double dy;
	double oth;
	double dist;
	double dth;
	int i;
	int j;
	int i1;
	int i2;

	this->ResetAll();
	cx = px;
	cy = py;



	for(int i=0;i<s;++i){
		dx = obs[i].x - cx;
		dy = obs[i].y - cy;
		dist = sqrt(dx*dx + dy*dy);
		dth = (obs[i].r-obsarr.robot_radius + 0.1)/dist;

		if(dth < 0)
			dth = 3.1415926;
		else{
			dth = min(1.0,dth);
			dth = asin(dth);
		}
		
		oth = atan2(dy,dx);
		i1 = AngtoVFH(oth - dth);
		i2 = AngtoVFH(oth + dth);

		for(j=i1;j<=i2;++j)
			setITV(j, dist);
	}
}

int VisableArea::AngtoVFH(double ang){//ang: -pi~-+pi

	double dth = 360.0/(double)VITVsize;
	double th = ang*180.0/3.1415926;
	return (int)( (th+180.0)/dth );

}

double VisableArea::Predict_Visbility(double rx,double ry,double radius){//1: visible, 0: not
int i;
int i1;
int i2;

double r;
double d;

double dx = rx - cx;
double dy = ry - cy;
double dist = sqrt(dx*dx+dy*dy);
double ang = atan2(dy,dx);
double dth = radius/dist;
dth = min(1.0,dth);
dth = max(0,dth);
dth = asin(dth);
i1 = AngtoVFH(ang - dth);
i2 = AngtoVFH(ang + dth);

d = dist - radius;

for(i=i1;i<=i2;++i){
	this->getITV(i,r,ang);
	if(r>d)
		return 1.0;
}

return 0;

}
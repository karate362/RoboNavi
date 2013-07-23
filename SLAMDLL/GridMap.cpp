#include "StdAfx.h"
#include "GridMap.h"
#include <math.h>
#include "IO_Functions.h"
//#include <omp.h>

using namespace Geom2D;
using namespace std;

const double GridMap::scale = 0.05;

GridMap::GridMap(double x,double y,double ang)
{
	memset(GMAP,0,Msize*8);
	MapPose.p.x=x;
	MapPose.p.y=y;
	MapPose.phi = ang;
}

GridMap::~GridMap(void)
{
}

inline void GridMap::GridToXY(int w, int h, double &x, double &y){

	Point p;
	p.x = (double)(w - this->width/2)*GridMap::scale;
	p.y = (double)(this->height/2 - h)*GridMap::scale;

	//R*dX + t
	PointTrans(p, MapPose);
	x = p.x;
	y = p.y;

}

void GridMap::XYToGrid(double x,double y,int& w,int &h){

	Point p;
	p.x = x;
	p.y = y;

	Transform2D t(MapPose);

	t.transform_to_relative(p);

	w = (int)round(p.x/GridMap::scale) + this->width/2;
	h = this->height/2 - (int)round(p.y/GridMap::scale); 

}


void GridMap::Find_Beam_Range(const vector<double>& range,const vector<double>& rad){


	double alpha = 0.03;//range uncertainty factor
	double beta = 1.0;//angle uncertainty factor

	int i=0;
	int s=0;

	s = range.size();

	if(drange.size() != s)
		drange.resize(s);

	for(i=0;i<s;++i)
		drange[i] = alpha*range[i] + 0.05;


	s = rad.size();

	if(drad.size() != s)
		drad.resize(s);

	drad[0] = beta * fabs(rad[0] - rad[1]);
	drad[s-1] = beta * fabs(rad[s-1] - rad[s-2]);

    s-=1;
	for(i=1;i<s;++i)
		drad[i] = beta * min( fabs(rad[i]-rad[i-1]) , fabs(rad[i]-rad[i+1]) );


}

double GridMap::inv_laser_model(Point gp, const Pose &rpose, const vector<double> &range, const vector<double> &rad){


	double lo = 0;
	double lfree = -2;
	double locc = 2;
	double l = lo;
	
	double r = 0;
	double phi = 0;

	double d = 0;
	double dmin = 0;
	int idx = 0;
	int i = 0;
	int s = rad.size();

	//Compute location to the robot
    Transform2D Rt(rpose);
    Rt.transform_to_relative(gp);
	r = sqrt( sqr(gp.x) + sqr(gp.y) );
	phi = atan2(gp.y,gp.x);

	 //Find corresponding beam
	dmin = fabs(phi-rad[0]);

	for(i=0;i<s;++i){

		d = fabs(phi-rad[i]);

		if(d<dmin){
			dmin = d;
			idx = i;
		}

	}
    
	//If in the beam range
	if(dmin < this->drad[idx] ){

		if( fabs(r-range[idx]) < this->drange[idx])//occupied
			l = locc;
		else
			if(r < range[idx])//free
				l = lfree;
	}

	
	return l-lo;
}



double GridMap::build_conspace(Point gp, const Pose &rpose, const vector<double> &range, const vector<double> &rad){


	double lo = 0;
	double lfree = -2;
	double locc = 2;
	double l = lo;
	
	double r = 0;
	double phi = 0;

	double d = 0;
	double dmin = 0;
	int idx = 0;
	int i = 0;
	int s = rad.size();

	Point lp;

	//Compute location to the robot
    Transform2D Rt(rpose);
    Rt.transform_to_relative(gp);
	r = sqrt( sqr(gp.x) + sqr(gp.y) );
	phi = atan2(gp.y,gp.x);

	 //Find corresponding beam
	
    l = lfree;

	for(i=0;i<s;++i){
        lp.x = range[i]*cos(rad[i]);
	    lp.y = range[i]*sin(rad[i]);

		if(dist(lp,gp)<0.3 )
			return locc-lo;

	}
    
	
	return l-lo;
}


void GridMap::Grid_Mapping(const Geom2D::Pose &gpose, const std::vector<double> &range, const std::vector<double> &rad){
//For each grid:

	int i=0;
	int j=0;
	Point gp;
	double grid;

	Find_Beam_Range(range,rad);//decide drange and drad

	Pose rpose = gpose;//based on map coordinate 


	for(i=0;i<this->width;++i)
		for(j=0;j<this->height;++j){

			GridToXY(i,j,gp.x,gp.y);
			grid = GMAP[getidx(i,j)];
			grid += inv_laser_model(gp,rpose,range,rad);

			grid = grid>-10 ? grid:-10;
			grid = grid<10 ? grid:10;

            GMAP[getidx(i,j)]=grid;

		}

}


/*
void GridMap::Grid_Mapping2(const Geom2D::Pose &gpose, const std::vector<double> &range, const std::vector<double> &rad){
//For each grid:

	int i=0;
	int j=0;
	Point gp;
	double grid;

	Pose rpose = gpose;//based on map coordinate 

	for(i=0;i<this->width;++i)
		for(j=0;j<this->height;++j){

			GridToXY(i,j,gp.x,gp.y);
			grid = GMAP[getidx(i,j)];
			grid += build_conspace(gp,rpose,range,rad);

			grid = grid>-10 ? grid:-10;
			grid = grid<10 ? grid:10;

            GMAP[getidx(i,j)]=grid;

		}

}
*/

double GridMap::set_conspace(Geom2D::Point lp){

    Point np;
	int w;
	int h;
	int del = (int)(0.3/this->scale);
	this->XYToGrid(lp.x,lp.y,w,h);

	for(int i=-1*del;i<=del;++i)
		for(int j=-1*del;j<=del;++j){
			if(this->InMAP(w+i,h+j)){
				this->GridToXY(w+i,h+j,np.x,np.y);

				if(dist(lp,np)<0.3)
					GMAP[getidx(w+i,h+j)] = 10;
			}
		}

	return 0;

}

void GridMap::Grid_Mapping2(const Geom2D::Pose &gpose, const std::vector<double> &range, const std::vector<double> &rad){
//For each grid:

	int i=0;
	int j=0;
	Point gp;
	Point lp;
	
	double grid;

	int s = range.size();

	Pose rpose = gpose;//based on map coordinate 
    Transform2D t(rpose);

	for(i=0;i<s;++i){
		lp.x = range[i]*cos(rad[i]);
		lp.y = range[i]*sin(rad[i]);
		t.transform_to_global(lp);
		set_conspace(lp);
	}

}


void GridMap::TransformMap(Geom2D::Pose np,GridMap& nmap){

	int i=0;
	int j=0;
    int ni = 0;
	int nj = 0;
	int nwidth = nmap.Width();
	int nheight = nmap.Height();
	Point gp;
	//Transform2D t(np);

	nmap.ResetMap();//totally unknown
	nmap.setPose(np);

	for(i=0;i<nwidth;++i)
		for(j=0;j<nheight;++j){//For each grid in this map, transform and copy it to nmap

			nmap.GridToXY(i,j,gp.x,gp.y);
			this->XYToGrid(gp.x,gp.y,ni,nj);

			if(this->InMAP(ni,nj)){
				nmap.setGrid(i,j,GMAP[getidx(ni,nj)]);
			}
            
		}

}



int GridMap::gridstate(int w, int h){//free:0, unknown:-1, occupied: 1, -2: out of the map

    if(w<0 || w>=width || h<0 || h>=height)
		return -2;

	int idx = getidx(w,h);

	double l = GMAP[idx];
	l = exp(l);
	l = l/(1+l);

	if(l>0.8)
		return 1;

	if(l<0.2)
		return 0;

	return -1;

}

void GridMap::FindNN(int w,int h,int dw,int dh){ //search in w+-dw, h+-dh

	int w1 = max(w-dw,0);
	int w2 = min(w+dw,width);
	int h1 = max(h-dh,0);
	int h2 = min(h+dh,height);
	int idx = getidx(w,h);
	double dist;
	Point nn;

	LMAP[idx] = 1000;
	

	for(int i=w1;i<w2;++i)
		for(int j=h1;j<h2;++j){//search range

			if( gridstate(i,j) == 1 ){//occupied
				dist = sqr((double)(i-w)) + sqr((double)(j-h));
				dist = scale*sqrt(dist);

				if(LMAP[idx]>dist){
					LMAP[idx] = dist;
					GridToXY(i,j,nn.x,nn.y);//Set nearest neighbor
					NPs[idx] = nn;
				}
			}

		}

}

void GridMap::LikelihoodMap(double dlim){

	int dw = (int)round(dlim/scale);//search range
	int dh = dw;
	int i=0;
	int j=0;

	for(i=0;i<this->width;++i)
		for(j=0;j<this->height;++j){
			FindNN(i,j, dw, dh);
		}
}


void GridMap::SaveGMAP(FILE* out){

	for(int j=0;j<height;++j){
		for(int i=0;i<width;++i)
			fprintf(out,"%.3f ",GMAP[getidx(i,j)]);
		fprintf(out,"\n");
	}

}

void GridMap::LoadGMAP(FILE* in){

	int w;
	int h;
	ReadDoubleMatrix(in,readbuf,w,h);
	copy(readbuf.begin(),readbuf.end(),GMAP);

}



double GridMap::inv_laser_model_DATMO(Point gp, const Pose &rpose, const vector<double> &range, const vector<double> &rad,const std::vector<int>& type){

    //type: 0: static, 1: moving, -1: unknown
	double lo = 0;
	double lfree = -2;
	double locc = 2;
	double l = lo;
	
	double r = 0;
	double phi = 0;

	double d = 0;
	double dmin = 0;
	int idx = 0;
	int i = 0;
	int s = rad.size();

	//Compute location to the robot
    Transform2D Rt(rpose);
    Rt.transform_to_relative(gp);
	r = sqrt( sqr(gp.x) + sqr(gp.y) );
	phi = atan2(gp.y,gp.x);

	 //Find corresponding beam
	dmin = fabs(phi-rad[0]);

	for(i=0;i<s;++i){

		d = fabs(phi-rad[i]);

		if(d<dmin){
			dmin = d;
			idx = i;
		}

	}
    
	//If in the beam range
	if(dmin < this->drad[idx] ){

		if( fabs(r-range[idx]) < this->drange[idx] ){  //occupied and this is a reading corresponds to static objects
			if(type[idx]==0)
			//if(type[idx]!=1)
			    l = locc;
			else	//how fast to lower the static occupancy probibility
				l = locc/3;
		}
		else
			if(r < range[idx])//free
				l = lfree;
			else
				l = lo;

	}

	
	return l-lo;
}

void GridMap::Grid_Mapping_DATMO(const Geom2D::Pose &gpose, const std::vector<double> &range, const std::vector<double> &rad,const std::vector<int>& type){
//For each grid:

	int i=0;
	int j=0;
	Point gp;
	double grid;

	Find_Beam_Range(range,rad);//decide drange and drad

	Pose rpose = gpose;//based on map coordinate 

//#pragma omp parallel for private(i, j )

	for(i=0;i<this->width;++i)
		for(j=0;j<this->height;++j){

			GridToXY(i,j,gp.x,gp.y);
			grid = GMAP[getidx(i,j)];
			grid += inv_laser_model_DATMO(gp,rpose,range,rad,type);

			grid = grid>-10 ? grid:-10;
			grid = grid<10 ? grid:10;

            GMAP[getidx(i,j)]=grid;

		}

}
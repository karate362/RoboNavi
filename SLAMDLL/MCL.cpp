#include "StdAfx.h"
#include "MCL.h"
#include <math.h>
#include <time.h>

using namespace std;
using namespace Geom2D;

inline void Normalize(vector<double>& w){//sum(w) = 1;

	int i = 0;
	int s = w.size();
	double sum=0;

	for(i=0;i<s;++i){
		sum+=w[i];
	}

	for(i=0;i<s;++i){
		w[i] = w[i]/sum;
	}

}

inline double NormPdf(double x){//mean 0, sigma 1

	return exp(sqr(x)*(-0.5)) / sqrt(2*Geom2D::PI);

}


double UniRand(){

	return (double)rand()/(double)RAND_MAX;
}

double NormRand(double mean,double sd){//return

	int i=0;

	double p = 0;

	for(i=0;i<12;++i)
		p += ( (double)rand()/(double)RAND_MAX - 0.5 );//(0,1)

	p = p*sd + mean;

	return p;

}


MCL::MCL(int pn,GridMap* gmap)
{		
	srand(time(0));//random

	map = gmap;
	N = pn;
	particles.resize(N);
	nparticles.resize(N);
	weighting.resize(N);
	pnum.resize(N);

	fill(weighting.begin(),weighting.end(),(1/(double)N));
	fill(pnum.begin(),pnum.end(),0);


}

MCL::~MCL(void)
{
}

void MCL::SetInitGuess(Geom2D::Pose mean, Geom2D::Pose plim, int n){

	N = n;
	particles.resize(N);
	nparticles.resize(N);
	weighting.resize(N);
	pnum.resize(N);
	fill(weighting.begin(),weighting.end(),(1/(double)N));
	fill(pnum.begin(),pnum.end(),0);

	for(int i=0;i<N;++i){
		particles[i].p.x = mean.p.x + (UniRand()-0.5)*plim.p.x;
		particles[i].p.y = mean.p.y + (UniRand()-0.5)*plim.p.y;
		particles[i].phi = mean.phi + (UniRand()-0.5)*plim.phi; 
	}


}

void MCL::ICP_correct(GridMap* gmap,Geom2D::Pose& rpose,const std::vector<double>& range,const std::vector<double>& rad){

	icpref.clear();
	icpdat.clear();

	//Find Matched point 
	int i=0;
    int s=range.size();

	double dist;
	double sigma_hit = 0.5;

	int w;
	int h;

	Point np;
	Pose dpose;
	Transform2D t(dpose);


	for(i=0;i<s;++i){//for each reading
		np.x = rpose.p.x + range[i]*cos(rad[i] + rpose.phi);
		np.y = rpose.p.y + range[i]*sin(rad[i] + rpose.phi);

		gmap->XYToGrid(np.x,np.y,w,h);

		if(gmap->gridstate(w,h) != -2){//grid in the map, Nearest Neighbor

			dist = gmap->getLdist(w,h);

			if(dist<sigma_hit){//limit
				icpdat.push_back(np);
				np = gmap->getNp(w,h);
				icpref.push_back(np);
			}

		}
	}

	if( icpdat.size() > range.size()/2){
	dpose = ficp.ICP_Least_Square(icpref,icpdat);//ref = R*dat + t --> mappose = R*rpose + t
	t.SetBase(dpose);
	t.transform_to_global(rpose);
	}
	

}


void MCL::Predict(const Geom2D::Pose dpose){

	double s1 = 0.5;
	double s2 = 0.5;
	double s3 = 0.5;

	int i = 0;
	int s = particles.size();

	Pose rdpose = dpose;

	Transform2D t(dpose);

	for(i=0;i<s;++i){//For any particle
		rdpose.p.x = NormRand(dpose.p.x, s1 * dpose.p.x);
		rdpose.p.y = NormRand(dpose.p.y, s2 * dpose.p.y);
		rdpose.phi = NormRand(dpose.phi, s3 * dpose.phi);

		t.SetBase(particles[i]);
		t.transform_to_global(rdpose);
		particles[i] = rdpose;

	}

}



double MCL::LikelihoodModel(GridMap* gmap,const Geom2D::Pose rpose,const std::vector<double>& range,const std::vector<double>& rad){

// find P(Z | rpose, gmap)
	int i=0;
    int s=range.size();

	double dist;
	double sigma_hit = 0.5;

	int w;
	int h;

	double p=0;


	gmap->XYToGrid(rpose.p.x,rpose.p.y,w,h);

	if(gmap->gridstate(w,h) == 1)//occupied
		return 0;



	for(i=0;i<s;i+=10){//for each reading  or  for some reading...

		gmap->XYToGrid(rpose.p.x + range[i]*cos(rad[i] + rpose.phi), rpose.p.y + range[i]*sin(rad[i] + rpose.phi),w,h);
		
		dist = gmap->getLdist(w,h);

		//p += NormPdf(dist/sigma_hit) ;
		p *= (NormPdf(dist/sigma_hit) + 0.001);

	}

	return p;

}

double MCL::FindLdist(std::vector<GridMap*> gmaps,double zx,double zy,Geom2D::Point &nn){

	//Find Ldist from a set of local maps

	double lmin = 1000;
	int i=0;
	int s=gmaps.size();

	int w;
	int h;

	for(i=0;i<s;++i){
		gmaps[i]->XYToGrid(zx,zy,w,h);
		lmin = min(gmaps[i]->getLdist(w,h),lmin);
	}

	return lmin;
}

double MCL::LikelihoodModel2(std::vector<GridMap*> &gmaps,const Geom2D::Pose rpose,const std::vector<double>& range,const std::vector<double>& rad){

	// find P(Z | rpose, gmap)
	int i=0;
    int s=range.size();

	double dist;
	double sigma_hit = 0.5;
	double zx=0;
	double zy=0;

	int w;
	int h;

	double p=0;

	Point nn;

	map->XYToGrid(rpose.p.x,rpose.p.y,w,h);

	if(map->gridstate(w,h) == 1)//occupied
		return 0;

	for(i=0;i<s;++i){//for each reading

		zx = rpose.p.x + range[i]*cos(rad[i] + rpose.phi);
		zy = rpose.p.y + range[i]*sin(rad[i] + rpose.phi);//Laser point
		
		dist = this->FindLdist(gmaps,zx,zy,nn);

		p += NormPdf(dist/sigma_hit) ;
		//p *= (NormPdf(dist/sigma_hit) + 0.001);

	}

	return p;
	
}


void MCL::FindRelatedMaps(const Geom2D::Pose &rpose,std::vector<GridMap*> &gmaps){
	int i=0;
	int s=localmaps.size();

	double dr;
	double dang;
	Pose mpose;

	gmaps.clear();

	for(i=0;i<s;++i){
		mpose = localmaps[i]->getPose();
		dr = dist(rpose.p,mpose.p);
		dang = atan2(mpose.p.y-rpose.p.y, mpose.p.x-rpose.p.x) - rpose.phi;

		while(dang > Geom2D::PI)
			dang -= 2*Geom2D::PI;

		while(dang < -1* Geom2D::PI)
			dang += 2*Geom2D::PI;


		if( dr<10 && fabs(dang)<Geom2D::PI/2)
			gmaps.push_back(localmaps[i]);

	}

}

void MCL::SetGridMapSet(std::vector<GridMap*> &gmaps){
	this->localmaps.resize(gmaps.size());
	copy(gmaps.begin(),gmaps.end(),localmaps.begin());
}

void MCL::Weighting(const std::vector<double>& range, const std::vector<double>& rad){
////(1)Pure MCL
////(2)ICP correction for every point
////(3)ICP correction for best point
	//And find the highest weighted node

	double wmax=0;
	maxidx = 0;

	int icpi = 0;

	vector<GridMap*> gmaps;

	for(int i=0; i<N; ++i){
		//ICP correct
		for(icpi=0;icpi<1;++icpi)
			this->ICP_correct(map,particles[i],range,rad);

		//this->FindRelatedMaps(particles[i],gmaps);//Find maps which are related with p[i]

		//weighting[i] = weighting[i] * LikelihoodModel(map,particles[i],range,rad);
		weighting[i] = weighting[i] * LikelihoodModel2(localmaps,particles[i],range,rad);

		if(weighting[i] > wmax)
			wmax = weighting[i],maxidx = i;

	}

	for(icpi=0;icpi<50;++icpi)
		this->ICP_correct(map,particles[maxidx],range,rad);
	
	
	Normalize(weighting);
}

void MCL::Resample(){

	//
	double c = weighting[0];
    double r = UniRand()/(double)N;
	double U = 0;

	int i=0;
	int m=0;

	fill(pnum.begin(),pnum.end(),0);

	for(m=0;m<N;++m){
		U = r + (double)(m)/(double)N;

		while(U>c){
			++i;
			c+=weighting[i];
		}
		++pnum[i];

	}

	m=0;//particle number

	for(i=0;i<N;++i){
		fill_n(nparticles.begin() + m,pnum[i],particles[i]);
		m += pnum[i];
	}

	copy(nparticles.begin(),nparticles.end(),particles.begin());
		

	//initialize weighting
	fill(weighting.begin(),weighting.end(),(1/(double)N));

}
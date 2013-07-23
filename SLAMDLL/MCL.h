#pragma once
#include <vector>
#include "geometry2D.hpp"
#include "GridMap.h"
#include "Filter_ICP.h"

class _declspec(dllexport) MCL
{
public:
	MCL(int pn,GridMap* gmap);
	~MCL(void);

	void SetInitGuess(Geom2D::Pose mean, Geom2D::Pose plim, int n);//uniform distribution
	void SetGridMap(GridMap* nmap){map = nmap;}
	void SetGridMapSet(std::vector<GridMap*> &gmaps);
	void Predict(const Geom2D::Pose dpose);	

	double LikelihoodModel(GridMap* gmap,const Geom2D::Pose rpose,const std::vector<double>& range,const std::vector<double>& rad);//Return likelihood model
	double LikelihoodModel2(std::vector<GridMap*> &gmaps,const Geom2D::Pose rpose,const std::vector<double>& range,const std::vector<double>& rad);
	double FindLdist(std::vector<GridMap*> gmaps,double zx,double zy,Geom2D::Point &nn);//Given a laser point, find its L dist and nearest obstacle.

	void FindRelatedMaps(const Geom2D::Pose &rpose,std::vector<GridMap*> &gmaps);//Find neighborhood map from localmaps
	void Weighting(const std::vector<double>& range,const std::vector<double>& rad);
	void Resample();

	void ICP_correct(GridMap* gmap, Geom2D::Pose& rpose,const std::vector<double>& range,const std::vector<double>& rad);//using icp to correct the pose, do only one time

	const Geom2D::Pose& getParticle(int i){return particles[i];}
	int size(){return N;}

	const Geom2D::Pose& getMaxiPartcle(){return particles[maxidx];}

private:

	int N;//Particle number
	std::vector<Geom2D::Pose> particles;
	std::vector<Geom2D::Pose> nparticles;
	std::vector<double> weighting;
	std::vector<int> pnum;
	GridMap* map;
	std::vector<GridMap*> localmaps;

	//ICP
	Geom2D::Filter_ICP ficp;
	std::vector<Geom2D::Point> icpref;
	std::vector<Geom2D::Point> icpdat;
	std::vector<double> dmin_R;
	std::vector<double> dmin_Z;

	//Result
	int maxidx;

};

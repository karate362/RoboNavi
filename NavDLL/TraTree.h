#pragma once

#include "DWA.h"
#include <vector>
#include<queue>
#include "ND.h"
#include "NavObj.h"
#include "VisableArea.h"


namespace RobotTra{

_declspec(dllexport) struct VELnode
{
	double x;
	double y;
	double th;
	double v;
	double w;
	double unsafety;
	double g;
	double h;
	int depth;
	int parent;
};

struct VHnode//HEAP node
{
	double f;
	int index;
};

class VHCmp{
public:
    bool operator () (const VHnode a,const VHnode b)
    {
        return a.f > b.f;
    }
};

void TranObsArr(std::vector<OBS> &obsarr,double dx,double dy,double dth);//robot: (0,0,0)->(dx,dy,dth), find obs change

double getp1(double p1);
double getp2(double p2);


double Setg1(VELnode& nd,void* datapt);//For navigation
double Seth1(VELnode& nd,void* datapt);

double Setg2(VELnode& nd,void* datapt);//For navigation, ND-distance
double Seth2(VELnode& nd,void* datapt);


double Setg3(VELnode& nd,void* datapt);//For tracking
double Seth3(VELnode& nd,void* datapt);

double Setg4(VELnode& nd,void* datapt);//For tracking, plus visibility
double Seth4(VELnode& nd,void* datapt);


double Setg5(VELnode& nd,void* datapt);//For tracking, visibility+ND distance
double Seth5(VELnode& nd,void* datapt);


class  TraTree;

void _declspec(dllexport) DoNavi(TraTree& dywstar,ObsArray& initobs, double& nv, double& nw, double lgx,double lgy);
void _declspec(dllexport) DoTracking(TraTree& dywstar,ObsArray& initobs, double& nv, double& nw, vector<GoalPoint>& TT);

class _declspec(dllexport) TraTree
{
friend double Setg1(VELnode& nd,void* datapt);
friend double Seth1(VELnode& nd,void* datapt);
friend double Setg2(VELnode& nd,void* datapt);
friend double Seth2(VELnode& nd,void* datapt);
friend double Setg3(VELnode& nd,void* datapt);
friend double Seth3(VELnode& nd,void* datapt);
friend double Setg4(VELnode& nd,void* datapt);
friend double Seth4(VELnode& nd,void* datapt);
friend double Setg5(VELnode& nd,void* datapt);
friend double Seth5(VELnode& nd,void* datapt);

friend void DoNavi(TraTree& dywstar,ObsArray& initobs, double& nv, double& nw, double lgx,double lgy);
friend void DoTracking(TraTree& dywstar,ObsArray& initobs, double& nv, double& nw, vector<GoalPoint>& TT);

public:
	TraTree(void);
	~TraTree(void);

	vector<VELnode> GetBestPath(){
		return bestpath;
	}

	vector<VELnode> GetTree(){
		return VelTree;
	}

	void OBSTrans(VELnode& bestnode,vector<OBS>& obsarr);
	void ExpandNewNodes(VHnode &bestnode,vector<OBS>& obsarr);
    void TraSearch(vector<OBS>& initobs,double& rv,double& rw, double lgx,double lgy,int MaximumNodes);//

	//void PureExpand(VELnode &nx);

    double (*Setg)(VELnode& nd,void* datapt);//Funcpt
	double (*Seth)(VELnode& nd,void* datapt);//Funcpt

	DWA* dwa;
//////////////////////////////// For cost computation
	ND* ndptr;//搜尋門時使用


/////////////////////////////////

	vector<GoalPoint> GoalTra;// specify the goal for each time point, size == max depth
	vector<VisableArea> VAs;// used to compute robot visability
	double alpha[5];

private:

	vector <VELnode> VelTree;
	priority_queue<VHnode, vector<VHnode>, VHCmp> VelHeap;
	vector <VELnode> bestpath;

	vector<OBS> tranobs;

	double smoothness;
	double aheadtime;

	VELnode goalnode;

	int msize;
	int mdepth;






};



}
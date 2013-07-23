// DWAstar.h: interface for the DWAstar class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_DWASTAR_H__2FA98BA0_9810_42D9_8DCE_379D40D01990__INCLUDED_)
#define AFX_DWASTAR_H__2FA98BA0_9810_42D9_8DCE_379D40D01990__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "DyWin.h"
#include "ND.h"
#include "VisableArea.h"
#include "NavObj.h"
#include <vector>
#include<queue>
#include <fstream>
using namespace std;

//_declspec(dllexport)
class DWAstar; 

struct DWAnode
{
	double x;
	double y;
	double th;
	double lgx;
	double lgy;
	double v;
	double w;
	double g;
	double h;
	int depth;
	int parent;
    vregtype type;
};

struct DHnode//HEAP node
{
	double f;
	int index;
};

class DHCmp{
public:
    bool operator () (const DHnode a,const DHnode b)
    {
        return a.f > b.f;
    }
};




double Setg1(DWAnode& nd, void* datapt);//計算g
double Setg2(DWAnode& nd, void* datapt);//計算g
double Setg3(DWAnode& nd, void* datapt);//計算g
double Setg4(DWAnode& nd, void* datapt);//計算g, incorparating door detection

double Seth1(DWAnode& nd, DWAnode& gd, void* datapt);//計算h
double Seth2(DWAnode& nd, DWAnode& gd, void* datapt);//計算h
double Seth3(DWAnode& nd, DWAnode& gd, void* datapt);//計算h
double Seth4(DWAnode& nd, DWAnode& gd, void* datapt);//計算h

void _declspec(dllexport) DoDWAstar(DWAstar& dywstar,ObsArray& initobs, double& nv, double& nw, double lgx,double lgy);
void _declspec(dllexport) DoDWAstar_Tracking(DWAstar& dywstar,ObsArray& initobs, double& nv, double& nw, vector<GoalPoint>& TT);


class _declspec(dllexport) DWAstar  
{
friend double Setg1(DWAnode& nd, void* datapt);//計算g
friend double Setg2(DWAnode& nd, void* datapt);//計算g
friend double Setg3(DWAnode& nd, void* datapt);//計算g
friend double Setg4(DWAnode& nd, void* datapt);//計算g

friend double Seth1(DWAnode& nd, DWAnode& gd, void* datapt);//計算h
friend double Seth2(DWAnode& nd, DWAnode& gd, void* datapt);//計算h
friend double Seth3(DWAnode& nd, DWAnode& gd, void* datapt);//計算h
friend double Seth4(DWAnode& nd, DWAnode& gd, void* datapt);//計算h

friend void DoDWAstar(DWAstar& dywstar,ObsArray& initobs, double& nv, double& nw, double lgx,double lgy);
friend void DoDWAstar_Tracking(DWAstar& dywstar,ObsArray& initobs, double& nv, double& nw, vector<GoalPoint>& TT);

public:
	DWAstar();
	virtual ~DWAstar();

	void initDWA(ifstream& pars, ifstream& obw);
	void initDWAstar(double at,double smooth, double safety, int max_node_num, int hf);

    int AstarExpand(DyWin& dywi,int pi,int dep,DWAnode& gd);//傳回node個數
    void DWAstarsearch(double& rv,double& rw,ObsArray& initobs, DWAnode& init_node, int depth,double lgx,double lgy);

	int AstarExpand_Tracking(DyWin& dywi,int pi,int dep,DWAnode& gd);//傳回node個數
    void DWAstarsearch_Tracking(double& rv,double& rw,ObsArray& initobs, DWAnode& init_node, int depth);

    double (*Setg)(DWAnode& nd, void* datapt);//計算g
	double (*Seth)(DWAnode& , DWAnode&, void* datapt );//函式指標

	void SaveTree(ofstream& out);

	double smoothness;//平滑度的參數
	double Safety;//給sonar預備用
	double p_dis;//
	double p_obser;
	double ahead_time;
    int max_expand;//最多expand的node數, 超過即停止

	vector<DWAnode> DWAtree;
	vector<DWAnode> BestPath;
    priority_queue<DHnode, vector<DHnode>, DHCmp> DWAHeap;
	ObsArray obs;
	DyWin dyw;
	ND* ndptr;//搜尋門時使用

	//
	vector<GoalPoint> GoalTra;// specify the goal for each time point, size == max depth
	vector<VisableArea> VAs;// used to compute robot visability


	int selected_node;
};




#endif // !defined(AFX_DWASTAR_H__2FA98BA0_9810_42D9_8DCE_379D40D01990__INCLUDED_)

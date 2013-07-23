// DWAstar.cpp: implementation of the DWAstar class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "DWAstar.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
ND Fnd;//Find door


DWAstar::DWAstar()
{
	VAs.reserve(10);
	this->ndptr = &Fnd;
}

DWAstar::~DWAstar()
{

}
ofstream outa("Atree.txt");


void DWAstar::initDWA(ifstream& pars, ifstream& obw)
{
double vrange[2];
double wrange[2];
double sample_time,va,wa;
int obsize;
obw>>dyw.a[0]>>dyw.a[1]>>dyw.a[2]>>dyw.a[3]>>dyw.a[4];
pars>>vrange[0]>>vrange[1]>>wrange[0]>>wrange[1]>>sample_time>>va>>wa>>obsize;
dyw.init(50,50,vrange[0],vrange[1],wrange[0],wrange[1],va,wa,sample_time,&(obs.obs));
}


void DWAstar::initDWAstar(double at,double sm, double sa, int ma, int hf)
{
	smoothness = sm;//平滑度的參數
	Safety = sa;//給sonar預備用
	ahead_time = at;
    max_expand = ma;//最多expand的node數, 超過即停止


	Setg = Setg1;


    if(hf == 1)
		Seth = Seth1;
	else
    if(hf == 2)
		Seth = Seth2;

}


int DWAstar::AstarExpand(DyWin& dywi,int pi,int parent_depth,DWAnode& gd)//Expand new nodes and push them into DWAtree
{
DWAnode nd;
DHnode nh;
vector <DWA_VALLEY>& regions = dywi.VR.regions;
int i=0;
int s=regions.size();
int rval = 0;

double x0 = DWAtree[pi].x;//parent的位置
double y0 = DWAtree[pi].y;
double th0 = DWAtree[pi].th;

double atime = 0;


if(parent_depth == 0)
atime = dywi.delta_t;
else
atime = ahead_time;


for(i=0;i<s;++i)
{
	if(regions[i].navigable)//確有v,w可走入
	{
    ++rval;
	nd.v = regions[i].v;
	nd.w = regions[i].w;
	nd.th = th0 + nd.w*atime;
	nd.type = regions[i].type;

	nd.x = x0 + nd.v*atime*cos(th0 + nd.w*atime/2);
	nd.y = y0 + nd.v*atime*sin(th0 + nd.w*atime/2);
	nd.parent = pi;
	nd.depth = parent_depth+1;

	////////////////////////////////////////////////////////
	Setg(nd,this);
	Seth(nd,gd,ndptr);
   ////////////////////////////////////////////////////////

	DWAtree.push_back(nd);
	nh.f=nd.g+nd.h;
	//
	nh.index=DWAtree.size()-1;//
	DWAHeap.push(nh);//
	}
}

return rval;
}


void DWAstar::DWAstarsearch(double& rv,double& rw, ObsArray& initobs, DWAnode& init_node, int depth,double lgx,double lgy)
{
int ni=0;

double pgx;
double pgy;

DHnode nh;
DWAnode nd;
DWAnode gd;

DWAnode* ndp;
int mdepth = 0;
int gi = 0;//標示最後找到的node
int counter = 0;//計算次數

///////////////initialization/////////////////////
gd.x=lgx;
gd.y=lgy;
init_node.g=0;
Seth(init_node,gd,ndptr);

DWAtree.clear();
DWAtree.push_back(init_node);

while (!DWAHeap.empty())//clear the heap
{
   DWAHeap.pop();
}
nh.index = 0;
nh.f = init_node.g + init_node.h;
DWAHeap.push(nh);


///////////////initialization/////////////////////


while(!DWAHeap.empty())//找到深度為depth的node為止... 會有在那之前就讓DWAHeap空掉的情形嗎?
{
ni = (DWAHeap.top()).index;
ndp = &DWAtree[ni];//從此點開始expand


if(DWAtree[ni].depth >= depth || counter > max_expand){//滿足goal condition: 若已找到設定depth深度的點　則break
	gi = ni;
	break;
}

if( sqrt(pow(DWAtree[ni].x-lgx,2)+pow(DWAtree[ni].y-lgy,2)) <=0.5 ){//滿足goal condition: 已夠接近
	gi = ni;
	break;
}

obs.copyobs(initobs);//複製obs
obs.tran_obs(ndp->x,ndp->y,ndp->th);//映射

//Find Local goal for this point
GPtoLP(ndp->x,ndp->y,ndp->th,lgx,lgy,pgx,pgy);//pgx,pgy為robot在DWAtree[j]處所見之goal
gd.x=pgx;
gd.y=pgy;

dyw.Region_Analysis(0.5,pgx,pgy);
dyw.DyWin_Reg_search(ndp->v,ndp->w,pgx,pgy,Safety);//已找出所有候補velocity

DWAHeap.pop();//

AstarExpand(dyw,ni,ndp->depth,gd);//
++counter;

}

ni = gi;
selected_node = gi;
BestPath.clear();
BestPath.push_back(DWAtree[ni]);
while(DWAtree[ni].depth > 1)
{
	ni = DWAtree[ni].parent;
	BestPath.push_back(DWAtree[ni]);
}

rv = DWAtree[ni].v;
rw = DWAtree[ni].w;
}


double Setg1(DWAnode& nd, void* datapt)//計算g
{
DWAstar* dstar = (DWAstar*)datapt;
DyWin& dyw = dstar->dyw;

DWAnode& pd = dstar->DWAtree[nd.parent];//parent的g再加上...
double dv = (fabs((nd.v - pd.v)/dyw.V_MAX) + fabs((nd.w - pd.w)/dyw.W_MAX))*0.5;

nd.g = pd.g + dstar->ahead_time + dstar->smoothness*dv;// cost以"時間"來表示

return nd.g;
}

double Setg2(DWAnode& nd, void* datapt)//計算g
{
DWAstar* dstar = (DWAstar*)datapt;
DyWin& dyw = dstar->dyw;

DWAnode& pd = dstar->DWAtree[nd.parent];//parent的g再加上...
double dv = (fabs((nd.v - pd.v)/dyw.V_MAX) + fabs((nd.w - pd.w)/dyw.W_MAX))*0.5;

double tdif = sqrt( (nd.x-nd.lgx)*(nd.x-nd.lgx) + (nd.y-nd.lgy)*(nd.y-nd.lgy) );

double adif = atan2(nd.lgy-nd.y,nd.lgx-nd.x) - nd.th ;//it should be in pi/2~-pi/2
double p1 = 0;
double p2 = 0;

p1 = (8-tdif)/6;  // 2~8m --> score: 1~0
p1 = max(0,p1);
p1 = min(1,p1);

double obser = 0;//probability of observing the target

nd.g = pd.g  + dstar->smoothness*dv + 10*(1-p1);// cost以"時間"來表示

return nd.g;
}


double Setg3(DWAnode& nd, void* datapt)//計算g
{
DWAstar* dstar = (DWAstar*)datapt;
DyWin& dyw = dstar->dyw;
VisableArea& VA = dstar->VAs[nd.depth];

double tdif;
double adif;
double p1 = 0;
double p2 = 0;
double obser;

//double discount = pow(0.9,(double)nd.depth);

DWAnode& pd = dstar->DWAtree[nd.parent];//parent的g再加上...
double dv = (fabs((nd.v - pd.v)/dyw.V_MAX) + fabs((nd.w - pd.w)/dyw.W_MAX))*0.5;

if(VA.GetVisableState(nd.x,nd.y,nd.th,tdif,adif)){

while(adif>PI)
adif = adif - 2*PI;

while(adif<(-1)*PI)
adif = adif + 2*PI;

p2 = (PI*(1/2) - fabs(adif) )/( PI*(1/4) ); //1/8 PI ~ 3/8 PI --> score: 1~0
p2 = max(0,p2);
p2 = min(1,p2);

obser = (1-p2);

}
else
obser = 1.5;//high cost!!

p1 = (8-tdif)/6;  // 2~8m --> score: 1~0
p1 = max(0,p1);
p1 = min(1,p1);

nd.g = pd.g  + dstar->smoothness*dv + ( dstar->p_dis*(1-p1) + dstar->p_obser*obser );

return nd.g;
}



double Setg4(DWAnode& nd, void* datapt)//計算g door detection
{
DWAstar* dstar = (DWAstar*)datapt;
DyWin& dyw = dstar->dyw;
VisableArea& VA = dstar->VAs[nd.depth];
ND* ndp = dstar->ndptr;


double tdif;
double adif;
double p1 = 0;
double p2 = 0;
double obser;

//double discount = pow(0.9,(double)nd.depth);

DWAnode& pd = dstar->DWAtree[nd.parent];//parent的g再加上...
double dv = (fabs((nd.v - pd.v)/dyw.V_MAX) + fabs((nd.w - pd.w)/dyw.W_MAX))*0.5;


if(VA.GetVisableState(nd.x,nd.y,nd.th,tdif,adif)){//There are straight paths

while(adif>PI)
adif = adif - 2*PI;

while(adif<(-1)*PI)
adif = adif + 2*PI;


p2 = (PI*(1/2) - fabs(adif) )/( PI*(1/2) ); //1/8 PI ~ 3/8 PI --> score: 1~0
p2 = max(-0.5,p2);
p2 = min(1.0,p2);
/*
if(fabs(adif) > PI*(1/2) )
p2 = 0;
else
p2 = 1;*/

obser = (1.0-p2);
//obser = 0;

}
else{//no straight path
obser = 1.5;//high cost!!
tdif = ndp->hval2(nd.x,nd.y,nd.lgx,nd.lgy);
}

p1 = (8-tdif)/6;  // 2~8m --> score: 1~0
p1 = max(0,p1);
p1 = min(1,p1);

nd.g = pd.g  + dstar->smoothness*dv + ( dstar->p_dis*(1-p1) + dstar->p_obser*obser );

return nd.g;
}


double Seth1(DWAnode& nd,DWAnode& gd,void* datapt )//計算h
{
double dx=0;
double dy=0;
double R=0;
double dist=0;
ND* ndp = (ND*)datapt;

dx = (gd.x - nd.x);
dy = (gd.y - nd.y);
nd.h = (sqrt(dx*dx + dy*dy)/0.5);


return nd.h;
}

double Seth2(DWAnode& nd,DWAnode& gd,void* datapt )//計算h-2
{
double dx=0;
double dy=0;
double R=0;
double dist=0;
ND* ndp = (ND*)datapt;

nd.h = ndp->hval(nd.x,nd.y,gd.x,gd.y)/0.5;

return nd.h;
}

double Seth3(DWAnode& nd,DWAnode& gd,void* datapt )//計算h-2
{
	DWAstar* dstar = (DWAstar*)datapt;
	
	double discount = pow(0.9,(double)nd.depth);
	int i = nd.depth+1;
	double lgx;
	double lgy;
	
	double p1 = 0;
	double dtime = dstar->ahead_time;
	nd.h = 0;

	for(i = nd.depth+1;i<dstar->GoalTra.size();++i){
	lgx = dstar->GoalTra[i].x;
	lgy = dstar->GoalTra[i].y;

	p1 = sqrt( (nd.x-lgx)*(nd.x-lgx) + (nd.y-lgy)*(nd.y-lgy) ) - dtime*(dstar->dyw.V_MAX);
	p1 = (8-p1)/6;  // 2~8m --> score: 1~0
	p1 = max(0,p1);
	p1 = min(1,p1);

	discount = discount*0.9;
    nd.h += discount*10*(1-p1); 
	dtime += dstar->ahead_time;

	}

	nd.h = 0;

	return nd.h;
}

double Seth4(DWAnode& nd,DWAnode& gd,void* datapt )//計算h-2
{
DWAstar* dstar = (DWAstar*)datapt;
ND* ndp = dstar->ndptr;

int i = nd.depth+1;
double lgx;
double lgy;
double dist;
double p1 = 0;
double obser = 0;
double dtime = dstar->ahead_time;
double discount = pow(0.9,(double)nd.depth);

nd.h = 0;

for(i = nd.depth+1;i<dstar->GoalTra.size();++i){
	lgx = dstar->GoalTra[i].x;
	lgy = dstar->GoalTra[i].y;

	if( !dstar->VAs[i].GetVisableState(nd.x,nd.y,nd.th,dist,p1) ){//There are no straight paths
		dist = ndp->hval2(nd.x,nd.y,lgx,lgy);
	}


	p1 = dist - dtime*(dstar->dyw.V_MAX);	
	p1 = (8-p1)/6;  // 2~8m --> score: 1~0
	p1 = max(0,p1);
	p1 = min(1,p1);

	discount = discount*0.9;

	obser = 1.0 - dstar->VAs[i].Predict_Visbility(nd.x,nd.y,dtime*(dstar->dyw.V_MAX));
    nd.h += ( dstar->p_dis*(1-p1) + dstar->p_obser*obser ); 
	dtime += dstar->ahead_time;
}

return nd.h;
}


void DWAstar::SaveTree(ofstream& out)
{
	int i=0;
	int d=0;
	int s=DWAtree.size();
	char outstr[64];

    int ni = selected_node;

    while(DWAtree[ni].depth >= 1)
	{
    sprintf(outstr,"%d,%f,%f,%f,%f ",DWAtree[ni].depth,DWAtree[ni].x,DWAtree[ni].y,DWAtree[ni].v,DWAtree[ni].w);
	out<<outstr<<endl;
	ni = DWAtree[ni].parent;
	}

}





void DoDWAstar(DWAstar& dywstar, ObsArray& initobs, double& vi, double& wi, double lgx,double lgy){//initobs: (0,0,0)

	DWAnode nownode;
	dywstar.ndptr = &Fnd;

	/////ND FIND DOOR///////
	//Fnd.ResetReg();
	//Fnd.Set_All_Intervals(initobs);
	Fnd.Find_Gap(0.8);
	Fnd.Find_REGION(lgx,lgy);
	Fnd.Find_Door(lgx,lgy,0.5);
    /////ND FIND DOOR////////
	dywstar.Setg = Setg1;
    dywstar.Seth = Seth2;

	nownode.v = vi;
    nownode.w = wi;
    nownode.x = 0;
    nownode.y = 0;
    nownode.th = 0;
    nownode.parent = -1;
    nownode.depth = 0;
	
	dywstar.DWAstarsearch(vi,wi,initobs,nownode,10,lgx,lgy);

   //dywstar.obs.copyobs(initobs);//複製obs
    //dywstar.dyw.Region_Analysis(0.5,lgx,lgy);

}








int DWAstar::AstarExpand_Tracking(DyWin& dywi,int pi,int parent_depth,DWAnode& gd)//Expand new nodes and push them into DWAtree
{
DWAnode nd;
DHnode nh;
vector <DWA_VALLEY>& regions = dywi.VR.regions;
int i=0;
int s=regions.size();
int rval = 0;

double x0 = DWAtree[pi].x;//parent的位置
double y0 = DWAtree[pi].y;
double th0 = DWAtree[pi].th;

double atime = 0;


if(parent_depth == 0)
atime = dywi.delta_t;
else
atime = ahead_time;


for(i=0;i<s;++i)
{
	if(regions[i].navigable)//確有v,w可走入
	{
    ++rval;
	nd.v = regions[i].v;
	nd.w = regions[i].w;
	nd.th = th0 + nd.w*atime;
	nd.type = regions[i].type;

	nd.x = x0 + nd.v*atime*cos(th0 + nd.w*atime/2);
	nd.y = y0 + nd.v*atime*sin(th0 + nd.w*atime/2);
	nd.parent = pi;
	nd.depth = parent_depth+1;
	nd.lgx = GoalTra[nd.depth].x;
	nd.lgy = GoalTra[nd.depth].y;
	////////////////////////////////////////////////////////
	Setg(nd,this);
	Seth(nd,gd,this);
   ////////////////////////////////////////////////////////

	DWAtree.push_back(nd);
	nh.f=nd.g+nd.h;
	//
	nh.index=DWAtree.size()-1;//
	DWAHeap.push(nh);//
	}
}

return rval;
}


void DWAstar::DWAstarsearch_Tracking(double& rv,double& rw, ObsArray& initobs, DWAnode& init_node, int depth)
{
int ni=0;

double lgx;
double lgy;

double pgx;
double pgy;

DHnode nh;
DWAnode nd;
DWAnode gd;
DWAnode deepestnode;

DWAnode* ndp;
int mdepth = 0;
int gi = 0;//標示最後找到的node
int counter = 0;//計算次數

///////////////initialization/////////////////////

//Setg(init_node,this);
init_node.g = 0;
Seth(init_node,gd,this);

DWAtree.clear();
DWAtree.push_back(init_node);

while (!DWAHeap.empty())//clear the heap
{
   DWAHeap.pop();
}
nh.index = 0;
nh.f = init_node.g + init_node.h;
DWAHeap.push(nh);

deepestnode = init_node;

///////////////initialization/////////////////////


while(!DWAHeap.empty())//找到深度為depth的node為止... 會有在那之前就讓DWAHeap空掉的情形嗎?
{
ni = (DWAHeap.top()).index;
ndp = &DWAtree[ni];//從此點開始expand
gi = ni;

if(ndp->depth > deepestnode.depth)
deepestnode = *ndp;

if(DWAtree[ni].depth >= depth || counter > max_expand){//滿足goal condition: 若已找到設定depth深度的點　則break
	gi = ni;
	break;
}


lgx = GoalTra[ndp->depth+1].x;
lgy = GoalTra[ndp->depth+1].y;//Next goal

obs.copyobs(initobs);//複製obs
obs.tran_obs(ndp->x,ndp->y,ndp->th);//映射

//Find Local goal for this point
GPtoLP(ndp->x,ndp->y,ndp->th,lgx,lgy,pgx,pgy);//pgx,pgy為robot在DWAtree[j]處所見之goal
gd.x=pgx;
gd.y=pgy;

dyw.Region_Analysis(0.5,pgx,pgy);
dyw.DyWin_Reg_search(ndp->v,ndp->w,pgx,pgy,Safety);//已找出所有候補velocity

DWAHeap.pop();//

AstarExpand_Tracking(dyw,ni,ndp->depth,gd);//
++counter;

}


//ni = gi;
//selected_node = gi;

BestPath.clear();
BestPath.push_back(deepestnode);
ni = deepestnode.parent;

while(DWAtree[ni].depth > 1)
{   
	BestPath.push_back(DWAtree[ni]);
	ni = DWAtree[ni].parent;
}
BestPath.push_back(DWAtree[ni]);

rv = DWAtree[ni].v;
rw = DWAtree[ni].w;

}



void DoDWAstar_Tracking(DWAstar& dywstar,ObsArray& initobs, double& vi, double& wi, vector<GoalPoint>& TT){

	DWAnode nownode;
	dywstar.ndptr = &Fnd;


//Set Goal trajectory
	int i = 0;
	int j = 0;
	double t = 0;
	double dt = 0;
	double dtmin = 0;
	int depth = 10;
	GoalPoint ngp;
	vector<GoalPoint>& GoalTra = dywstar.GoalTra;
	vector<VisableArea>& VAs = dywstar.VAs;
	GoalTra.clear();

	for(i = 0; i<=depth; ++i){

		//Find the ngp from TT
		dtmin = fabs(TT[0].t - t);
		ngp = TT[0];
		
		for(j=0;j<TT.size();++j){
			dt = fabs(TT[j].t - t);
			if(dt<dtmin)
				dtmin = dt,ngp = TT[j];
		}
		/////////////////////

		GoalTra.push_back(ngp);
		t+=dywstar.ahead_time;
	}


	VAs.resize(GoalTra.size());

	for(i=0;i<GoalTra.size();++i)
		VAs[i].SetSingleVA(initobs,GoalTra[i].x,GoalTra[i].y);


	nownode.v = vi;
    nownode.w = wi;
    nownode.x = 0;
    nownode.y = 0;
    nownode.th = 0;
    nownode.parent = -1;
    nownode.depth = 0;
	nownode.lgx = GoalTra[0].x;
	nownode.lgy = GoalTra[0].y;


	/////ND FIND DOOR///////
	//Fnd.ResetReg();
	//Fnd.Set_All_Intervals(initobs.obs);
	Fnd.Find_Gap(0.8);
	Fnd.Find_REGION();
	Fnd.Find_Door(0.49);
    /////ND FIND DOOR////////

	dywstar.Setg = Setg4;
    dywstar.Seth = Seth4;
	dywstar.DWAstarsearch_Tracking(vi,wi,initobs,nownode,depth);

	//dywstar.obs.copyobs(initobs);//複製obs
	//dywstar.dyw.Region_Analysis(0.5,TT[0].x,TT[0].y);
}




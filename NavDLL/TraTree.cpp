#include "StdAfx.h"
#include "TraTree.h"

namespace RobotTra{



TraTree::TraTree(void)
{

    smoothness = 1;
	aheadtime = 1.25;

	msize = 50000;

	VelTree.reserve(msize);

	//alpha[0] = 1;
	//alpha[1] = 0.5;


}

TraTree::~TraTree(void)
{
}

/*
void TraTree::PureExpand(VELnode &bestnode){

	if(tranobs.size() != obsarr.size()) //transfer the obstacles
		tranobs.resize(obsarr.size());
	
	copy(obsarr.begin(),obsarr.end(),tranobs.begin());

	TranObsArr(tranobs,bestnode.x,bestnode.y,bestnode.th);

	dwa->Update_Vel_State(obsarr,bestnode.v,bestnode.w);
	vector<VEL> &cands = dwa->candidates;

	
	double &x0 = bestnode.x;
	double &y0 = bestnode.y;
	double &th0 = bestnode.th;

	int i = 0;
	int s = cands.size();

	for(i=0;i<s;++i){
		nd.v = cands[i].v;
		nd.w = cands[i].w;
		nd.unsafety = cands[i].unsafety;
		nd.th = th0 + nd.w*aheadtime;
	    nd.x = x0 + nd.v*aheadtime*cos(th0 + nd.w*aheadtime/2);
	    nd.y = y0 + nd.v*aheadtime*sin(th0 + nd.w*aheadtime/2);
		nd.parent = bestidx.index;
		nd.depth = bestnode.depth + 1;
		VelTree.push_back(nd);//push_back
	}


}
*/

void TraTree::ExpandNewNodes(VHnode &bestidx,std::vector<OBS> &obsarr){// obsarr has been transformed to the robot coordinate

//TODO
/*
	For each candidate:
	
	1.compute new pose: x =.. y=.. th=...

	
	2.compute cost: g = best.g + ..., h=...

	3. HEAP.push();


	*/
	VELnode &bestnode = VelTree[bestidx.index];
	VELnode nd;//new nodes
	VHnode nh;

	
	dwa->Update_Vel_State(obsarr,bestnode.v,bestnode.w);
	vector<VEL> &cands = dwa->candidates;//dwa->GetCandidates();

	double &x0 = bestnode.x;
	double &y0 = bestnode.y;
	double &th0 = bestnode.th;

	int i = 0;
	int s = cands.size();

	for(i=0;i<s;++i){
		nd.v = cands[i].v;
		nd.w = cands[i].w;
		nd.unsafety = cands[i].unsafety;
		nd.th = th0 + nd.w*aheadtime;
	    nd.x = x0 + nd.v*aheadtime*cos(th0 + nd.w*aheadtime/2);
	    nd.y = y0 + nd.v*aheadtime*sin(th0 + nd.w*aheadtime/2);
		nd.parent = bestidx.index;
		nd.depth = bestnode.depth + 1;
		VelTree.push_back(nd);//push_back

		nh.index = VelTree.size()-1;
		nh.f = Setg(nd,this) + Seth(nd,this);
		VelHeap.push(nh);//push in the heap

	}
}

void TraTree::TraSearch(std::vector<OBS> &initobs, double &rv, double &rw, double lgx, double lgy, int MaximumExpands){

	VHnode bestidx;
	VELnode bestnode;
	VELnode deepestnode;

	//vector<OBS> tranobs;
	int expand_num = 0;
	//aheadtime = dwa->dt;

	tranobs.resize(initobs.size());//initobs should not be changed!

	//initialize the VelTree

	while(!VelHeap.empty())
		VelHeap.pop();
	VelTree.clear();

	this->goalnode.x = lgx;
	this->goalnode.y = lgy;

	bestnode.x = 0;
	bestnode.y = 0;
	bestnode.th = 0;
	bestnode.g = 0;
	bestnode.h = 0;
	bestnode.parent = 0;
	bestnode.depth = 0;
	bestnode.unsafety = 0;
	bestnode.v = rv;
	bestnode.w = rw;
	VelTree.push_back(bestnode);

	deepestnode = bestnode;

	bestidx.f = 0;
	bestidx.index = 0;
	VelHeap.push(bestidx);


	while(expand_num < MaximumExpands && !VelHeap.empty() && VelTree.size()<msize){//Expand new velocities, do tree search
		//Find the best node
		bestidx = VelHeap.top();
		bestnode = VelTree[bestidx.index];
		VelHeap.pop();

		//record the deepest node
		if(deepestnode.depth < bestnode.depth)
			deepestnode = bestnode;

        //Translate obstacle positions to the coordinate of bestnode
	    copy(initobs.begin(),initobs.end(),tranobs.begin());

		TranObsArr(tranobs,bestnode.x,bestnode.y,bestnode.th);
        //Expand new nodes, DWA executes here
	    ExpandNewNodes(bestidx,tranobs);

	    ++expand_num;

	}

	//bestnode = VelTree[(VelHeap.top()).index];//Find best node
	//bestnode = VelTree[VelTree.size() - 1];//Find Last node
	bestnode = deepestnode;//Find the deepest node


	bestpath.clear();
	bestpath.push_back(bestnode);
	
	while(bestnode.depth > 1)//Find immediate best velocity
	{	
		bestnode = VelTree[bestnode.parent];
        bestpath.push_back(bestnode);
	}

	rv = bestnode.v;
	rw = bestnode.w;

	//If there are no nodes?
	if(VelTree.size() == 1)//No nodes expanded!! stop!!
	{
		rv = 0;
		rw = 0;
	}

}

void TranObsArr(std::vector<OBS> &obsarr,double dx,double dy,double dth){//robot: (0,0,0)->(dx,dy,dth), find obs change

 int i = 0;
 int s = obsarr.size();

 double nx;
 double ny;

 //x' = R(-th)*(x-t)
 //R(th) = [cos(th)  -sin(th)]
 //        [sin(th)   cos(th)]

 for(i = 0; i<s; ++i){
    nx = obsarr[i].x - dx;
	ny = obsarr[i].y - dy;
    obsarr[i].x = cos(dth)*nx + sin(dth)*ny;
	obsarr[i].y = cos(dth)*ny - sin(dth)*nx;
	}

}


double Setg1(VELnode& nd,void* datapt){

	TraTree &DWAtree = *(TraTree*)datapt;
	VELnode &pd = DWAtree.VelTree[nd.parent];
	DWA* dwa = DWAtree.dwa;

	double dv = (fabs((nd.v - pd.v)/dwa->Vmax) + fabs((nd.w - pd.w)/dwa->Wmax))*0.5;
	double vavg = 1 - fabs(nd.v/dwa->Vmax);
	nd.g = pd.g + DWAtree.aheadtime + 1*DWAtree.aheadtime*nd.unsafety + 5*DWAtree.aheadtime*vavg;//+ DWAtree.smoothness*dv;// cost以"時間"來表示

	return nd.g;

}

double Seth1(VELnode& nd,void* datapt){

	TraTree &DWAtree = *(TraTree*)datapt;
	VELnode &gd = DWAtree.goalnode;
	DWA* dwa = DWAtree.dwa;

	double dx = nd.x - gd.x;
	double dy = nd.y - gd.y;

	nd.h = sqrt(dx*dx+dy*dy)/(dwa->Vmax);

	return nd.h;

}


double Setg2(VELnode& nd,void* datapt){

	TraTree &DWAtree = *(TraTree*)datapt;
	VELnode &pd = DWAtree.VelTree[nd.parent];
	DWA* dwa = DWAtree.dwa;

	double dv = (fabs((nd.v - pd.v)/dwa->Vmax) + fabs((nd.w - pd.w)/dwa->Wmax))*0.5;
	double vavg = 1 - fabs(nd.v/dwa->Vmax);
	nd.g = pd.g + DWAtree.aheadtime + 0.2*DWAtree.aheadtime*nd.unsafety;//+ DWAtree.smoothness*dv;// cost以"時間"來表示

	return nd.g;

}

double Seth2(VELnode& nd,void* datapt){

	TraTree &DWAtree = *(TraTree*)datapt;
	VELnode &gd = DWAtree.goalnode;
	DWA* dwa = DWAtree.dwa;
	ND* ndp = DWAtree.ndptr;

	double dx = nd.x - gd.x;
	double dy = nd.y - gd.y;


	nd.h = ndp->hval2(nd.x,nd.y,gd.x,gd.y)/(dwa->Vmax);

	return nd.h;

}


double Setg3(VELnode& nd,void* datapt){

	TraTree &DWAtree = *(TraTree*)datapt;
	VELnode &pd = DWAtree.VelTree[nd.parent];
	DWA* dwa = DWAtree.dwa;

	double vavg = 1 - fabs(nd.v/dwa->Vmax);

	int depth = min(nd.depth,DWAtree.mdepth);

	GoalPoint gd = DWAtree.GoalTra[depth];

	double dx = nd.x - gd.x;
	double dy = nd.y - gd.y;

    double tdif = sqrt(dx*dx+dy*dy);

	double p1 = getp1(tdif);

	nd.g = pd.g + (1-p1) + 0.5*vavg;

	return nd.g;

}

double Seth3(VELnode& nd,void* datapt){


    nd.h = 0;
	return nd.h;

}

double Setg4(VELnode& nd,void* datapt){

	TraTree &DWAtree = *(TraTree*)datapt;
	VELnode &pd = DWAtree.VelTree[nd.parent];
    int depth = min(nd.depth,DWAtree.mdepth);
	
	VisableArea& VA = DWAtree.VAs[depth];

	GoalPoint gd = DWAtree.GoalTra[depth];

	double tdif;
	double adif;
	double p1 = 0;
	double p2 = 0;
	double obser;


	if(VA.GetVisableState(nd.x,nd.y,nd.th,tdif,adif)){
		p2 = getp2(adif);
		obser = (1-p2);
		//obser = 0;

	}
	else
		obser = 1.5;  //high cost!!

	p1 = getp1(tdif);

	nd.g = pd.g + (1-p1) + 0.5*obser;

	return nd.g;

}

double Seth4(VELnode& nd,void* datapt){


	TraTree &DWAtree = *(TraTree*)datapt;
	DWA* dwa = DWAtree.dwa;

	int i = nd.depth+1;
	double lgx;
	double lgy;
	double dist;
	double p1 = 0;
	double obser = 0;
	double dtime = DWAtree.aheadtime;
	nd.h = 0;

for(i = nd.depth+1;i<DWAtree.GoalTra.size();++i){
	lgx = DWAtree.GoalTra[i].x;
	lgy = DWAtree.GoalTra[i].y;
/*
	if( !DWAtree.VAs[i].GetVisableState(nd.x,nd.y,nd.th,dist,p1) ){//There are no straight paths
		dist = ndp->hval2(nd.x,nd.y,lgx,lgy);
	}*/

	dist = sqrt( (nd.x-lgx)*(nd.x-lgx) + (nd.y-lgy)*(nd.y-lgy) );
	dist = dist - dtime*(dwa->Vmax);	
    p1 = getp1(dist);

	obser = 1.0 - DWAtree.VAs[i].Predict_Visbility(nd.x,nd.y,dtime*(dwa->Vmax));

    nd.h += (1-p1) + 0.5*obser; 
	dtime += DWAtree.aheadtime;
}

	return nd.h;

}


double Setg5(VELnode& nd,void* datapt){//ND distance

	TraTree &DWAtree = *(TraTree*)datapt;
	VELnode &pd = DWAtree.VelTree[nd.parent];
	ND* ndp = DWAtree.ndptr;
    
	int depth = min(nd.depth,DWAtree.mdepth);
	
	VisableArea& VA = DWAtree.VAs[depth];

	GoalPoint gd = DWAtree.GoalTra[depth];

	double tdif;
	double adif;
	double p1 = 0;
	double p2 = 0;
	double obser;
	double* alpha = DWAtree.alpha;


	if(VA.GetVisableState(nd.x,nd.y,nd.th,tdif,adif)){
		p2 = getp2(adif);
		obser = (1-p2);
		//obser = 0;

	}
	else{
		obser = 1.5;  //high cost!!
		tdif = ndp->hval2(nd.x,nd.y,gd.x,gd.y);
	}

	p1 = getp1(tdif);

	nd.g = pd.g + alpha[0]*(1-p1) + alpha[1]*obser + alpha[2]*nd.unsafety;

	return nd.g;

}

double Seth5(VELnode& nd,void* datapt){


	TraTree &DWAtree = *(TraTree*)datapt;
	DWA* dwa = DWAtree.dwa;
	ND* ndp = DWAtree.ndptr;

	int i = nd.depth+1;
	double lgx;
	double lgy;
	double dist;
	double p1 = 0;
	double obser = 0;
	double dtime = DWAtree.aheadtime;
	double* alpha = DWAtree.alpha;
	nd.h = 0;

for(i = nd.depth+1;i<DWAtree.GoalTra.size();++i){
	lgx = DWAtree.GoalTra[i].x;
	lgy = DWAtree.GoalTra[i].y;

	if( !DWAtree.VAs[i].GetVisableState(nd.x,nd.y,nd.th,dist,p1) ){//There are no straight paths
		dist = ndp->hval2(nd.x,nd.y,lgx,lgy);
	}

	dist = sqrt( (nd.x-lgx)*(nd.x-lgx) + (nd.y-lgy)*(nd.y-lgy) );
	dist = dist - dtime*(dwa->Vmax);	
    p1 = getp1(dist);

	obser = 1.0 - DWAtree.VAs[i].Predict_Visbility(nd.x,nd.y,dtime*(dwa->Vmax));

    nd.h += alpha[0]*(1-p1) + alpha[1]*obser; 
	dtime += DWAtree.aheadtime;
}

	return nd.h;

}


double getp1(double tdif){
	double p1 = (8-tdif)/7;  // 2~8m --> score: 1~0
	p1 = max(0,p1);
	p1 = min(1,p1);

	return p1;
}


double getp2(double adif){
	double p2 = (PI - fabs(adif) )/( PI ); //1/8 PI ~ 3/8 PI --> score: 1~0  加上角度限制 就表現得很怪!
	p2 = max(0,p2);
	p2 = min(1,p2);

	return p2;
}



void DoNavi(TraTree& DWAtree, ObsArray& initobs, double& vi, double& wi, double lgx,double lgy){//initobs: (0,0,0)


    /////ND FIND DOOR////////
	DWAtree.Setg = Setg2;
    DWAtree.Seth = Seth2;

	DWAtree.TraSearch(initobs.obs, vi, wi, lgx, lgy, 100);

}

void DoTracking(TraTree& DWAtree,ObsArray& initobs, double& nv, double& nw, vector<GoalPoint>& TT){
//Set Goal trajectory
	int i = 0;
	int j = 0;
	double t = 0;
	double dt = 0;
	double dtmin = 0;
	int depth = 10;
	GoalPoint ngp;
	vector<GoalPoint>& GoalTra = DWAtree.GoalTra;
	vector<VisableArea>& VAs = DWAtree.VAs;
	
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
		
		GoalTra.push_back(ngp);
		t += DWAtree.aheadtime;
	}

    //////Compute Visable Area///
	VAs.resize(GoalTra.size());

	for(i=0;i<GoalTra.size();++i)
		VAs[i].SetSingleVA(initobs,GoalTra[i].x,GoalTra[i].y);

	////////////////////////////



    /////ND FIND DOOR////////
	DWAtree.Setg = Setg5;
    DWAtree.Seth = Seth5;
	//DWAtree.aheadtime = 1.0;
	DWAtree.mdepth = depth-1;
	DWAtree.TraSearch(initobs.obs, nv, nw, 20, 0, 200);
}


}//namespace



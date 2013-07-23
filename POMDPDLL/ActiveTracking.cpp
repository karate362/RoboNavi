#include "stdafx.h"
#include "ActiveTracking.h"

using namespace RobotTra;

void GenerateQ_TS(BELnode& nb,void* AG,void* datapt){// Generate Actions and compute new Q node(beliefs)
//In this work, we only have three actions:
	TraTree* dwastar = (TraTree*)AG;
	HMOMDP* hmp = (HMOMDP*) datapt;
	vector<Qnode>& QArray = hmp->QArray;
	Qnode nq;
	VELnode& nx = *((VELnode*)nb.state);

	//DWA* Expand

	dwastar->PureExpand(

}
#pragma once

//#include "BelNode.h"
#include "HMOMDP.h"
/*
State: x1 and x2.

Action: u1,u2,u3

Observation: z = x1, z = x2.

Reward:
r(x1,u1) = -100, r(x2,u1) = 100, r(x1,u2) = 100, r(x2,u2) = -50;

Believe: 
(p1,1-p1)



If we use AEMS2:
Ht(b) = U(b)-L(b)
Hba = 1 or 0
Hbaz = p(z|b,a)

*/

void GenerateQ_TS(BELnode& nb,void* AG,void* datapt);// Generate Actions and compute new Q node(beliefs)
void GenerateB_TS(Qnode& nq,void* OG,void* datapt);// Generate Observations and compute new beliefs

void computeQvalue_TS(BELnode& nb,int qidx,void* datapt);// only find the max value
void BackupQvalue_TS(Qnode& nq,void* datapt);//only find the max value

double likelihood_TS(void* bel,void* o);
void* BeliefUpdate_TS(BELnode& pb, void* a, void* o, void* datapt);


//void computeBvalue_TS(Qnode& nq,int bidx,void* datapt);//Is it needed?
double computeReward_TS(BELnode& nb,void* action,void* datapt);//reward of doing a at believe b.

void BelOutput_TS(FILE* out,void* bel);

double computeHb_TS(BELnode& nb,void* datapt);
double computeHba_TS(BELnode& nb,void* action,void* datapt);
double computeHbaz_TS(BELnode& nb,void* action,void* observation,void* datapt);

int _declspec(dllexport) planning_TS(double p1,HMOMDP& hmp);


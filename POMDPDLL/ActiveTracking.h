#pragma once
#include "HMOMDP.h"
#include "TraTree.h"

using namespace RobotTra;



void GenerateQ_AT(BELnode& nb,void* AG,void* datapt);// Generate Actions and compute new Q node(beliefs)
void GenerateB_AT(Qnode& nq,void* OG,void* datapt);// Generate Observations and compute new beliefs

void computeQvalue_AT(BELnode& nb,int qidx,void* datapt);// only find the max value
void BackupQvalue_AT(Qnode& nq,void* datapt);//only find the max value

double likelihood_AT(void* bel,void* o);
void* BeliefUpdate_AT(BELnode& pb, void* a, void* o, void* datapt);


//void computeBvalue_TS(Qnode& nq,int bidx,void* datapt);//Is it needed?
double computeReward_AT(BELnode& nb,void* action,void* datapt);//reward of doing a at believe b.

void BelOutput_AT(FILE* out,void* bel);

double computeHb_AT(BELnode& nb,void* datapt);
double computeHba_AT(BELnode& nb,void* action,void* datapt);
double computeHbaz_AT(BELnode& nb,void* action,void* observation,void* datapt);

int _declspec(dllexport) planning_AT(HMOMDP& hmp,void* datapt);

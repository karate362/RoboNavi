#include "StdAfx.h"
#include "TwoStateEx.h"
using namespace std;

int TS_Action[] = {1,2,3};
int TS_Observation[] = {1,2}; //x1, x2
vector <double> TS_bels;//p(x1)


void GenerateQ_TS(BELnode& nb,void* AG,void* datapt){// Generate Actions and compute new Q node(beliefs)
//In this work, we only have three actions:
	HMOMDP* hmp = (HMOMDP*) datapt;
	vector<Qnode>& QArray = hmp->QArray;
	Qnode nq;

	for(int i=0;i<3;++i){
		nq.action = (TS_Action + i);
		QArray.push_back(nq);
	}
}
	
	
void GenerateB_TS(Qnode& nq,void* OG,void* datapt){// Generate Observations and compute new beliefs

	HMOMDP* hmp = (HMOMDP*) datapt;
	vector<BELnode>& BELArray = hmp->BELArray;
	BELnode &pb = BELArray[nq.parent];
	BELnode nb;
	int act = *((int*)nq.action);

	//If the nq.action is ending...
	if(act == 1 || act == 2){ //ending action
    //Only push back a ending node
	//Set Uv,Lv
		nb.bel = pb.bel;
		nb.observation = TS_Observation;
		nb.Uv = 0;
		nb.Lv = 0;
		nb.Hb = 0;
		nb.prob = 1;
		BELArray.push_back(nb);	/**/
		return;
	}
	//Now I don;t compute B(b,a), but in the future we can only compute B(b,a) in this function and sample O!!

	for(int i=0;i<2;++i){

		nb.observation = (TS_Observation + i);
		nb.prob = likelihood_TS(pb.bel,nb.observation);//likelihood
		nb.bel = BeliefUpdate_TS(pb,nq.action,nb.observation,NULL);	
		nb.Uv = 100;
		//nb.Lv = -100; 
		nb.Lv  =computeReward_TS(nb,&(TS_Action[0]),NULL);//constant policy: action 1
		nb.Hb = computeHb_TS(nb,NULL);
		
		BELArray.push_back(nb);	/**/
	}

}

double likelihood_TS(void* bel,void* o){
	//TODO:
	double p1 =  *((double*)bel);
	int obs = *((int*)o);
	double l=0;

	if(obs==1)
		l = p1*0.7 + (1-p1)*0.3;

	if(obs==2)
		l = p1*0.3 + (1-p1)*0.7;

	return l;
}

void* BeliefUpdate_TS(BELnode& pb, void* a, void* o, void* datapt){
// given u3: x1->x1: 0.2, x1->x2: 0.8...
// z1 from x1: 0.7, z1 from x2: 0.3...


	double p = *((double*)pb.bel);
	int act = *((int*)a);
	int obs = *((int*)o);
	double p1 = p;
	double p2 = 1-p;

	//action: only u3
	p1 = 0.2*p1 + 0.8*p2;
	p2 = 1-p1;

	if(obs==1){
		p1 = p1*0.7;
		p2 = p2*0.3;
	}

	if(obs==2){
		p1 = p1*0.3;
		p2 = p2*0.7;
	}

	p1 = p1/(p1+p2);


	TS_bels.push_back(p1);

	return &( *( TS_bels.end()-1 )  );// it is : &( TS_bels[TS_bels.size()-1]  );
}


void computeQvalue_TS(BELnode& nb,int qidx,void* datapt){
	//For my equation, it should be a weighting of other q...
	//Here we should not do anything

	HMOMDP* hmp = (HMOMDP*) datapt;
	vector<Qnode>& QArray = hmp->QArray;
	Qnode& nq = QArray[qidx];
	nq.bUv = nq.Uv;
	nq.bLv = nq.Lv;
}


void BackupQvalue_TS(Qnode& nq,void* datapt){//	

	nq.bUv = nq.Uv;
	nq.bLv = nq.Lv;
}




double computeReward_TS(BELnode& nb,void* a,void* datapt){
	double p1 = *((double*)nb.bel);
	double r = 0;
	int act = *((int*)a);

	switch (act){
		case 1:
			r = p1*(-100.0) + (1-p1)*100.0;
			break;
		case 2:
			r = p1*(100.0) + (1-p1)*(-50.0);
			break;
		case 3:
			r = -1.0;
			break;
		default:
			r = 0;
	}
		
	return r;
}//reward of doing a at believe b.

/*
If we use AEMS2:
Ht(b) = U(b)-L(b)
Hba = 1 or 0
Hbaz = p(z|b,a)
*/

double computeHb_TS(BELnode& nb,void* datapt){//
	return nb.Uv - nb.Lv;
}

double computeHba_TS(BELnode& nb,void* action,void* datapt){//

	return 1.0;
}

double computeHbaz_TS(BELnode& nb,void* action,void* observation,void* datapt){
	
	return likelihood_TS(nb.bel,observation);

}


void BelOutput_TS(FILE* fout,void* bel){
	
	double p1 = *((double*)bel);



	fprintf(fout,"%.2f ",p1);	

}

int planning_TS(double p1,HMOMDP& hmp){
    BELnode nb;
	TS_bels.clear();
	TS_bels.push_back(p1);
	nb.bel = &(TS_bels[0]);
	nb.parent = -1;
	nb.b_best = 0;
	nb.Uv = 100;
	nb.Lv = -100;
	nb.Hb = 200;

	hmp.GenerateQ = GenerateQ_TS;
	hmp.GenerateB = GenerateB_TS;
	hmp.computeQvalue = computeQvalue_TS;
	hmp.BackupQvalue = BackupQvalue_TS;
	hmp.computeReward = computeReward_TS;
	hmp.computeHba = computeHba_TS;
	hmp.computeHbaz = computeHbaz_TS;
	hmp.BELOutput = BelOutput_TS;

	hmp.Planning(nb);


return 0;
}
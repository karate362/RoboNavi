#include "StdAfx.h"
#include "HMOMDP.h"
#include <algorithm>
#include <numeric>


HMOMDP::HMOMDP(void)
{
	disfactor = 1.0;
	BELArray.reserve(50000);
	QArray.reserve(50000);
}

HMOMDP::~HMOMDP(void)
{
	BELArray.~vector();
	QArray.~vector();
}


void HMOMDP::Expand(BELnode& nb){
vector<Qnode>::iterator qi;
int i=0;
int bidx = &nb - &(BELArray[0]);

//TODO: Get action set, for each action, expand Qnodes, save them using QArray, Q_start and Q_end
nb.Q_start = QArray.size();
GenerateQ(nb,this->ActionGenerator,this);
nb.Q_end = QArray.size() - 1;

//TODO: for each new Qnode, expand BELnodes and compute Q values (weighting sum of new BEL values)
for(i=nb.Q_start; i<=nb.Q_end; ++i){
	QArray[i].parent = bidx;
	ExpandQ(QArray[i]);
}


for(i=nb.Q_start; i<=nb.Q_end; ++i){
	this->computeQvalue(nb,i,this); //computing the Q-value for back up, saved in bUv, bLv
}


for(i=nb.Q_start; i<=nb.Q_end; ++i){
	QArray[i].Hba = computeHba(nb,QArray[i].action,this);//U(b,a) = R(b,a) + sum_z( p(z|b,a)*U(b,a,z) ), it could be the value of Qnode
}

//Updating the upper bound and lower bound
oldbUv = nb.Uv;
oldbLv = nb.Lv;
qi = std::max_element( QArray.begin()+nb.Q_start, QArray.begin()+nb.Q_end+1, QUCmp);
nb.Uv = min(nb.Uv,qi->bUv);
qi = std::max_element( QArray.begin()+nb.Q_start, QArray.begin()+nb.Q_end+1, QLCmp);
nb.Lv = max(nb.Lv,qi->bLv);

//Updating the heuristic value
qi = std::max_element( QArray.begin()+nb.Q_start, QArray.begin()+nb.Q_end+1, HbaCmp);
nb.Hb = qi->Hb * qi->Hba;
nb.b_best = qi->b_best;

}


void HMOMDP::ExpandQ(Qnode &nq){
vector<BELnode>::iterator bi;
int qidx = &nq - &(QArray[0]);
int i=0;

//TODO: Get observation set
BELnode& nb = BELArray[nq.parent];


//TODO: for each observation, expand BELnodes, save them using BELArray, B_start and B_end
nq.B_start = BELArray.size();
GenerateB(nq,ObsGenerator,this);//save all new BELnodes in the BELArray
nq.B_end = BELArray.size()-1;


//computing value bound s of this node.
//Updating the heuristic value
nq.Uv = computeReward(nb,nq.action,this);
nq.Lv = nq.Uv;
for(i=nq.B_start; i<=nq.B_end; ++i){
	BELArray[i].b_best = i;// set b_best: itself
	BELArray[i].parent = qidx;
	//BELArray[i].Hb = computeHb( nb, NULL); // Done generateB
	BELArray[i].Hbaz = computeHbaz( BELArray[nq.parent],nq.action,BELArray[i].observation,this );
	nq.Uv += disfactor * BELArray[i].prob * BELArray[i].Uv;// updating self Q-value   
	nq.Lv += disfactor * BELArray[i].prob * BELArray[i].Lv;  
}
bi =  std::max_element( BELArray.begin()+nq.B_start, BELArray.begin()+nq.B_end+1, HbazCmp);
nq.Hb = bi->Hbaz * bi->Hb;
nq.b_best = bi->b_best;//index

}


int HMOMDP::Backup(BELnode& nb){
//TODO:
vector<BELnode>::iterator bi;
//Update the value and heuristic of nq
	Qnode& nq = QArray[nb.parent];
	oldqUv = nq.Uv;
	oldqLv = nq.Lv;
	nq.Uv = nq.Uv + disfactor * nb.prob * (nb.Uv - oldbUv);
	nq.Lv = nq.Lv + disfactor * nb.prob * (nb.Lv - oldbLv);

//From sibling node...
	//So... if there are ending nodes, they should not be put into the BELArray?
bi = std::max_element( BELArray.begin()+nq.B_start, BELArray.begin()+nq.B_end+1, HbazCmp);
nq.Hb = bi->Hbaz * bi->Hb;
nq.b_best = bi->b_best;//index

BackupQ(nq);

return nq.parent;

}


void HMOMDP::BackupQ(Qnode& nq){
//TODO:
vector<Qnode>::iterator qi;
BELnode& nb = BELArray[nq.parent];
BackupQvalue(nq,NULL);// update all sibling node's sUv and sLv

oldbUv = nb.Uv;
oldbLv = nb.Lv;

qi = std::max_element( QArray.begin()+nb.Q_start, QArray.begin()+nb.Q_end+1, QUCmp);
nb.Uv = qi->bUv;
qi = std::max_element( QArray.begin()+nb.Q_start, QArray.begin()+nb.Q_end+1, QLCmp);
nb.Lv = qi->bLv;

//Updating the heuristic value
qi = std::max_element( QArray.begin()+nb.Q_start, QArray.begin()+nb.Q_end+1, HbaCmp);
nb.Hb = qi->Hb * qi->Hba;
nb.b_best = qi->b_best;

}


void HMOMDP::Planning(BELnode& nb){

	BELArray.clear();
	QArray.clear();

//Expand and Backup repeatedly
	vector<Qnode>::iterator qi;
	nb.b_best = 0;
	//nb.parent = -1;
	BELArray.push_back(nb);
	int bidx;

	while(BELArray.size()<30000 && BELArray[0].Hb > 0){ //if Hb == 0, the tree will not be improved anymore
		bidx = (BELArray[0].b_best);//Choose the node with the highest heuristic value
		Expand(BELArray[bidx]);


		while(bidx!=0)
			bidx = Backup(BELArray[bidx]);
	}
	
	if(!QArray.empty())
	  qi = std::max_element( QArray.begin()+BELArray[0].Q_start, QArray.begin()+BELArray[0].Q_end+1, QLCmp);//If QArray..size() == 0... error!!

	this->Uv = BELArray[0].Uv;
	this->Lv = BELArray[0].Lv;
	this->best_action = qi->action;

}

void HMOMDP::BELTreeOutput(){

vector<BELnode>::iterator bi = BELArray.begin();

	FILE* fout = fopen("POMDP_Tree.txt","w");

	fprintf(fout,"%d %.2f %.2f ",-1,bi->Uv,bi->Lv);
	this->BELOutput(fout,bi->bel);
	fprintf(fout,"\n");

	for(int i=1;i<BELArray.size();++i){
		++bi;
		fprintf(fout,"%d %.2f %.2f ",QArray[bi->parent].parent,bi->Uv,bi->Lv);	
		this->BELOutput(fout,bi->bel);
		fprintf(fout,"\n");
	}


	fclose(fout);


}
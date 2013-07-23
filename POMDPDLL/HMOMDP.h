#pragma once

#include "BelNode.h"
#include <vector>
#include <queue>

using namespace std;

class _declspec(dllexport) HMOMDP
{
public:
	HMOMDP(void);
	~HMOMDP(void);

	void Planning(BELnode& nb);




public:

	void Expand(BELnode& nb);//expand Qnodes from this BELnode
	int Backup(BELnode& nb);//Backup Qnodes from this BELnode

	void ExpandQ(Qnode& nq);//expand BELnodes from this Qnode
	void BackupQ(Qnode& nq);//Backup BELnodes from this Qnode

	void BELTreeOutput();

	void (*GenerateQ)(BELnode& nb,void* AG,void* datapt);
	void (*GenerateB)(Qnode& nq,void* OG,void* datapt);// using the Action Generator , this is a function pointer
	void (*computeQvalue)(BELnode& nb,int qidx,void* datapt);// computing (sUv, sLv) from Uv, Lv, don't modify siblings!!
	void (*BackupQvalue)(Qnode& nq,void* datapt);// update all sibling node's sUv and sLv
    void (*computeBvalue)(Qnode& nq,int bidx,void* datapt);
	double (*computeReward)(BELnode& nb,void* action,void* datapt);//reward of doing a at believe b.

	double (*computeHb)(BELnode& nb,void* datapt);
    double (*computeHba)(BELnode& nb,void* action,void* datapt);
    double (*computeHbaz)(BELnode& nb,void* action,void* observation,void* datapt);

	void (*BELOutput)(FILE* out,void* bel);

	std::vector<BELnode> BELArray;//index 0 == root node
	std::vector<Qnode> QArray;

	double Uv;
	double Lv;
	void* best_action;

    
private:
	void* ActionGenerator;//It can be TraTree...
	void* ObsGenerator;//Observation generator
	void* otherdata;

	double disfactor;

	double oldbUv;
	double oldbLv;
	double oldqUv;
	double oldqLv;


};

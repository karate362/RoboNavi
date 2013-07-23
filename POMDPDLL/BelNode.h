//#pragma once
#ifndef BELNODE_H_
#define BELNODE_H_

struct _declspec(dllexport) BELnode
{
	double Uv;//value upper bound
	double Lv;//value lower bound
	double prob;
	void* observation;
	int parent;//for root node, parent == -1
	int Q_start;
	int Q_end;

	void* state;
	void* bel;

	double Hb;
	double Hbaz;
	int b_best;

};

struct BHnode//HEAP node
{
	double f;
	int index;
};

class BHCmp{
public:
    bool operator () (const BHnode a,const BHnode b)
    {
        return a.f > b.f;
    }
};

struct _declspec(dllexport) Qnode
{
	double bUv;// upper bound, related to sibling nodes
	double bLv;// lower bound, related to sibling nodes

	double Uv;//value upper bound
	double Lv;//value lower bound
	void* action;//input command
	int parent;
	int B_start;
	int B_end;

	void* state;
	void* bel;

	double Hb;
	double Hba;
	int b_best;

};

//Comparing functions
bool QUCmp(Qnode& i, Qnode& j);
bool QLCmp(Qnode& i, Qnode& j);


bool HbaCmp(Qnode& i, Qnode& j);
bool HbazCmp(BELnode& i, BELnode& j);


#endif
#include "StdAfx.h"
#include "BelNode.h"


//Comparing functions
bool QUCmp(Qnode& i, Qnode& j) { return i.bUv<j.bUv; }
bool QLCmp(Qnode& i, Qnode& j) { return i.bLv<j.bLv; }


bool HbaCmp(Qnode& i, Qnode& j) { return (i.Hb * i.Hba) < (j.Hb * j.Hba); }
bool HbazCmp(BELnode& i, BELnode& j) { return (i.Hb * i.Hbaz)<(j.Hb * j.Hbaz); }
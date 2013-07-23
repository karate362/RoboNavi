// VelReg.cpp: implementation of the VelReg class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "VelReg.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////



VelReg::VelReg()
{
D_MAX = 10.0;
}

VelReg::~VelReg()
{

}

void VelReg::Save_Reg(ofstream& out)
{
int i;
int s = regions.size();
DWA_VALLEY* pr;
for(i=0;i<s;++i)
{
	pr = &regions[i];
	out<<pr->sr<<' '<<pr->sl<<' '<<pr->ITVg<<' '<<pr->type<<' '<<pr->v<<' '<<pr->w<<endl;
}

}

int VelReg::DegtoITV(double th)//theta->ITV index
{

//return (int)(th + 0.5);

return (int)((double)(ITVsize-1)*th/180.0 + 0.5);
}


double VelReg::ITVtoRad(int index)
{
	return tan(((double)index)*PI/180);
}

void VelReg::Set_VFH(OBS* o)//D_MAX: initial value
{
//障礙物只計-90~90度


}


void VelReg::Set_Intervals(OBS* o)//D_MAX: initial value
{
double &xo = o->x;
double &yo = o->y;
double &ro = o->r;

double th1=0;
double th2=0;
double buf=0;//計算方便用
double R=0;
double th = 0;
double d;

int i1=0;
int i2=0;
int i=0;

//if(xo+ro < 0)//不考慮後方...?
//return;

//找出影響的intervals
buf = (xo*xo + yo*yo - ro*ro)/2;

if(buf < 0)//可能有誤差
buf = 0;

th1 = atan2( buf , (yo+ro) ) * 180/PI;//會在0~180之間, 因buf必大於0
th2 = atan2( buf , (yo-ro) ) * 180/PI;
i1 = DegtoITV(th1);
i2 = DegtoITV(th2);
if(i2<i1)//令index i1<i2
i=i2,i2=i1,i1=i;

//計算碰撞距離
th = atan2( (xo*xo + yo*yo)/2 , yo );//velocity space上的角度

if(th>89.5*PI/180 && th<90.5*PI/180)//可以視為直線了
{
	if(xo<=0)
		d = D_MAX;
	else
        d = xo - ro;
}
else//並非直線
{
	R=tan(th);//回轉半徑
	if(R>=0)
		th=atan2(xo,R-yo);
	else
		th=atan2(xo,yo-R);
	if(th<0)//令在0~360之內
		th+=2*PI;
	d = fabs(R)*th - ro;
}

//Set Intervals
for(i=i1;i<=i2;++i)
{
	if(d<intervals[i].dmax)
		intervals[i].dmax = d;
}

}

void VelReg::Set_All_Intervals(vector<OBS>& obsarr)
{
    vector<OBS> &obs = obsarr;
	int i=0;
	int s=obsarr.size();
	for(i=0;i<s;++i)
		Set_Intervals(&obs[i]);
}


void VelReg::Find_Gap(double threshold)
{
	int i = 0;
	double dis = 0;
	GAP ng;

	gaps.clear();

	ng.i = 0;
	ng.max = false;//left > right
	gaps.push_back(ng);

	for(i=1;i<ITVsize;++i)
	{
		dis = intervals[i].dmax - intervals[i-1].dmax;

        if(dis<0)
			ng.max = true;
		else
			ng.max = false;

		dis = fabs(dis);

		if( dis > threshold || (intervals[i].dmax-3)*(intervals[i-1].dmax-3)<0 )//加上一條: 若其一超過3m長度...?
		{
			ng.i = i;
			gaps.push_back(ng);
		}
	}
	
	ng.i = ITVsize;
	ng.max = true;// right > left
	gaps.push_back(ng);


}

void VelReg::Find_REGION(double lgx,double lgy)
{
int i = 0;
int j = 0;
int s = gaps.size();
int s2 = 0;

DWA_VALLEY nv;
nv.reward_max = -1;
nv.navigable = false;
nv.v = 0;
nv.w = 0;
regions.clear();

GAP* gl;
GAP* gr;

double goaldeg = atan2( (lgx*lgx + lgy*lgy) , (2*lgy) )*180/PI;
double goaldis = sqrt(lgx*lgx + lgy*lgy);
int goalITV = DegtoITV(goaldeg);

for(i=1; i<s; ++i)//比較i和i-1之間
{
	gr = &gaps[i-1];
	gl = &gaps[i];

	nv.rise_r = (bool)(1 - gr->max);
	nv.rise_l = gl->max;
	nv.sr = gr->i;
	nv.sl = gl->i - 1;//i從0~ITVsize 但第一個gap必定是 i = 0;應不用擔心index問題
	//nv.navigable = false;

	if(nv.rise_r || nv.rise_l)//有至少一邊是rising
	{  
		if(abs(nv.sr-nv.sl) > 60 )
			this->SplitReg(nv);//PutNewRegion(nv);//
		else
			PutNewRegion(nv);
	}/*
	else//goal在此region中
	{
		if((goalITV-nv.sr)*(goalITV-nv.sl)<=0 && intervals[goalITV].dmax>goaldis && lgx>0 )//HSGR
		{
        PutNewRegion(nv);
		}

	}*/

}

}

void VelReg::SplitReg(DWA_VALLEY& reg){
	
	DWA_VALLEY nv;
	int orinsize = abs(reg.sl - reg.sr);
	int splitnum = (int)(orinsize/45) + 1;
	int del = orinsize/splitnum + 1;

	for(int i=0;i<splitnum;++i){//r->l, increasing
		nv.sr = reg.sr + i*del;
		nv.sl = min(nv.sr+del-1,reg.sl);
		PutNewRegion(nv);
	}

}

inline void VelReg::PutNewRegion(DWA_VALLEY& nv){

		int s2=this->regions.size();
		
		for(int j=nv.sr;j<=nv.sl;++j)
          this->intervals[j].regindex = s2;
		this->regions.push_back(nv);
}
/*
void VelReg::SplitReg(DWA_VALLEY& reg,double lgx,double lgy){
	
	DWA_VALLEY nv;
	int orinsize = abs(reg.sl - reg.sr);
	int splitnum = orinsize/45 + 1;
	int del = orinsize/splitnum + 1;

	int itvi=0;

	for(int i=1;i<splitnum;++i){//r->l, increasing
		nv.sr = reg.sr + i*del;
		nv.sl = min(nv.sr+del,reg.sl);
		Set_Situation(nv,lgx,lgy);

		PutNewRegion(nv);
	}
	reg.sl = reg.sr+del;
    Set_Situation(reg, lgx, lgy);

}
*/




void VelReg::Set_Situation(DWA_VALLEY& reg,double lgx,double lgy)
{/*
int smr = reg.sr-1;
if(smr<0)
smr=0;

int sml = reg.sl+1;
if(sml>=ITVsize)
sml=ITVsize-1;

double disr = intervals[smr].dmax;
double disl = intervals[sml].dmax;*/
double goaldeg = atan2( (lgx*lgx + lgy*lgy) , (2*lgy) )*180/PI;
double goaldis = sqrt(lgx*lgx + lgy*lgy);


int goalITV = DegtoITV(goaldeg);

/*
if(disr<0.3 || disl<0.3)//LS
{
	
	if(disr<0.3 && disl<0.3)//LS2
		reg.type=LS2;
	else
		reg.type=LS1;
}
else
{//HS
	if(goalITV>=reg.sr && goalITV<=reg.sl && intervals[goalITV].dmax>goaldis && lgx>0 )//HSGR
		reg.type=HSGR;
	else
		if(abs(reg.sl-reg.sr)>45){// Split the region each region should be smaller than 45...
			
			reg.type=HSWR;
			//SplitReg(reg,lgx,lgy);
			//return;

		}
	    else
		    reg.type=HSNR;
}

if(abs(reg.sl-reg.sr)>45){// Split the region, each region should be smaller than 45...
			
	//reg.type=HSWR;
	SplitReg(reg,lgx,lgy);
	return;
	} 
else*/

if(goalITV>=reg.sr && goalITV<=reg.sl && intervals[goalITV].dmax>goaldis && lgx>0 )//HSGR
		reg.type=HSGR;

else
reg.type=HSNR;


switch(reg.type)
{
	/*
case LS1:
	if(disr>disl)
		reg.ITVg=reg.sr;
	else
		reg.ITVg=reg.sl;
break;

case LS2:
	reg.ITVg=(reg.sr+reg.sl)/2;
break;
*/
case HSGR:
	reg.ITVg=goalITV;
break;

case HSWR:
	if(goalITV<=reg.sr)
		reg.ITVg=(2*reg.sr + 1*reg.sl)/3;
	else
	if(goalITV>=reg.sl)
		reg.ITVg=(1*reg.sr + 2*reg.sl)/3;
	else
		reg.ITVg = goalITV;
break;

case HSNR:
	reg.ITVg=(reg.sr+reg.sl)/2;
break;


default:
break;
}



}

/////////////////////////////reward function////////////////////////
double VelReg::Regheading(double v,double w)
{
double reward;
double width;
int ITVi = DegtoITV(atan2(v,w)*180/PI);
int ri = intervals[ITVi].regindex;

if(ri<0)
return 0;

DWA_VALLEY& reg = regions[ri];

if(reg.sl == reg.sr)
  reward = 1;
else
  reward = 1.0 - (double)(abs(ITVi-reg.ITVg))/(double)(abs(reg.sl - reg.sr));

return reward;
}


double VelReg::Regmargin(double v,double w,double dt)
{
double reward;
int ITVi = DegtoITV(atan2(v,w)*180/PI);
int ri = intervals[ITVi].regindex;

if(ri<0)
return 0;

DWA_VALLEY& reg = regions[ri];

if(reg.sl == reg.sr)
  reward = 0;
else
  reward = 1.0 - 2*(double)(abs(ITVi-(reg.sl+reg.sr)/2))/(double)(abs(reg.sl - reg.sr));

return reward;
}

//////////////////////////////////////////////////////////////////////
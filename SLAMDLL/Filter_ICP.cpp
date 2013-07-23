#include "StdAfx.h"
#include "Filter_ICP.h"

namespace Geom2D {

Filter_ICP::Filter_ICP(void)
{
}

Filter_ICP::~Filter_ICP(void)
{
}

Pose Filter_ICP::DoICP(const vector<Point>& RawModel,const vector<Point>& RawData,const Pose& init,int trys,vector<double>& dmin_R,vector<double>& dmin_Z){//Data: (x,y)
//Model ~= R*Data + t

	Transform2D T(init);
	MODEL.resize(RawModel.size());
    DATA.resize(RawData.size());

	copy(RawModel.begin(),RawModel.end(),MODEL.begin());
	copy(RawData.begin(),RawData.end(),DATA.begin());

	Pose dpose;
	icpose = init;

	//determine dmin_S
	int i = 0;
	int s = RawData.size();
	dmin_S.resize(s);
	for(i=0;i<s;++i)
		dmin_S[i] = dmin_R[i] + dmin_Z[i];
		//dmin_S[i] = 0.15;

	//ICP

	for(i=0;i<trys;++i){
		T.transform_to_global_vector(DATA);
		Find_Nearest(MODEL,DATA);

		if(nB.size() >= DATA.size()/5){//enough matched points
		dpose = ICP_Least_Square(nA,nB);
  
		T.SetBase(dpose);
		T.transform_to_global(icpose.p);
		icpose.phi+=dpose.phi;
		}
		else
			i = trys;
	
	}
	return icpose;
}

void Filter_ICP::Find_Nearest(const std::vector<Point>& A, const std::vector<Point>& B){

	int min = 0;
	double dmin = 0.3;
	int s = B.size();
	int idx;


	nA.clear();
	nB.clear();

	for(int i=0;i<s;++i){
		dmin = dmin_S[i];
		idx = Find_Nearest_Node(A,B[i]);
		if( Compute_Distance(A[idx],B[i]) < dmin){// in proper range
			nA.push_back(A[idx]);
			nB.push_back(B[i]);
		}

	}


   

}

int Filter_ICP::Find_Nearest_Node(const std::vector<Point>& Model, const Point& n){
	int i = 0;
	int idx = 0;
	int s = Model.size();
	double dmin = Compute_Distance(Model[0],n);
    double imin = dmin;

	for(i=1;i<s;++i){
		imin = Compute_Distance(Model[i],n);
		if(imin<dmin)
			dmin=imin, idx=i;
	}

	return idx;
}

Pose Filter_ICP::ICP_Least_Square(const std::vector<Point> &A, const std::vector<Point> &B){//A ~ R*B+t

	//assert(A.size() == B.size() && A.size() != 0);//should be the same size
	int n = A.size();
	double N = static_cast<double>(n);

	double sAx,sAy,sBx,sBy;//summation
	double sAxBx,sAyBy,sAxBy,sAyBx;
	double mAx,mAy,mBx,mBy;

    sAx=sAy=sBx=sBy=sAxBx=sAyBy=sAxBy=sAyBx=mAx=mAy=mBx=mBy=0;

	for(int i=0;i<n;++i){
		sAx+=A[i].x;
		sAy+=A[i].y;
		sBx+=B[i].x;
		sBy+=B[i].y;
		sAxBx+=A[i].x*B[i].x;
		sAyBy+=A[i].y*B[i].y;
		sAxBy+=A[i].x*B[i].y;
		sAyBx+=A[i].y*B[i].x;
	}
	mAx = sAx/N;
	mAy = sAy/N;
	mBx = sBx/N;
	mBy = sBy/N;

	double Sxx = sAxBx - sAx*sBx/N; // calculate S
	double Syy = sAyBy - sAy*sBy/N;
	double Sxy = sAxBy - sAx*sBy/N;
	double Syx = sAyBx - sAy*sBx/N;

	double phi = atan2(Syx-Sxy, Sxx+Syy); 
	
	Pose pse; // calculate pose
	pse.p.x = mAx - (mBx*cos(phi) - mBy*sin(phi));
	pse.p.y = mAy - (mBx*sin(phi) + mBy*cos(phi));
	pse.phi = phi;

	return pse;

}




};//Geom2D
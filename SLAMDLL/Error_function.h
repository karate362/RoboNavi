
#ifndef ERR_FUNC_H_
#define ERR_FUNC_H_

#include "geometry2D.hpp"
#include <cmath>

namespace Geom2D {



inline Pose _declspec(dllexport) robot_encoder_err(const Pose& dpose){
	Pose pose_err;
	pose_err.p.x = 0.1*dpose.p.x;
	pose_err.p.y = 0.1*dpose.p.y;
	pose_err.phi = 0.1*dpose.phi;

	return pose_err;

}


inline void _declspec(dllexport) robot_pose_err_radius(const Pose& pose_err, vector<Point>& PolarData, vector<double>& dmin_R){//result in dmin_R

	int s = PolarData.size();
    dmin_R.resize(s);

	double del_1 = sqrt( pow(pose_err.p.x,2) + pow(pose_err.p.y,2) );
	double del_2 = 0;

	for(int i=0;i<s;++i){
		del_2 = fabs(PolarData[i].x*pose_err.phi);
		dmin_R[i] = del_1+del_2;
		//dmin_R[i] = 0;
	}
}

inline void _declspec(dllexport) Laser_measure_err_radius(vector<Point>& PolarData, vector<double>& dmin_Z){

	int s = PolarData.size();
    dmin_Z.resize(s);

	double del_1 = 0;
	double del_2 = 0;

	double dth1;
	double dth2;

	//compute dth, save in dmin_Z
	if(s>=2){
		dmin_Z[0] = fabs(PolarData[1].y - PolarData[0].y);
		dmin_Z[s-1] = fabs(PolarData[s-2].y - PolarData[s-1].y);
	}
	for(int i=1;i<s-1;++i){
		dth1 = fabs(PolarData[max(i-1,0)].y - PolarData[i].y);
		dth2 = fabs(PolarData[min(i+1,s-1)].y - PolarData[i].y);
		dmin_Z[i] = max(dth1,dth2);
	}

    //Compute error
	for(int i=0;i<s;++i){
		del_1 = 2*0.01*PolarData[i].x;
		del_2 = 0.5*dmin_Z[i]*PolarData[i].x;
		dmin_Z[i] = sqrt(pow(del_1,2)+pow(del_2,2));
		dmin_Z[i] = min(0.5,dmin_Z[i]);
		dmin_Z[i] = max(0.1,dmin_Z[i]);
	}

}


} // namespace Geom2D 

#endif

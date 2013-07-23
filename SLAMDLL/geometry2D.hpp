/* Simple 2D geometric operations with points, poses, and lines.
 *
 * These algorithms were worked out by me from first principles several years
 * ago. They work ok, but are not particularly good implementations. I recommend
 * the geometric source-code and resources by David Eberly on the "Magic Software"
 * website: www.magic-software.com.
 *
 * Tim Bailey 2004.
 */
#ifndef GEOMETRY_2D_H_
#define GEOMETRY_2D_H_

#include <vector>
#include <cmath>
//#include <cv.h>
//#include <highgui.h>
#include <stdio.h>
#include <conio.h>
#include <time.h>

using namespace std;

namespace Geom2D {

//
// Basic Structures
//

struct _declspec(dllexport) Point {
	double x;
	double y;
	double size;
	double relative_range;
	double relative_rad;
	int index;
	bool local_minima;
};

struct _declspec(dllexport) Pose {
	Point p;
	double phi;
};

struct _declspec(dllexport) Line {
	Point first;
	Point second;
};
struct _declspec(dllexport) Occupied_ID {
	double used_ID[6];
};
struct _declspec(dllexport) Pedestrian {
	//bool leg_candidate;
	double x;
	double y;
	int id;
	double size;
	double appear_time;
	double present;
	double Tmean[4];
	double Tsigma[16];
};
//
// Utility functions
//

const double PI = 3.14159265358979;

inline 
double sqr(double x) { return x*x; }

inline double round(double dInput)
{
    if(dInput >= 0.0f)
    {
        return floor(dInput + 0.5f);
    }
    return ceil(dInput - 0.5f);
}

template<class T>
inline
void swap(T& a, T& b) {
	T tmp(a);
	a = b;
	b = tmp;
}

inline
double pi_to_pi(double angle) { // normalise an angle to within +/- PI
	while (angle < -PI)
		angle += 2.*PI;
	while (angle > PI)
		angle -= 2.*PI;
	return angle;
}

//
// Point and Pose algorithms
//

inline 
double _declspec(dllexport) dist_sqr(const Point& p, const Point& q) { // squared distance between two Points
	return (sqr(p.x-q.x) + sqr(p.y-q.y));
}

inline
double _declspec(dllexport) dist(const Point& p, const Point& q)
{ 
	return sqrt(dist_sqr(p,q)); 
}


//
// Line algorithms
//

bool intersection_line_line (Point& p, const Line& l, const Line& m);
double distance_line_point (const Line& lne, const Point& p);
void intersection_line_point(Point& p, const Line& l, const Point& q);

//
// Basic transformations on 2-D Points (x,y) and Poses (x,y,phi).
//

class _declspec(dllexport) Transform2D {
public:
	Transform2D(const Pose& ref);

	void SetBase(const Pose& ref);
	void transform_to_relative(Point &p);
	void transform_to_relative(Pose &p);
	void transform_to_global(Point &p);
	void transform_to_global(Pose &p);

	void transform_to_global_vector(vector<Point>& Data);

private:
	Pose base;
	double c;
	double s;
};

inline
void Transform2D::transform_to_relative(Point &p) {//p<= R(-th)(p-t), global coordinate --> transform coordinate
	p.x -= base.p.x; 
	p.y -= base.p.y;
	double t(p.x);
	p.x = p.x*c + p.y*s;
	p.y = p.y*c -   t*s;
}

inline
void Transform2D::transform_to_global(Point &p) {//p <= R(th)*p + t, transform coordinate --> global coordinate
	double t(p.x); 
	p.x = base.p.x + c*p.x - s*p.y;
	p.y = base.p.y + s*t   + c*p.y;
}

inline
void Transform2D::transform_to_global_vector(vector<Point>& Data) {
	int n = Data.size();
	for(int i=0;i<n;++i){
		transform_to_global(Data[i]);
	}

}

inline
void Transform2D::transform_to_relative(Pose &p) {
	transform_to_relative(p.p);
	p.phi= pi_to_pi(p.phi-base.phi);
}

inline
void Transform2D::transform_to_global(Pose &p) {
	transform_to_global(p.p);
	p.phi= pi_to_pi(p.phi+base.phi);
}

inline void _declspec(dllexport) DataPolarToXY(vector<Point>& PolarData){

	int s = PolarData.size();
	double range;
	double ang;

	for(int i=0;i<s;++i){
		range = PolarData[i].x;
		ang = PolarData[i].y;
		PolarData[i].x = range*cos(ang);
		PolarData[i].y = range*sin(ang);
	}

}

void _declspec(dllexport) DoubleToPointArray(vector<Point>& data, vector<double>& range, vector<double>& rad);


inline void _declspec(dllexport) Point_vector_copy(const vector <Point>& REF,vector <Point>& DIST){
	if(DIST.size() != REF.size())
		DIST.resize(REF.size());
	
	copy(REF.begin(),REF.end(),DIST.begin());
}

inline
void _declspec(dllexport)  PointTrans(Point& p, Pose res){
	double c = cos(res.phi);
	double s = sin(res.phi);
	double x;
	double y;

	//R*p2 + t = p2

		x = c*p.x - s*p.y + res.p.x;
		y = s*p.x + c*p.y + res.p.y;
		p.x = x;
		p.y = y;
}

inline
void _declspec(dllexport)  VectorTrans(vector<Point> &data, Pose res){

	vector<Point>::iterator it;

	//R*p2 + t = p2
	for(it = data.begin(); it!=data.end(); ++it){
		PointTrans(*it, res);
	}
}



} // namespace Geom2D

#endif

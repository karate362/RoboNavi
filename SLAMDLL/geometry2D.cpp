/* Simple 2D geometric operations with points, poses, and lines.
 * Tim Bailey 2004.
 */
#include "stdafx.h"
#include "geometry2D.hpp"
#include <cmath>
#include <cassert>

using namespace std;

namespace Geom2D {

#if 0
double pi_to_pi(double angle) 
// An alternative implementation that uses fmod() rather than while-loops.
{
	angle = fmod(angle, 2.*PI);
	if (angle < -PI)
		angle += 2.*PI;
	else if (angle > PI)
		angle -= 2.*PI;
	return angle;
}
#endif


Transform2D::Transform2D(const Pose& ref) : base(ref) 
{
	c = cos(ref.phi);
	s = sin(ref.phi);
}

void Transform2D::SetBase(const Pose& ref)
{
		base = ref;
		c = cos(ref.phi);
		s = sin(ref.phi);
	}

bool intersection_line_line (Point& p, const Line& l, const Line& m) 
// Compute the intersection point of two lines.
// Returns false for parallel lines.
{
	double gl, gm, bl, bm;
	bool lVert = true, mVert = true;

	// calculate gradients 
	if ((gl = l.second.x - l.first.x) != 0.0) {
		gl = (l.second.y - l.first.y)/gl;
		lVert = false;
	}
	if ((gm = m.second.x - m.first.x) != 0.0) {
		gm = (m.second.y - m.first.y)/gm;
		mVert = false;
	}

	if (lVert == mVert) { // check for parallelism 
		if (gl == gm)
			return false;
	}

	bl = l.first.y - gl*l.first.x; // calculate y intercepts 
	bm = m.first.y - gm*m.first.x;

	if (lVert) { // calculate intersection 
		p.x = l.first.x;
		p.y = gm*p.x + bm;
	}
	else if (mVert) {
		p.x = m.first.x;
		p.y = gl*p.x + bl;
	}
	else {
		p.x = (bm - bl)/(gl - gm);
		p.y = gm*p.x + bm;
	}

	return true;
}

double distance_line_point (const Line& lne, const Point& p)
// Note: distance is -ve if point is on left of line and +ve if it is on 
// the right (when looking from first to second). 
{
	Point v;
	v.x= lne.second.x - lne.first.x;
	v.y= lne.second.y - lne.first.y;

	return ((lne.second.y - p.y)*v.x 
		  - (lne.second.x - p.x)*v.y) 
		  / sqrt(v.x*v.x + v.y*v.y);
}

void intersection_line_point(Point& p, const Line& l, const Point& q) 
// Compute the perpendicular intersection from a point to a line
{
	Line m; 	// create line through q perpendicular to l
	m.first = q;
	m.second.x = q.x + (l.second.y - l.first.y);
	m.second.y = q.y - (l.second.x - l.first.x);

	bool not_parallel = intersection_line_line(p, l, m);
	assert(not_parallel);
}



void DoubleToPointArray(vector<Point>& data, vector<double>& range, vector<double>& rad){
	int i=0;
	int s=range.size();
	data.resize(s);

	for(i=0;i<s;++i){
		data[i].x=range[i];
		data[i].y=rad[i];
	}
}



} // namespace Geom2D

#pragma once
#include "geometry2D.hpp"
#include "Error_function.h"
#include <math.h>
using namespace std;

namespace Geom2D {
class _declspec(dllexport) Filter_ICP
{
public:
	Filter_ICP(void);
	~Filter_ICP(void);

	Pose DoICP(const vector<Point>& Model,const vector<Point>& Data,const Pose& init,int trys,vector<double>& dmin_robot,vector<double>& dmin_sensor);

	Pose ICP_Least_Square(const std::vector<Point>& A, const std::vector<Point>& B);
	void Find_Nearest(const std::vector<Point>& A, const std::vector<Point>& B);
	int inline Find_Nearest_Node(const std::vector<Point>& Model, const Point& n);

	double inline Compute_Distance(const Point& n1,const Point& n2){

		double dx = n1.x-n2.x;
		double dy = n1.y-n2.y;
		return sqrt(dx*dx + dy*dy);
	}


private:
	vector<Point> MODEL;
	vector<Point> DATA;
	vector<Point> nA;//rearranged A, depicts nearest neighbor
	vector<Point> nB;

	vector<double> dmin_S;//sum of dmin

	double Rcov;//convergence factor;
	Pose initpose;
	Pose icpose;
};

};//Geom2D
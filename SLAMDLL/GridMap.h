#pragma once
#include "geometry2D.hpp"
#include <vector>


//Work with GridMap class:

//	GridMap* gmap = new GridMap();  //should use (new) to avoid stack overflow
//  Call Grid_Mapping(const Geom2D::Pose& rpose,const std::vector<double>& range,const std::vector<double>& rad);
//  drange and drad should be computed!!

class _declspec(dllexport) GridMap
{
public:
	GridMap(double x=0,double y=0,double ang=0);
	~GridMap(void);

	//Get attributes

	int Width(){return width;}
	int Height(){return height;}
	int size(){return Msize;}

	double* getGMAP(){return GMAP;}
	double* getLMAP(){return LMAP;}

	double getGrid(int w,int h){return GMAP[getidx(w,h)];}

    double getLdist(int w,int h){//
		if(gridstate(w,h) != -2)//In the map
			return LMAP[getidx(w,h)];
		else
			return 1000;
	}

	Geom2D::Point getNp(int w,int h){return NPs[getidx(w,h)];}
	Geom2D::Pose getPose(){return MapPose;}
	void setPose(Geom2D::Pose& mp){MapPose = mp;}

	//set attributes
	void ResetMap(){memset(GMAP,0,Msize*8);}
	void setGrid(int w, int h, double v){GMAP[getidx(w,h)] = v;}
	void setNp(int w, int h, Geom2D::Point np){NPs[getidx(w,h)] = np;}
	void TransformMap(Geom2D::Pose np,GridMap& nmap);// set the map center to np and copy to the nmap


	bool InMAP(int w, int h){
		return w>=0 && w<width && h>=0 && h<height;
	}

	//Draw Map functions
	inline void GridToXY(int w,int h,double& x, double& y);//(w,h) change to Global (x,y) according to MapPose
    void XYToGrid(double x,double y,int& w,int &h);


	void Find_Beam_Range(const std::vector<double>& range,const std::vector<double>& rad);
	double inv_laser_model(Geom2D::Point gp,const Geom2D::Pose& rpose,const std::vector<double>& range,const std::vector<double>& rad);
	double build_conspace(Geom2D::Point gp,const Geom2D::Pose& rpose,const std::vector<double>& range,const std::vector<double>& rad);	
	double set_conspace(Geom2D::Point lp);

	void Grid_Mapping(const Geom2D::Pose& rpose,const std::vector<double>& range,const std::vector<double>& rad);
	void Grid_Mapping2(const Geom2D::Pose& rpose,const std::vector<double>& range,const std::vector<double>& rad);
	double inv_laser_model_DATMO(Geom2D::Point gp,const Geom2D::Pose& rpose,const std::vector<double>& range,const std::vector<double>& rad,const std::vector<int>& type);
	void Grid_Mapping_DATMO(const Geom2D::Pose& rpose,const std::vector<double>& range,const std::vector<double>& rad,const std::vector<int>& type);

	void FindNN(int w,int h,int dw,int dh); //search in w+-dw, h+-dh
	void LikelihoodMap(double dlim);//Compute likelihood map
	int gridstate(int w,int h);

	void SaveGMAP(FILE* out);//assume that width == height
	void SaveLMAP(FILE* out);
	void LoadGMAP(FILE* in);
	void LoadLMAP(FILE* in);


private:

	//Data
	static const int width = 200;
	static const int height = 200;
	static const int Msize = width*height;

	Geom2D::Pose MapPose;//MAP equals a 3D feature, it corresponds to (width/2,height/2)
	static const double scale;//5cm

	std::vector<double> drange;
	std::vector<double> drad;//Used for drawing grid map 

	//Functions
	inline int getidx(int w,int h){return h*width+w;}

private:
	double GMAP[Msize];//in log form
	double LMAP[Msize];//distance
    Geom2D::Point NPs[Msize];//Nearest neighbor
	std::vector<double> readbuf;

};


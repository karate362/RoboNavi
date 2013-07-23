#pragma once
#include "ICP_Odemetry.h"
#include "GridMap.h"



class _declspec(dllexport) DATMO
{
public:
	DATMO(void);
	~DATMO(void);

	//void CreateNewMap();
	void UpdateMap();
	void DetectMOs_dummy(Geom2D::Pose& np, vector<double>& range, vector<double>& rad);
	void DetectMOs(Geom2D::Pose& np, vector<double>& range, vector<double>& rad);

	void ClusterMOs(vector<Geom2D::Point>& MOs);
	void DetectPedestrian(vector<Geom2D::Point>& CMOs, vector<Geom2D::Pedestrian>& current_pedestrian);
	void TrackingPedestrian(vector<Geom2D::Pedestrian>& tracked_pedestrian, vector<Geom2D::Pedestrian>& current_pedestrian, Geom2D::Occupied_ID & current_ID);
	
	
	void DATMO::filter(vector<Geom2D::Pedestrian>& current_pedestrian, Geom2D::Occupied_ID & current_ID);
	void DATMO::writePosition(vector<Geom2D::Pedestrian>& current_pedestrian);

	void getStaticReadings(vector<double>& range, vector<double>& rad);
	void getMOs(vector<Geom2D::Point>& MOs);
	void getCenter(Geom2D::Point & center);
	void getCMOs(vector<Geom2D::Point>& CMOs);
	GridMap* getGMAP(){return gmap;}

//public:
//	DATMODlg datmodlg;

private:

	GridMap* gmap;//Gridmap, used to record last n readings, it should be extended to "Gridmap sets"
	GridMap* buffermap;
	int nowidx;// New scan will be written in here
	static const int snum = 4;//The grid map is created by 5 scans
	
	const double x_offset ;//map center
    const double y_offset ;//map center

	vector<Geom2D::Pose> poses;
	vector<vector<double>> ranges;
	vector<vector<double>>rads;


	vector<double> Lastrange;
	vector<double> Lastrad;//Save the last range and rad
	vector<int> rtype;//type of the last reading, 0:static,1:moving,-1:unknown-->occupied
    Geom2D::Pose Lastpose;

    vector<Geom2D::Point> MOposes;
	vector<Geom2D::Point> MOsizes;

/*
	vector<double> morange;
	vector<double> morad;//Moving objects

	vector<double> strange;
	vector<double> strad;//Static objects
	Geom2D::Pose stpose;
	Geom2D::Pose mopose;

	//vector<Geom2D::Point> MOs;//moving objects in (x,y) form
*/
};

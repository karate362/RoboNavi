#pragma once
#include "GridMap.h"
#include "windows.h"
#include "mmsystem.h"

class _declspec(dllexport) KFUpdate
{
public:
	void KFInitialize(Geom2D::Pedestrian & new_pedestrian);
	void measurementUpdate(Geom2D::Pedestrian & current_pedestrian, double z_measurement[]);
	void timeUpdate(Geom2D::Pedestrian & current_pedestrian, DWORD scan_time);
};
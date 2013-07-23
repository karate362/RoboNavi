#pragma once
#include "stdafx.h"
#include "windows.h"
#include "mmsystem.h"
#include <vector>

bool StartSimLaser(void* laser);
bool StartLMS100Laser(void* laser);

void getSimLaser(void* laser,std::vector<double> &range,std::vector<double> &rad);
void getLMS100Laser(void* laser,std::vector<double> &range,std::vector<double> &rad);

void StopSimLaser(void* laser);
void StopLMS100Laser(void* laser);

void SaveSimLaser(void* robot, void* laser, void* output);
void SaveLMS100Laser(void* robot, void* laser, void* output);
void ICPOutput(void* robotpt, std::vector<double> laserpt, FILE* outputpt);

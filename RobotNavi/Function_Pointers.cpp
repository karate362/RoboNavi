#include "stdafx.h"
#include "Function_Pointers.h"
#include "RobotNavi.h"
#include "RobotDlg.h"
#include "math.h"

vector <double> Global_Laser_Range;
vector <double> Global_Laser_Ang;


//Functions for dealing with Simulated Laser
bool StartSimLaser(void* laser){//Initial the simulated sick laser, return false if connecting fails
	ArSick &sick = *(ArSick*)laser;

	return (!sick.blockingConnect());

}

void getSimLaser(void* laser,std::vector<double> &range,std::vector<double> &rad){//Put the readings into range and rad
	ArSick &sick = *(ArSick*)laser;

	const std::list<ArSensorReading *> *readingsList; // Instantiate a list of sensor readings
    std::list<ArSensorReading *>::const_iterator it; // Instantiate an iterator object for the list

	range.clear();
	rad.clear();

	sick.lockDevice();///LOCK
	
	readingsList = sick.getRawReadings(); // Get the list of readings	

	for (it = readingsList->begin(); it != readingsList->end(); it++){ // Loop through readings
		rad.push_back( (*it)->getSensorTh() * 3.1415926/180 );
		range.push_back( (double)((*it)->getRange())/1000 );
	}

	sick.unlockDevice();//UNLOCK


}

void StopSimLaser(void* laser){
	ArSick &sick = *(ArSick*)laser;
	sick.stopRunning();
}



//Functions for dealing with LMS100 Laser
bool StartLMS100Laser(void* laser){
	LaserDlg* ldlg = (LaserDlg*)laser;

	return ldlg->StartLaser();
}


void getLMS100Laser(void* laser,std::vector<double> &range,std::vector<double> &rad){
	LaserDlg* ldlg = (LaserDlg*)laser;

	ldlg->getLaser(range,rad);
}


void StopLMS100Laser(void* laser){
	LaserDlg* ldlg = (LaserDlg*)laser;

	ldlg->StopLaser();
}



void SaveSimLaser(void* robotpt, void* laserpt, void* outputpt){
	P3robot &probot = *(P3robot*)robotpt;
    ArSick &sick = *(ArSick*)laserpt;
	FILE* out = (FILE*)outputpt;

	probot.SimLaserOutput(out);
}

void SaveLMS100Laser(void* robotpt, void* laserpt, void* outputpt){
	P3robot &probot = *(P3robot*)robotpt;
	LaserDlg* ldlg = (LaserDlg*)laserpt;
	FILE* out = (FILE*)outputpt;

	fprintf(out,"%.3f %.3f %.3f ",probot.getX()/1000,probot.getY()/1000,probot.getThRad());//m scale

	ldlg->SaveLaser(out);


}

void ICPOutput(void* robotpt, vector<double> laserpt, FILE* outputpt){
	P3robot &probot = *(P3robot*)robotpt;
	//LaserDlg ldlg = laserpt;
	FILE* out = outputpt;

	fprintf(out,"%.3f %.3f %.3f ",probot.getX()/1000,probot.getY()/1000,probot.getThRad());//m scale

	//ldlg->SaveLaser(out);


}
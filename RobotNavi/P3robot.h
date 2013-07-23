#pragma once

#include "stdafx.h"
#include "Aria.h"
#include "windows.h"
#include "mmsystem.h"
#include <stdio.h>

DWORD WINAPI ODEMETRY_THREAD (LPVOID num);


class P3robot
{
public:
	friend DWORD WINAPI ODEMETRY_THREAD (LPVOID num);
	P3robot(void);
	~P3robot(void);

	void Connect();
	void DisConnect();

	void SetOdemetry(double x,double y,double ang);

	void SimLaserOutput(FILE* out);

	ArPose getPose(){return pose;}
	double getX(){return pose.getX();}
	double getY(){return pose.getY();}
	double getTh(){return pose.getTh();}
	double getThRad(){return pose.getThRad();}
	double getV(){return robot.getVel()/1000;}
	double getW(){return ArMath::degToRad(robot.getRotVel());}

	bool Connected(){return IsConnected;}

public:
	// robot
    ArRobot robot;
    // a laser in case one is used
    ArSick sick;
    ArSimpleConnector* simpleConnector;
    // a key handler so we can do our key handling
    ArKeyHandler keyHandler;
    // sonar, must be added to the robot, for teleop and wander
    ArSonarDevice sonarDev;
    ArAnalogGyro* gyro;

private:
	void StartOdemetryThread();

private:
	ArPose pose;

	HANDLE hThread; //Odemetry thread
    DWORD dwThreadID;
	bool IsThread;
	bool IsConnected;
};

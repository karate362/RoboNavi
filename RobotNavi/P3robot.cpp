#include "StdAfx.h"
#include "P3robot.h"
#include "Thread_Functions.h"

DWORD WINAPI ODEMETRY_THREAD (LPVOID num){//compute the odemetry
    DWORD ptime = 0;
	DWORD time = 0;
	P3robot &probot = *((P3robot*)num); 
	ArRobot &robot = probot.robot;
    bool &IsThread = probot.IsThread;

	double v = 0;
	double w = 0;

	double x = 0;
	double y = 0;
	double ra = 0;
	double dt = 0;

	probot.pose.setX(0);
	probot.pose.setY(0);
	probot.pose.setThRad(0);

	ptime = timeGetTime();

	while(IsThread){//assumption: the robot maintained the same velocity in the 50ms
		v = robot.getVel();//mm/s
		w = ArMath::degToRad(robot.getRotVel());

		x = probot.pose.getX();
		y = probot.pose.getY();
		ra = probot.pose.getThRad();

		time = timeGetTime();
		dt = (double)(time-ptime)/1000;
		ptime = time;

		probot.pose.setX(x + v*dt*cos(ra+w*dt/2));
		probot.pose.setY(y + v*dt*sin(ra+w*dt/2));
		probot.pose.setThRad(ra + w*dt);

		Sleep(50);
		
	}

	return 0;
}


P3robot::P3robot(void)
{
	pose.setX(0);
	pose.setY(0);
	pose.setThRad(0);
	IsConnected = false;
}

P3robot::~P3robot(void)
{
	this->DisConnect();
	Aria::shutdown();
}


void P3robot::Connect(){

  int argc=0; 

  char** argv=0;

  // set up our simpleConnector
  simpleConnector = new ArSimpleConnector(&argc, argv);

  // parse its arguments
  simpleConnector->parseArgs();

  // if there are more arguments left then it means we didn't
  // understand an option
  if (argc > 1)
  {    
    simpleConnector->logOptions();
    keyHandler.restore();
    exit(1);
  }

  // mandatory init
  Aria::init();
  //ArLog::init(ArLog::StdOut, ArLog::Verbose);

  // let the global aria stuff know about it
  Aria::setKeyHandler(&keyHandler);
  // toss it on the robot
  robot.attachKeyHandler(&keyHandler);


  //sonarDev.setCumulativeBufferSize(32);

  // add the sonar to the robot
  robot.addRangeDevice(&sonarDev);
  // add the laser to the robot
  robot.addRangeDevice(&sick);
  //sick.setCumulativeBufferSize(721);
  // add a gyro, it'll see if it should attach to the robot or not
  gyro=new ArAnalogGyro(&robot);

  // set up the robot for connecting
  if (!simpleConnector->connectRobot(&robot))
  {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    keyHandler.restore();
	IsConnected = false;
    return ;
  }

  	IsConnected = true;

  // set up the laser before handing it to the laser mode
  simpleConnector->setupLaser(&sick);
  // turn on the motors
  robot.comInt(ArCommands::ENABLE, 1);
  robot.comInt(ArCommands::JOYDRIVE, 1);
    // run the robot in its own thread, so it gets and processes packets and such
  robot.runAsync(true);

  this->StartOdemetryThread();

}

void P3robot::DisConnect(){
	IsConnected = false;
	EndThread(hThread,IsThread);
	robot.disconnect();
}

void P3robot::SetOdemetry(double x,double y,double ang){
	pose.setX(x);
	pose.setY(y);
	pose.setThRad(ang);
}

void P3robot::StartOdemetryThread(){
	// TODO: Add your control notification handler code here
	DWORD ExitCode;
	this->IsThread = false;//end the thread

	do{GetExitCodeThread(hThread,&ExitCode);}        
	while(ExitCode == STILL_ACTIVE);//wait until the thread is over

	IsThread = true;
	hThread=CreateThread(NULL,0, ODEMETRY_THREAD,(LPVOID)this, 0,&dwThreadID);//thread start
 
}

void P3robot::SimLaserOutput(FILE* out){
    const std::list<ArSensorReading *> *readingsList; // Instantiate a list of sensor readings
    std::list<ArSensorReading *>::const_iterator it; // Instantiate an iterator object for the list

	sick.lockDevice();

	fprintf(out,"%.3f %.3f %.3f ",pose.getX()/1000,pose.getY()/1000,pose.getThRad());

	readingsList = sick.getRawReadings();
	for (it = readingsList->begin(); it != readingsList->end(); it++){ // Loop through readings
		fprintf(out,"%.3f ",(double)((*it)->getRange())/1000);
	}
	fprintf(out,"\n");

	sick.unlockDevice();
}
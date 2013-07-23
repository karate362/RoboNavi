// RNavDlg.cpp : implementation file
//

#include "stdafx.h"
#include "PCtrl.h"
#include "RNavDlg.h"
#include "DWAstar.h"
#include "NaviDraw.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

DCB global_dcb;//RS232
MFC_VIEWER view;//viewer
HANDLE hMutex;//lock object
ObsArray Lobs;
ND Fnd;
int* sonar_count;
double Safety_Reg;

void getLaser(ArSick *thisLaser, ObsArray& obs);
void getLaser2(LaserThread& Lthread, ObsArray& obs);
void getSonar(ArRobot& robot, ObsArray& obs);

/////////////////////////////////////////////////////////////////////////////
// RNavDlg dialog
DWORD WINAPI SETUP_THREAD (LPVOID num);
DWORD WINAPI ROBOT_THREAD (LPVOID num);
DWORD WINAPI LASER_THREAD (LPVOID num);
DWORD WINAPI SONAR_THREAD (LPVOID num);
DWORD WINAPI LASER_THREAD2 (LPVOID num);//Real S200
////////////////////////////////////////////////////////////////
double global_gx;
double global_gy;

void RNavDlg::Ctrl_START()
{

int portnum;

HANDLE hThread1;  
DWORD dwThreadID1;

HANDLE hThread2;  
DWORD dwThreadID2;

HANDLE hThread3;  
DWORD dwThreadID3;

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
    return ;
  }

  // set up the laser before handing it to the laser mode
  simpleConnector->setupLaser(&sick);
  // turn on the motors
  robot.comInt(ArCommands::ENABLE, 1);
  robot.comInt(ArCommands::JOYDRIVE, 1);
    // run the robot in its own thread, so it gets and processes packets and such
  robot.runAsync(false);


  if(sensor == 0)
  {
	Lobs.init(50,robot.getNumSonar() * 5,0.3);
	hThread3=CreateThread(NULL,0,SONAR_THREAD,(LPVOID)this,0,&dwThreadID3);//Sensor read thread
  }
  else
  if(sensor == 1)
  {
  //Run Laser
	sick.runAsync();
	if (!sick.blockingConnect())
	{
    printf("Couldn't connect to sick, exiting\n");
    Aria::shutdown();
    return ;
	}
	Lobs.init(80,361,0.28);
	hThread2=CreateThread(NULL,0,LASER_THREAD,(LPVOID)this,0,&dwThreadID2);//Sensor read thread
  }
  else
  if(sensor == 2)
  {
	portnum = GetDlgItemInt(IDC_EDIT4);

	//Run Laser
	if(!Lthread.RS232_init(portnum,38400,global_dcb))
	{
		SetDlgItemText(IDC_EDIT3,"RS232 Fall!");
		//Sleep(1000);
		//exit(0);
	}
	else
	if(!Lthread.GetT())
	{
		SetDlgItemText(IDC_EDIT3,"Get Token Fall!");
		//Sleep(1000);
		//exit(0);
	}
	else
	{
	Lobs.init(80,721,0.28);
	hThread2=CreateThread(NULL,0,LASER_THREAD2,(LPVOID)this,0,&dwThreadID2);//Sensor read thread
	}

	}

  hMutex = CreateMutex(NULL,false,"ThreadSync");  
  hThread1=CreateThread(NULL,0,ROBOT_THREAD,(LPVOID)this,0,&dwThreadID1);//Robot control thread

}

DWORD WINAPI ROBOT_THREAD (LPVOID num)

{

 //char* stop = (char*)num;
 RNavDlg* rdlg = (RNavDlg*)num;
 ArRobot &robot = rdlg->robot;

//DWA variable
  DWAstar dywstar;
  DWAnode nownode;
  DyWin dyw;
  DyWin* DwSave;

  double at;
  double sa;
  double sm;
  int ma;
  int hur;

  double vi = 0;
  double wi = 0;
  double rv = 0;
  double rw = 0;

  double rx = 0;
  double ry = 0;
  double rth = 0;

  double dx = 0;
  double dy = 0;
  double dth = 0;

  double gx = 0;
  double gy = 0;
  double lgx = 0;
  double lgy = 0;

//DWA variable

//ND variable
  ND nd;
//ND variable

//////////////存放記錄資料夾//////////////////

ofstream path_rec;
ofstream obs_rec;
ofstream reg_rec;
ofstream itv_rec;
ofstream tree_rec;

char sensor[16];
char alg[16];
char date[256];
char bmpname[256];
char txtname[256];
time_t   timep;   
struct tm *tp;   

if(rdlg->is_mkdir)
{
if(rdlg->algorithm == 0)//DWA
sprintf(alg,"DWA");
if(rdlg->algorithm == 1)//DWA*
sprintf(alg,"DWAstar_%d",rdlg->sdepth);
if(rdlg->algorithm == 2)//ND
sprintf(alg,"ND");

if(rdlg->sensor == 0)//sonar
sprintf(sensor,"SONAR");
if(rdlg->sensor == 1)//laser
sprintf(sensor,"LASER");

time(&timep);   
tp=localtime(&timep); 
sprintf(date,"%s_%s_%d_%d_%d_%d_%d_%d",alg,sensor,1900+tp->tm_year,1+tp->tm_mon,tp->tm_mday,tp->tm_hour,tp->tm_min,tp->tm_sec);
mkdir(date);

sprintf(txtname,"%s/%s",date,"path.txt");
path_rec.open(txtname);

sprintf(txtname,"%s/obj_weight.txt",date);
CopyFile("obj_weight.txt",txtname,false);

sprintf(txtname,"%s/parameters.txt",date);
CopyFile("parameters.txt",txtname,false);
}

//////////////////////////////////////////////

//Other
  char velstr[16];
  DWORD p_time = 0;//上次時間
  DWORD c_time = 0;//計算時間
  int Sleeptime;
  int policy = 0;
  int draw_ratio = 25;
//Other

//////////////////////////////Init////////////////////////////////
ifstream in1("DWAp.txt");
ifstream in2("obj_weight.txt");
ifstream in3("DWAstarp.txt");

if(rdlg->algorithm==0)
{
DwSave = &dyw; 
dyw.init(in1,in2,&(Lobs.obs));
Sleeptime=(int)(dyw.delta_t*1000.0);
}
else
if(rdlg->algorithm==1)
{
DwSave = &(dywstar.dyw);
dywstar.initDWA(in1,in2);
dywstar.ndptr = &Fnd;
Sleeptime=(int)(dywstar.dyw.delta_t*1000.0);

in3>>sa>>at>>sm>>ma>>hur;

if(rdlg->sensor == 0)//sonar
dywstar.initDWAstar(at,sm,sa,ma,hur);
else
if(rdlg->sensor == 1)//simulation laser
dywstar.initDWAstar(at,sm,0,ma,hur);
else//其他laser
dywstar.initDWAstar(at,sm,0,ma,hur);

}
gx = global_gx;
gy = global_gy;

//////////////////////////////Init////////////////////////////////

if(rdlg->is_path)
path_rec<<gx*1000.0<<" "<<gy*1000.0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<endl; 


//等待到有第一筆sensor data進來才開始運行
while(Lobs.obs.empty())
{Sleep(500);}


while(1)
{
//get time

////////////////////get new state variable
rv = (double)(robot.getVel())/1000;
rw = (double)(robot.getRotVel())*3.1415/180;

rx = (double)(robot.getX())/1000;//m
ry = (double)(robot.getY())/1000;//m
rth = (double)(robot.getTh());
rth = ArMath::degToRad(rth);//rad

GPtoLP(rx,ry,rth,gx,gy,lgx,lgy);//Compute the goal location

if( sqrt(lgx*lgx+lgy*lgy) < 0.3)//到達終點附近
{
robot.setVel(0);
robot.setRotVel(0);
rdlg->SetDlgItemText(IDC_EDIT1,"STOP");
Sleep(10000);
}

//get mutex
WaitForSingleObject(hMutex,INFINITE);

///////////對obs做矯正//////////
Lobs.tran_obs2(rx,ry,rth);
///////////對obs做矯正//////////


p_time = timeGetTime();
////////////////////////DWA/////////////////////////////////////////

/////ND FIND DOOR////////
Fnd.ResetReg();
Fnd.Find_Gap(0.5);
Fnd.Find_REGION(lgx,lgy);
Fnd.Find_Door(lgx,lgy,0.5);
/////ND FIND DOOR////////

if(rdlg->algorithm==0)
{
dyw.Region_Analysis(1.0,lgx,lgy);
policy = dyw.DyWin_search(vi,wi,rx,ry,rth,gx,gy,vi,wi);
}
else
if(rdlg->algorithm==1)
{
nownode.v = vi;
nownode.w = wi;
nownode.x = 0;
nownode.y = 0;
nownode.th = 0;
nownode.parent = -1;
nownode.depth = 0;
dywstar.DWAstarsearch(vi,wi,Lobs,nownode,rdlg->sdepth,lgx,lgy);
}

//policy = 0: BUG
//policy = 1: DWA

////////////////////////DWA/////////////////////////////////////////


robot.setVel(vi*1000);
robot.setRotVel(wi*180.0/3.1415);

c_time = timeGetTime() - p_time;
rdlg->SetDlgItemInt(IDC_EDIT1,c_time);

//release mutex
ReleaseMutex(hMutex);

//////////////////////SHOW////////////////////////////////////////

view.FillAll();

DrawRobotOnView(view,vi,wi,lgx,lgy,policy,draw_ratio);
DrawOBSOnView(view,Lobs,draw_ratio);
//DrawNDOnView(view,Fnd,VFHsize,draw_ratio);

sprintf(velstr,"%f",vi);
rdlg->SetDlgItemText(IDC_V,velstr); 
sprintf(velstr,"%f",wi);
rdlg->SetDlgItemText(IDC_W,velstr);

if(rdlg->algorithm==2)
DrawNDOnView(view,nd,VFHsize,draw_ratio);

if(rdlg->algorithm==1)
DrawDWATreeOnView(view,dywstar,0,0,0,draw_ratio);

rdlg->RePaint();

//////////////////////SHOW////////////////////////////////////////


/////////////////////record//////////////////////////////////////


p_time = timeGetTime();

if(rdlg->is_path)
path_rec<<robot.getX()<<" "<<robot.getY()<<" "<<ArMath::degToRad(robot.getTh())<<" "<<vi<<" "<<wi<<" "<<policy<<" "<<p_time<<endl; 

if(rdlg->is_obs)
{
sprintf(txtname,"%s/%d_obs.txt",date,p_time);
obs_rec.open(txtname);
Lobs.SaveObs(obs_rec);
obs_rec.close();
}

if(rdlg->is_itv)
{
sprintf(txtname,"%s/%d_itv.txt",date,p_time);
itv_rec.open(txtname);
DwSave->SaveITV(itv_rec);
itv_rec.close();
}

if(rdlg->is_reg)
{
sprintf(txtname,"%s/%d_reg.txt",date,p_time);
reg_rec.open(txtname);
DwSave->VR.Save_Reg(reg_rec);
reg_rec.close();
}

if(rdlg->is_tree)
{
sprintf(txtname,"%s/%d_tree.txt",date,p_time);
tree_rec.open(txtname);
dywstar.SaveTree(tree_rec);
tree_rec.close();
}

/////////////////////record//////////////////////////////////////


if(c_time<Sleeptime)
Sleep(Sleeptime - c_time);



}


return 0;


}


void getLaser(ArSick *thisLaser, ObsArray& obs)
{
const std::list<ArSensorReading *> *readingsList; // Instantiate a list of sensor readings
std::list<ArSensorReading *>::const_iterator it; // Instantiate an iterator object for the list
int i = -1; // Loop counter for readings
readingsList = thisLaser->getRawReadings(); // Get the list of readings

obs.setLastReadingTime(timeGetTime());

for (it = readingsList->begin(); it != readingsList->end(); it++) // Loop through readings
{
i++;
obs.set_obs_laser((*it)->getRange(),(*it)->getSensorTh(),0);
Fnd.Set_Intervals((*it)->getRange()/1000.0,(*it)->getSensorTh());
}

}

void getLaser2(LaserThread& Lthread, ObsArray& obs)
{
int i=0;
int s = Lthread.Readings.size();

obs.setLastReadingTime(Lthread.getLastReadingTime());

for(i=0;i<s;++i)
{

  if( fabs(Lthread.Readings[i].theta)<=90.0 )//先看前方180度即可
  {
	  obs.set_obs_laser(Lthread.Readings[i].range*1000,Lthread.Readings[i].theta,0);
	  Fnd.Set_Intervals(Lthread.Readings[i].range,Lthread.Readings[i].theta);
  }

}

}

void getSonar(ArRobot& robot, ObsArray& obs)
{

double lx=0;
double ly=0;
int numsonar=robot.getNumSonar();
int i=0;
ArSensorReading* SonarReading;
OBS nobs;


for(i=0;i<numsonar;++i)
{
SonarReading = robot.getSonarReading(i);

if(SonarReading->getRange() < 3000)//確實有讀到東西
{
  if(SonarReading->getCounterTaken() != sonar_count[i])//新讀值
  {
  lx = SonarReading->getLocalX()/1000.0;
  ly = SonarReading->getLocalY()/1000.0;
  obs.set_obs_sonar(lx,ly,0);
  obs.setLastReadingTime(timeGetTime());
  sonar_count[i] = SonarReading->getCounterTaken();
  }
//!!只要是同一顆sonar, 當counter更新時必定是新讀值
}

}

}


DWORD WINAPI LASER_THREAD(LPVOID laser)//模擬使用
{


// a laser in case one is used
ArSick& sick = ((RNavDlg*)laser)->sick;
// robot
ArRobot& robot = *(sick.getRobot());

ArTime lrtime;

time_t  pSec=0;
time_t  pMSec=0;

while(1)
{
//get mutex
WaitForSingleObject(hMutex,INFINITE);

lrtime = sick.getLastReadingTime();

if(pSec!=lrtime.getSec() || pMSec!=lrtime.getMSec())//得到不一樣的reading才紀錄下來
{
sick.lockDevice();//鎖住device 以免中途讀值改變
pSec = lrtime.getSec();
pMSec = lrtime.getMSec();

Lobs.x = robot.getX()/1000.0;//pose record
Lobs.y = robot.getY()/1000.0;//pose record
Lobs.th = robot.getTh()*PI/180.0;//pose record

Lobs.obs.clear();
Fnd.ResetVFH();
getLaser(&sick,Lobs);
sick.unlockDevice();

((RNavDlg*)laser)->SetDlgItemInt(IDC_EDIT3,pMSec);
}
//release mutex
ReleaseMutex(hMutex);


Sleep(1200);//test
//Sleep(0);

}

return 0;
}
/*
DWORD WINAPI LASER_THREAD2(LPVOID laser)//實際連結S200 LASER, 為求修正方便, 直接呼叫GetRawData而不使用其thread
{
//Laser
LaserThread& Lthread = ((RNavDlg*)laser)->Lthread;
// robot
ArRobot& robot = ((RNavDlg*)laser)->robot;

double rx;
double ry;
double rth;

int error;

while(1)
{
	rx = robot.getX()/1000.0;//pose record
	ry = robot.getY()/1000.0;//pose record
	rth = robot.getTh()*PI/180.0;//pose record

	if(Lthread.ReadRawData(error))//讀取成功
	{
	    ((RNavDlg*)laser)->SetDlgItemText(IDC_EDIT3,"Laser success");
		Lthread.TranData();//轉換data
	    //((RNavDlg*)laser)->SetDlgItemInt(IDC_EDIT1,Lthread.Readings.size());
		WaitForSingleObject(hMutex,INFINITE);//Get mutex
		Lobs.obs.clear();
        //Fnd.ResetAll();
		Lobs.x = rx;//記錄robot位置
		Lobs.y = ry;
		Lobs.th = rth;
		getLaser2(Lthread,Lobs);
		ReleaseMutex(hMutex);//Release mutex
	}
	else
	{
		((RNavDlg*)laser)->SetDlgItemText(IDC_EDIT3,"Laser error");
        //getLaser2(Lthread,Lobs);
	}

	Sleep(50);
}

return 0;
}*/


DWORD WINAPI LASER_THREAD2(LPVOID laser)//實際連結S200 LASER, 為求修正方便, 直接呼叫GetRawData而不使用其thread
{
//Laser
LaserThread& Lthread = ((RNavDlg*)laser)->Lthread;
// robot
ArRobot& robot = ((RNavDlg*)laser)->robot;

int error;

DWORD ptime;

Lthread.LaserStart();

while(1)
{
	if(ptime != Lthread.getLastReadingTime())//新讀值
	{		
		WaitForSingleObject(hMutex,INFINITE);//Get mutex

	    Lthread.LockDevice();//Lock device
        ptime = Lthread.getLastReadingTime();
		((RNavDlg*)laser)->SetDlgItemInt(IDC_EDIT3,ptime);//
		Lobs.obs.clear();
		Lobs.x = Lthread.rx;//記錄robot位置
		Lobs.y = Lthread.ry;
		Lobs.th = Lthread.rth;
		Fnd.ResetVFH();
		getLaser2(Lthread,Lobs);
	    Lthread.ReleaseDevice();

		ReleaseMutex(hMutex);//Release mutex
	}

	Sleep(50);
}

return 0;
}


DWORD WINAPI SONAR_THREAD(LPVOID num)
{

// a laser in case one is used
ArRobot& robot = ((RNavDlg*)num)->robot;
double rx;
double ry;
double rth;

double dx;
double dy;
double dth;
int i;

int numsonar=robot.getNumSonar();

sonar_count = new int[numsonar];
for(i=0;i<numsonar;++i)
sonar_count[i] = -1;

while(1)
{
//get mutex
WaitForSingleObject(hMutex,INFINITE);

rx = robot.getX()/1000.0;//pose record
ry = robot.getY()/1000.0;//pose record
rth = robot.getTh()*PI/180.0;//pose record

Lobs.tran_obs2(rx,ry,rth);

getSonar(robot, Lobs);


//release mutex
ReleaseMutex(hMutex);
Sleep(50);
}

return 0;
}

RNavDlg::RNavDlg(CWnd* pParent /*=NULL*/)
	: CDialog(RNavDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(RNavDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INIT
}


void RNavDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(RNavDlg)
		// NOTE: the ClassWizard will add DDX and DDV calls here
	//}}AFX_DATA_MAP
}


BEGIN_MESSAGE_MAP(RNavDlg, CDialog)
	//{{AFX_MSG_MAP(RNavDlg)
	ON_WM_PAINT()
	ON_WM_CREATE()
	//ON_BN_CLICKED(IDC_SAVE1, OnSave1)
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()


/////////////////////////////////////////////////////////////////////////////
// RNavDlg message handlers

BOOL RNavDlg::OnInitDialog()
{
   ((CButton*)GetDlgItem(IDC_RADIO2))->SetCheck(TRUE);//設定演算法
   ((CButton*)GetDlgItem(IDC_RADIOS2))->SetCheck(TRUE);//設定sensor 
   SetDlgItemInt(IDC_EDIT2,5);
   SetDlgItemInt(IDC_EDIT4,2);
   //UpdataData(false);   
   return 0;
}

void RNavDlg::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
   CClientDC dc(this);
   view.init(dc,250,250);
   view.FillAll();


}

void RNavDlg::OnPaint()
{

   CClientDC dc(this); 

   view.PasteOn(dc,0,0);

   CDialog::OnPaint();

}


void RNavDlg::OnOK() 
{
	CString str;
	GetDlgItemText(IDC_GX,str);
	global_gx = atof((LPCSTR)str);
	GetDlgItemText(IDC_GY,str);
	global_gy = atof((LPCSTR)str);

	is_mkdir=false;
	is_path=false;
	is_obs=false;
	is_reg=false;
	is_tree=false;

    CButton* check;

	check =(CButton*)this->GetDlgItem(IDC_CHECK1);
    if(check->GetCheck() == BST_CHECKED)
		is_path=true,is_mkdir=true;

	check =(CButton*)this->GetDlgItem(IDC_CHECK2);
    if(check->GetCheck() == BST_CHECKED)
		is_obs=true,is_mkdir=true;

	check =(CButton*)this->GetDlgItem(IDC_CHECK3);
    if(check->GetCheck() == BST_CHECKED)
		is_itv=true,is_mkdir=true;

	check =(CButton*)this->GetDlgItem(IDC_CHECK4);
    if(check->GetCheck() == BST_CHECKED)
		is_reg=true,is_mkdir=true;

	check =(CButton*)this->GetDlgItem(IDC_CHECK5);
    if(check->GetCheck() == BST_CHECKED)
		is_tree=true,is_mkdir=true;

    if(IsDlgButtonChecked(IDC_RADIO1))//DWA
		algorithm = 0,SetDlgItemText(IDC_EDIT1,"DWA");
	else
    if(IsDlgButtonChecked(IDC_RADIO2))//DWA*
	{
		algorithm = 1,SetDlgItemText(IDC_EDIT1,"DWA*");
		sdepth = GetDlgItemInt(IDC_EDIT2);
	}
	else
    if(IsDlgButtonChecked(IDC_RADIO3))//ND
		algorithm = 2,SetDlgItemText(IDC_EDIT1,"ND");


    if(IsDlgButtonChecked(IDC_RADIOS1))//Sonar
		sensor = 0;
	else
    if(IsDlgButtonChecked(IDC_RADIOS2))//Laser
		sensor = 1;
	else
    if(IsDlgButtonChecked(IDC_RADIOS3))//Laser
		sensor = 2;
	else
    if(IsDlgButtonChecked(IDC_RADIOS4))//Laser
		sensor = 3;

    this->Ctrl_START();
}




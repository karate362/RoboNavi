// RobotNavi.cpp : Defines the class behaviors for the application.
//

#include "stdafx.h"
#include "RobotNavi.h"
#include "MainFrm.h"

#include "RobotNaviDoc.h"
#include "RobotNaviView.h"

#include "JoyDlg.h"
#include "RobotDlg.h"
#include "LaserDlg.h"
#include "ICPDlg.h"
#include "MapDlg.h"
#include "NavDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CRobotNaviApp

BEGIN_MESSAGE_MAP(CRobotNaviApp, CWinApp)
	ON_COMMAND(ID_APP_ABOUT, &CRobotNaviApp::OnAppAbout)
	// Standard file based document commands
	ON_COMMAND(ID_FILE_NEW, &CWinApp::OnFileNew)
	ON_COMMAND(ID_FILE_OPEN, &CWinApp::OnFileOpen)
	// Standard print setup command
	ON_COMMAND(ID_FILE_PRINT_SETUP, &CWinApp::OnFilePrintSetup)
	ON_COMMAND(ID_FILE_ROBOTSTART, &CRobotNaviApp::OnFileRobotstart)
	ON_COMMAND(ID_FILE_LASERSTART, &CRobotNaviApp::OnFileLaserstart)
	ON_COMMAND(ID_FILE_ICPSTART, &CRobotNaviApp::OnFileIcpstart)
	ON_COMMAND(ID_FILE_DRAWMAP, &CRobotNaviApp::OnFileDrawmap)
	ON_COMMAND(ID_FILE_JOYSTICKSTART, &CRobotNaviApp::OnFileJoystickstart)
	ON_COMMAND(ID_FILE_NAVISTART, &CRobotNaviApp::OnFileNavistart)
END_MESSAGE_MAP()


// CRobotNaviApp construction

CRobotNaviApp::CRobotNaviApp()
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}


// The one and only CRobotNaviApp object

CRobotNaviApp theApp;


// CRobotNaviApp initialization

BOOL CRobotNaviApp::InitInstance()
{
	// InitCommonControlsEx() is required on Windows XP if an application
	// manifest specifies use of ComCtl32.dll version 6 or later to enable
	// visual styles.  Otherwise, any window creation will fail.
	INITCOMMONCONTROLSEX InitCtrls;
	InitCtrls.dwSize = sizeof(InitCtrls);
	// Set this to include all the common control classes you want to use
	// in your application.
	InitCtrls.dwICC = ICC_WIN95_CLASSES;
	InitCommonControlsEx(&InitCtrls);

	CWinApp::InitInstance();

	// Initialize OLE libraries
	if (!AfxOleInit())
	{
//		AfxMessageBox(IDP_OLE_INIT_FAILED);
		return FALSE;
	}
	AfxEnableControlContainer();
	// Standard initialization
	// If you are not using these features and wish to reduce the size
	// of your final executable, you should remove from the following
	// the specific initialization routines you do not need
	// Change the registry key under which our settings are stored
	// TODO: You should modify this string to be something appropriate
	// such as the name of your company or organization
	SetRegistryKey(_T("Local AppWizard-Generated Applications"));
	LoadStdProfileSettings(4);  // Load standard INI file options (including MRU)
	// Register the application's document templates.  Document templates
	//  serve as the connection between documents, frame windows and views
	CSingleDocTemplate* pDocTemplate;
	pDocTemplate = new CSingleDocTemplate(
		IDR_MAINFRAME,
		RUNTIME_CLASS(CRobotNaviDoc),
		RUNTIME_CLASS(CMainFrame),       // main SDI frame window
		RUNTIME_CLASS(CRobotNaviView));
	if (!pDocTemplate)
		return FALSE;
	AddDocTemplate(pDocTemplate);



	// Parse command line for standard shell commands, DDE, file open
	CCommandLineInfo cmdInfo;
	ParseCommandLine(cmdInfo);


	// Dispatch commands specified on the command line.  Will return FALSE if
	// app was launched with /RegServer, /Register, /Unregserver or /Unregister.
	if (!ProcessShellCommand(cmdInfo))
		return FALSE;

	// The one and only window has been initialized, so show and update it
    
	m_pMainWnd->ShowWindow(SW_SHOW);
	m_pMainWnd->UpdateWindow();
	// call DragAcceptFiles only if there's a suffix
	//  In an SDI app, this should occur after ProcessShellCommand
	return TRUE;
}



// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()

// App command to run the dialog
void CRobotNaviApp::OnAppAbout()
{
	CAboutDlg aboutDlg;
	aboutDlg.DoModal();
}


// CRobotNaviApp message handlers


void CRobotNaviApp::OnFileRobotstart()
{
	// TODO: Add your command handler code here
	RobotDlg rdlg;
	rdlg.DoModal();
}

void CRobotNaviApp::OnFileLaserstart()
{
	// TODO: Add your command handler code here
	LaserDlg ldlg;
	ldlg.DoModal();
}

void CRobotNaviApp::OnFileIcpstart()
{
	// TODO: Add your command handler code here
	ICPDlg* idlg = new ICPDlg();
	//idlg.DoModal();
	idlg->Create(IDD_ICP);
	idlg->ShowWindow(1);
}

void CRobotNaviApp::OnFileDrawmap()
{
	// TODO: Add your command handler code here
	MapDlg* mdlg = new MapDlg(200,200);
	mdlg->Create(IDD_MAP);
	mdlg->ShowWindow(1);
}

void CRobotNaviApp::OnFileJoystickstart()
{
	// TODO: Add your command handler code here
    JoyDlg* jdlg = new JoyDlg();
	jdlg->Create(IDD_JOYDLG);
	jdlg->ShowWindow(1);
}

void CRobotNaviApp::OnFileNavistart()
{
	// TODO: Add your command handler code here
		// TODO: Add your command handler code here
	NavDlg* ndlg = new NavDlg();
	ndlg->Create(IDD_NAVDLG);
	ndlg->ShowWindow(1);
}

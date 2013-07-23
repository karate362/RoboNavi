// JoyDlg.cpp : implementation file
//

#include "stdafx.h"
#include "RobotNavi.h"
#include "JoyDlg.h"
#include "JoyStick.h"


// JoyDlg dialog

JoyStick myjoy;

IMPLEMENT_DYNAMIC(JoyDlg, CDialog)

JoyDlg::JoyDlg(CWnd* pParent /*=NULL*/)
	: CDialog(JoyDlg::IDD, pParent)
{
	memset(Keyold,0x00,32);
}

JoyDlg::~JoyDlg()
{
}

void JoyDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(JoyDlg, CDialog)
	ON_WM_PAINT()
	ON_WM_CREATE()
	ON_BN_CLICKED(IDCANCEL, &JoyDlg::OnBnClickedCancel)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDOK, &JoyDlg::OnBnClickedOk)
END_MESSAGE_MAP()


// JoyDlg message handlers
// JoyDlg message handlers
BOOL JoyDlg::OnInitDialog(){
   CClientDC dc(this);
   jview.init(dc,320,240);
   jview.FillAll(255,255,255);
   jview.DrawCircleT(0,0,10,2,255,0,0);

myjoy.m_hWnd=m_hWnd;

if(!myjoy.Initialize()){
	//OutputDebugString((LPCSTR)"init err-inCDIJoystickDlg::OnInitDialog\n");
	OnCancel();
	return FALSE;
}

    SetTimer(1,50,NULL);//

   	return TRUE;
}


void JoyDlg::OnPaint()
{

   CClientDC dc(this); 

   jview.PasteOn(dc,0,0);

   CDialog::OnPaint();

}
void JoyDlg::OnBnClickedCancel()
{
	// TODO: Add your control notification handler code here
	KillTimer(1);//
	OnCancel();
}

void JoyDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: Add your message handler code here and/or call default

int cx;
int cy;

char Keynew[32];

if(FAILED(myjoy.PollDevice())){
	KillTimer(1); 
    MessageBox(TEXT("error"),TEXT("exit"),MB_ICONERROR|MB_OK);

}

jview.FillAll(255,255,255);

memcpy(Keynew,myjoy.m_diJs.rgbButtons,32);

for(int i=0;i<32;i++){
	if( (Keynew[i] & 0x80) && !(Keyold[i] & 0x80) ){//Push down
		SetDlgItemInt(IDC_EDIT3,i);	
		keyqueue.push(i);
	}

	Keyold[i] = Keynew[i];


}

SetDlgItemInt(IDC_EDIT1,myjoy.m_diJs.lX);//left<->right 0~65535
SetDlgItemInt(IDC_EDIT2,myjoy.m_diJs.lY);//up<->down 0~65535

jx = myjoy.m_diJs.lX;
jy = myjoy.m_diJs.lY;

cx = (jx - 32767)*jview.Width() / 65535 + jview.Width()/2;
cy = (jy - 32767)*jview.Height() / 65535 + jview.Height()/2; 

jview.DrawCircle(cx,cy,10,2,255,0,0);


OnPaint();

CDialog::OnTimer(nIDEvent);
}


void JoyDlg::OnBnClickedOk()
{
	// TODO: Add your control notification handler code here
	char str[128];

	while(!keyqueue.empty()){
		sprintf(str,"%s,%d",str,keyqueue.front());
	keyqueue.pop();
	}
	this->SetDlgItemText(IDC_EDIT3,str);
	
}

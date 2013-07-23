// MFC_VIEWER.cpp: implementation of the MFC_VIEWER class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "MFC_VIEWER.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

MFC_VIEWER::MFC_VIEWER()
{

}

MFC_VIEWER::~MFC_VIEWER()
{

}

void MFC_VIEWER::init(CClientDC& dc,int width,int height)
{
bitmap.CreateCompatibleBitmap(&dc,width,height); //點陣圖初始化 和這個視窗dc配合
bitmap.GetBitmap(&bitmap_info);//取得bmp infomation, 長 寬 pixel depth之類

dcbmp.CreateCompatibleDC(&dc);//CDC初始化 和這個視窗dc配合
dcbmp.SelectObject(bitmap);//讓點陣圖和CDC配合起來 這樣就可以直接操作CDC

}

void MFC_VIEWER::initBitmap(CClientDC& dc,int width,int height,UINT nPlane,UINT nBit,void* initbit){
	bitmap.CreateBitmap( width, height, nPlane, nBit,initbit);
bitmap.GetBitmap(&bitmap_info);//取得bmp infomation, 長 寬 pixel depth之類

dcbmp.CreateCompatibleDC(&dc);//CDC初始化 和這個視窗dc配合
dcbmp.SelectObject(bitmap);//讓點陣圖和CDC配合起來 這樣就可以直接操作CDC
}


void MFC_VIEWER::FillRect(int x1,int y1,int x2,int y2,int r,int g,int b)
{
   CBrush brush(RGB(r,g,b));
   dcbmp.FillRect(CRect(x1,y1,x2,y2),&brush);
   brush.DeleteObject();
}

void MFC_VIEWER::FillAll(int r,int g,int b)
{
   FillRect(0,0,Width(),Height(),r,g,b);
}

void MFC_VIEWER::DrawLine(int x1,int y1,int x2,int y2,int width,int r,int g,int b)
{
	CPen pen;
	pen.CreatePen(PS_SOLID,width,RGB(r,g,b));
	dcbmp.SelectObject(&pen);
	dcbmp.MoveTo(x1,y1);
	dcbmp.LineTo(x2,y2);
	pen.DeleteObject();
}

void MFC_VIEWER::DrawCircle(int x,int y, int radius,int width,int r,int g,int b)
{
	CPen pen;
	pen.CreatePen(PS_SOLID,width,RGB(r,g,b));
	dcbmp.SelectObject(&pen);
	dcbmp.Ellipse(x-radius,y-radius,x+radius,y+radius);
	pen.DeleteObject();
}

void MFC_VIEWER::DrawLineT(int x1,int y1,int x2,int y2,int width,int r,int g,int b)
{
	Transform(x1,y1,x1,y1);
	Transform(x2,y2,x2,y2);
	CPen pen;
	pen.CreatePen(PS_SOLID,width,RGB(r,g,b));
	dcbmp.SelectObject(&pen);
	dcbmp.MoveTo(x1,y1);
	dcbmp.LineTo(x2,y2);
	pen.DeleteObject();
}

void MFC_VIEWER::DrawCircleT(int x,int y, int radius,int width,int r,int g,int b)
{
	Transform(x,y,x,y);
	CPen pen;
	pen.CreatePen(PS_SOLID,width,RGB(r,g,b));
	dcbmp.SelectObject(&pen);
	dcbmp.Ellipse(x-radius,y-radius,x+radius,y+radius);
	pen.DeleteObject();
}

void MFC_VIEWER::DrawArcT(int x,int y, int radius,int length,int width,int r,int g,int b){

	//Transform(x,y,x,y);
	double ang = fabs((double)length/(double)radius);
	ang = min(ang,2*3.1415926);

	int xed = (x) + (int)((double)abs(radius)*sin(ang));
	int yed = (y+abs(radius)) - (int)((double)abs(radius)*cos(ang));//Draw clockwisely
	int x1 = x+abs(radius);
	int x2 = x-abs(radius);
	int y1 = y + 2*radius;
	int y2 = y;

	if(radius<0){
		y1 = y;
		y2 = y+2*radius;
		yed = y - yed;
	}


	Transform(x,y,x,y);
	Transform(x1,y1,x1,y1);
	Transform(x2,y2,x2,y2);
	Transform(xed,yed,xed,yed);

	CPen pen;
	pen.CreatePen(PS_SOLID,width,RGB(r,g,b));
	dcbmp.SelectObject(&pen);
	
	if(radius>=0)
		dcbmp.Arc(x1,y1,x2,y2,x,y,xed,yed);
	else
		dcbmp.Arc(x1,y1,x2,y2,xed,yed,x,y);

	pen.DeleteObject();
}

void MFC_VIEWER::LoadPicture(CString path)
{
	bitmap.m_hObject=LoadImage(NULL,(LPCTSTR)path,IMAGE_BITMAP,0,0,LR_LOADFROMFILE);
    dcbmp.SelectObject(bitmap);
    //bitmap.GetBitmap(&bitmap_info);

}

void MFC_VIEWER::SavePicture(CString path){
	CImage img;
	img.Attach((HBITMAP)bitmap);
	img.Save(path);

}
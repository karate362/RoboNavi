// MFC_VIEWER.h: interface for the MFC_VIEWER class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MFC_VIEWER_H__E742FF06_56BA_48C1_B469_EF183FFF7CA2__INCLUDED_)
#define AFX_MFC_VIEWER_H__E742FF06_56BA_48C1_B469_EF183FFF7CA2__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <atlimage.h>

class MFC_VIEWER  
{
public:
	MFC_VIEWER();
	virtual ~MFC_VIEWER();

    void init(CClientDC& dc,int width,int height);
	void initBitmap(CClientDC& dc,int width,int height,UINT nPlane,UINT nBit,void* initbit = NULL);


	void PasteOn(CClientDC& dc,int ox,int oy)
	{
        bitmap.GetBitmap(&bitmap_info);
		dc.BitBlt(ox,oy,bitmap_info.bmWidth,bitmap_info.bmHeight,&dcbmp,0,0,SRCCOPY);
	}

	int Width()
	{return bitmap_info.bmWidth;}
	int Height()
	{return bitmap_info.bmHeight;}
	void FillRect(int x1,int y1,int x2,int y2,int r,int g,int b);
	void DrawLine(int x1,int y1,int x2,int y2,int width,int r,int g,int b);
    void DrawCircle(int x,int y, int radius,int width,int r,int g,int b);

	void DrawLineT(int x1,int y1,int x2,int y2,int width,int r,int g,int b);//Draw line in transformed coordinate 
    void DrawCircleT(int x,int y, int radius,int width,int r,int g,int b);//Draw circle in transformed coordinate 
	void DrawArcT(int x,int y, int radius,int length,int width,int r,int g,int b);//Draw Arc in transformed coordinate 

	inline void Transform(int px, int py,int &nx, int &ny)//results are saved in nx,ny
	{
		int x = px;
		int y = py;
		nx = Width()/2 - py;
		ny = Height()/2 - px;
	}



	void FillAll(int r=255,int g=255,int b=255);
    void LoadPicture(CString path);
	void SavePicture(CString);

	void CopyFrom(void* src, int size){
		//memcpy(bitmap_info.bmBits,src,size);
		bitmap.SetBitmapBits(size,src);
		//bitmap.CreateBitmap(200, 200, 1, 8,src);
	}//


public:
	CBitmap bitmap;
	BITMAP bitmap_info;//BMP infomation
    CDC dcbmp;//CDCª«¥ó

};

#endif // !defined(AFX_MFC_VIEWER_H__E742FF06_56BA_48C1_B469_EF183FFF7CA2__INCLUDED_)

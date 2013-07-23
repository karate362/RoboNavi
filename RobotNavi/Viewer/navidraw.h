// NaviDraw.h: interface for the NaviDraw class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_NAVIDRAW_H__DA89FC67_2433_4C0A_A355_6313DB623544__INCLUDED_)
#define AFX_NAVIDRAW_H__DA89FC67_2433_4C0A_A355_6313DB623544__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "MFC_VIEWER.h"
#include "DyWin.h"
#include "ND.h"
#include "DWAstar.h"

#include "VisableArea.h"
#include "TraTree.h"

class NaviDraw  
{
public:
	NaviDraw();
	virtual ~NaviDraw();

};

void DrawNDOnView(MFC_VIEWER& view,ND& nd,int size,int ratio);
void DrawRobotOnView(MFC_VIEWER& view,double v,double w,double gx,double gy,int policy,int ratio);
void DrawOBSOnView(MFC_VIEWER& view,ObsArray& obsarr,int ratio);
void DrawDWATreeOnView(MFC_VIEWER& view,DWAstar& dstar,double rx,double ry,double rth,int ratio);
void DrawVAsOnView(MFC_VIEWER& view,VisableArea& VA,double ratio);
void DrawSensorReading(MFC_VIEWER& viewer, vector<double> &range, vector<double> &rad, double scale);

//void DrawTraTreeOnView(MFC_VIEWER& view,TraTree& dstar,double rx,double ry,double rth,int ratio);

#endif // !defined(AFX_NAVIDRAW_H__DA89FC67_2433_4C0A_A355_6313DB623544__INCLUDED_)

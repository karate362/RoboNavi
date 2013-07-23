// RobotNaviDoc.cpp : implementation of the CRobotNaviDoc class
//

#include "stdafx.h"
#include "RobotNavi.h"

#include "RobotNaviDoc.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CRobotNaviDoc

IMPLEMENT_DYNCREATE(CRobotNaviDoc, CDocument)

BEGIN_MESSAGE_MAP(CRobotNaviDoc, CDocument)
END_MESSAGE_MAP()


// CRobotNaviDoc construction/destruction

CRobotNaviDoc::CRobotNaviDoc()
{
	// TODO: add one-time construction code here

}

CRobotNaviDoc::~CRobotNaviDoc()
{
}

BOOL CRobotNaviDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: add reinitialization code here
	// (SDI documents will reuse this document)

	return TRUE;
}




// CRobotNaviDoc serialization

void CRobotNaviDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}


// CRobotNaviDoc diagnostics

#ifdef _DEBUG
void CRobotNaviDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CRobotNaviDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CRobotNaviDoc commands

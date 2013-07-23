// RobotNaviView.cpp : implementation of the CRobotNaviView class
//

#include "stdafx.h"
#include "RobotNavi.h"

#include "RobotNaviDoc.h"
#include "RobotNaviView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CRobotNaviView

IMPLEMENT_DYNCREATE(CRobotNaviView, CView)

BEGIN_MESSAGE_MAP(CRobotNaviView, CView)
	// Standard printing commands
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CView::OnFilePrintPreview)
END_MESSAGE_MAP()

// CRobotNaviView construction/destruction

CRobotNaviView::CRobotNaviView()
{
	// TODO: add construction code here

}

CRobotNaviView::~CRobotNaviView()
{
}

BOOL CRobotNaviView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}

// CRobotNaviView drawing

void CRobotNaviView::OnDraw(CDC* /*pDC*/)
{
	CRobotNaviDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: add draw code for native data here
}


// CRobotNaviView printing

BOOL CRobotNaviView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// default preparation
	return DoPreparePrinting(pInfo);
}

void CRobotNaviView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add extra initialization before printing
}

void CRobotNaviView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add cleanup after printing
}


// CRobotNaviView diagnostics

#ifdef _DEBUG
void CRobotNaviView::AssertValid() const
{
	CView::AssertValid();
}

void CRobotNaviView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CRobotNaviDoc* CRobotNaviView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CRobotNaviDoc)));
	return (CRobotNaviDoc*)m_pDocument;
}
#endif //_DEBUG


// CRobotNaviView message handlers

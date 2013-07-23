// RobotNaviView.h : interface of the CRobotNaviView class
//


#pragma once


class CRobotNaviView : public CView
{
protected: // create from serialization only
	CRobotNaviView();
	DECLARE_DYNCREATE(CRobotNaviView)

// Attributes
public:
	CRobotNaviDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:
	virtual BOOL OnPreparePrinting(CPrintInfo* pInfo);
	virtual void OnBeginPrinting(CDC* pDC, CPrintInfo* pInfo);
	virtual void OnEndPrinting(CDC* pDC, CPrintInfo* pInfo);

// Implementation
public:
	virtual ~CRobotNaviView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // debug version in RobotNaviView.cpp
inline CRobotNaviDoc* CRobotNaviView::GetDocument() const
   { return reinterpret_cast<CRobotNaviDoc*>(m_pDocument); }
#endif


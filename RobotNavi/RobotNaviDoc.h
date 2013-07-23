// RobotNaviDoc.h : interface of the CRobotNaviDoc class
//


#pragma once


class CRobotNaviDoc : public CDocument
{
protected: // create from serialization only
	CRobotNaviDoc();
	DECLARE_DYNCREATE(CRobotNaviDoc)

// Attributes
public:

// Operations
public:

// Overrides
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);

// Implementation
public:
	virtual ~CRobotNaviDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
};



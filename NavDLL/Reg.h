// Reg.h: interface for the Reg class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_REG_H__79EEAAD4_F83C_437C_82A1_F1D84B60ABA7__INCLUDED_)
#define AFX_REG_H__79EEAAD4_F83C_437C_82A1_F1D84B60ABA7__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define PI 3.1415926

struct ITV {//velocity space interval
double dmax;
int regindex;//����region
};

struct GAP {
int i;//��ܦbintervals[i]�P[i-1]����
bool max;//true: right, false:left
double d;//���u�䤧����
};


class Reg  
{
public:
	Reg();
	virtual ~Reg();

};

#endif // !defined(AFX_REG_H__79EEAAD4_F83C_437C_82A1_F1D84B60ABA7__INCLUDED_)

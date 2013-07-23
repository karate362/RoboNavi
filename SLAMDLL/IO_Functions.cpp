#include "StdAfx.h"
#include "IO_Functions.h"

using namespace std;

#define MAX_STRING_LENGTH 8192

char ReadBuf[MAX_STRING_LENGTH];



void _declspec(dllexport) StrToReading(char* raw, double& x,double& y, double& ang, vector<double> &rad, vector<double> &data){//Get Data in Polar form
	
	char* it = raw;
	double range;
    double phi = -90;
	double dphi = 0;

	rad.clear();
	data.clear();

	int s=0;
    int i=0;

    it = strtok(raw," ");
	x = atof(it);
    it = strtok(NULL," ");
	y = atof(it);
    it = strtok(NULL," ");
	ang = atof(it);
	it = strtok(NULL," ");

	while(*it != '\n')
	{
		range = atof(it);
        data.push_back(range);

		it = strtok(NULL," ");
	}

	s = data.size();//361 or 181, -90 ~ 90
	dphi = 180/(double)(s-1);

	for(i=0;i<s;++i){
		rad.push_back(phi*3.1415926/180);		
		phi += dphi;
	}
}


bool _declspec(dllexport) ReadRawLaserData(FILE* fin,double& x,double& y, double& ang, std::vector<double>& rad, std::vector<double>& range){

	//Read one line from fin
	if( !fgets(ReadBuf,MAX_STRING_LENGTH,fin) )
		return false;
	else
		StrToReading(ReadBuf, x, y, ang,rad, range);

	return true;

}

void _declspec(dllexport) WriteMeasurementData(FILE* fout,const double &x, const double &y, const double &phi, const std::vector<double> &range ){

	int s = range.size();
	fprintf(fout,"%.3f %.3f %.3f ",x,y,phi);

	for(int i=0;i<s;++i)
		fprintf(fout,"%.3f ",range[i]);

	fprintf(fout,"\n");

}

void _declspec(dllexport) StrToReading2(char* raw, double& x,double& y, double& ang, vector<double> &rad, vector<double> &data){//Get Data in Polar form
	
	char* it = raw;
	double range;
    double phi = -90;
	double dphi = 0;

	rad.clear();
	data.clear();

	int s=0;
    int i=0;

    it = strtok(raw," ");
	x = atof(it);
    it = strtok(NULL," ");
	y = atof(it);
    it = strtok(NULL," ");
	ang = atof(it);
	it = strtok(NULL," ");

	while(*it != '\n')
	{
		range = atof(it);
        data.push_back(range);

		it = strtok(NULL," ");
	}

	s = data.size();//361 or 181, -90 ~ 90
	dphi = 180/(double)(s-1);

	for(i=0;i<s;++i){
		rad.push_back(phi*3.1415926/180);		
		phi += dphi;
	}
}


bool _declspec(dllexport) ReadICPLaserData(FILE* fin,double& x,double& y, double& ang, std::vector<double>& rad, std::vector<double>& range){

	//Read one line from fin
	if( !fgets(ReadBuf,MAX_STRING_LENGTH,fin) )
		return false;
	else
		StrToReading2(ReadBuf, x, y, ang,rad, range);

	return true;

}
void _declspec(dllexport) StrToDoubleArray(char* raw, vector<double> &data){

	char* it = raw;

	data.clear();

	it = strtok(raw," ");

	while(*it != '\n')
	{
        data.push_back(atof(it));

		it = strtok(NULL," ");
	}

}

void _declspec(dllexport) ReadDoubleMatrix(FILE* fin,vector<double>& reading,int& width,int& height){


	vector<double> buf;

	reading.clear();

	int dst = 0;

	height = 0;

	while( fgets(ReadBuf,MAX_STRING_LENGTH,fin) ){

		StrToDoubleArray(ReadBuf,buf);//Where's the problem??
		++height;
		width = buf.size();
		reading.resize(reading.size() + width);
		copy(buf.begin(),buf.end(),reading.begin() + dst);
		dst += width;
	}


}

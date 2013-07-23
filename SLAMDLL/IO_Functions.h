#pragma once
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <math.h>
#include <vector>


void _declspec(dllexport) ReadDoubleMatrix(FILE* fin,std::vector<double>& reading,int& width,int& height);
void _declspec(dllexport) StrToDoubleArray(char* raw, std::vector<double> &data);

void _declspec(dllexport) StrToReading(char* raw, double& x,double& y, double& ang, std::vector<double>& rad, std::vector<double>& range);
bool _declspec(dllexport) ReadRawLaserData(FILE* fin,double& x,double& y, double& ang, std::vector<double>& rad, std::vector<double>& range);
void _declspec(dllexport) WriteMeasurementData(FILE* fout, const double &x, const double &y, const double &phi, const std::vector<double> &range );

void _declspec(dllexport) StrToReading2(char* raw, double& x,double& y, double& ang, std::vector<double>& rad, std::vector<double>& range);
bool _declspec(dllexport) ReadICPLaserData(FILE* fin,double& x,double& y, double& ang, std::vector<double>& rad, std::vector<double>& range);
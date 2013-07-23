#include "SickLms100Laser.h"
#include <stdio.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Robotics;

#define DATA_LENGTH 541
#define DEVICE_ADDR "192.168.0.1"

ofstream outfile("LR.txt");

int main() {
	int aScan[DATA_LENGTH];

	SickLms100Laser *pLaser = new SickLms100Laser();

	if (pLaser->Open(DEVICE_ADDR)) {
		pLaser->Scan(aScan);

		for (int i = 0; i < DATA_LENGTH; i++) {
			printf("[%2d] %5d  ", i, aScan[i]);
			outfile<<i<<" "<<aScan[i]<<endl;

			if (i % 6 == 5) printf("\n");
		}

		pLaser->Close();
	}

	delete pLaser;

	printf("\nPress enter to exit...");

	getchar();

	return 0;
}
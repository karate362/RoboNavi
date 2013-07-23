#include "StdAfx.h"
#include "DATMO.h"
#include <iostream>
#include <stdio.h>
#include <cmath>

using namespace Geom2D;
DATMO::DATMO(void):x_offset(4),y_offset(0)
{
	nowidx = 0;
	poses.resize(snum);
	ranges.resize(snum);
	rads.resize(snum);


	gmap = new GridMap(x_offset,y_offset,0);//Front 3m is the map center
	buffermap = new GridMap();

}


DATMO::~DATMO(void)
{
	delete gmap;
}


void DATMO::DetectMOs(Geom2D::Pose& np, vector<double>& nrange, vector<double>& nrad){//np: get the new reading at pose np.

	int s = nrange.size();
	int i=0;

	Point lp;
	int w;
	int h;
	double occ;
	Transform2D t(np);

	Lastrange.resize(nrange.size());
	Lastrad.resize(nrad.size());
    rtype.resize(nrange.size());
	copy(nrange.begin(),nrange.end(),Lastrange.begin());
    copy(nrad.begin(),nrad.end(),Lastrad.begin());
	Lastpose = np;


	for(i=0;i<s;++i){
		lp.x = nrange[i]*cos(nrad[i]);
		lp.y = nrange[i]*sin(nrad[i]);
		t.transform_to_global(lp);

		gmap->XYToGrid(lp.x,lp.y,w,h);

		if(gmap->InMAP(w,h)){
			occ = gmap->getGrid(w,h);
			occ = exp(occ);
			occ = occ/(occ+1);
		}
		else
			occ = 0.5;


		if(occ<0.2 ){//free --> occupied! moving object!
			rtype[i] = 1;
		}
		else
			if(occ<=0.6){//unknown --> occupied!
				rtype[i] = -1;
			}
			else// static object!
				rtype[i] = 0;
	}

}

void DATMO::DetectMOs_dummy(Geom2D::Pose& np, vector<double>& nrange, vector<double>& nrad){
//TODO:
	Lastrange.resize(nrange.size());
	Lastrad.resize(nrad.size());
    rtype.resize(nrange.size());
	Lastpose = np;
	copy(nrange.begin(),nrange.end(),Lastrange.begin());
    copy(nrad.begin(),nrad.end(),Lastrad.begin());
	
	for(int i=0;i<rtype.size();++i)
		rtype[i] = 1;

}


void DATMO::UpdateMap(){//Don't create new map, only transform the old map;
	GridMap* buf;
    Pose mapose;
	mapose.p.x = x_offset;
	mapose.p.y = y_offset;
	mapose.phi = 0;

	Transform2D t(Lastpose);
	t.transform_to_global(mapose);

	gmap->TransformMap(mapose,*buffermap);
	buf = gmap;
	gmap = buffermap;
	buffermap = buf;

    gmap->setPose(mapose);
    gmap->Grid_Mapping_DATMO(Lastpose,Lastrange,Lastrad,rtype);
}


void DATMO::ClusterMOs(vector<Geom2D::Point>& MOs){

	int s = MOs.size();
	int i=1;
	int j = 0;
	int start = 0;
	int end = 0;
	double size = 0;
	double theta = 0;
	double a=0;
	double b=0;
	double c=0;
	vector<Geom2D::Point> Laser_xy;

	Point p1;
	Point p2;
	Point p3;		//cluster start
	Point p4;		//cluster start - 1
	Point p5;		//cluster end
	Point p6;		//cluster end + 1

	int pointnum = 0;
	Point mopose;

	MOposes.clear();
	MOsizes.clear();
	Transform2D t(Lastpose);
	int u=Lastrange.size();
	//convert laser point to xy
	for(int j=0;j<u;++j){
		
			p3.x = Lastrange[j]*cos(Lastrad[j]);
			p3.y = Lastrange[j]*sin(Lastrad[j]);
			t.transform_to_global(p3);
			p3.relative_range = Lastrange[j];
			p3.relative_rad = Lastrad[j];
			p3.index = j;
			Laser_xy.push_back(p3);
		
	}
	
	if(s==0)
		return;
	else{
		mopose.x = MOs[0].x;
		mopose.y = MOs[0].y;
		mopose.index = MOs[0].index;
		pointnum = 1;
	}


	for(i=1;i<s;++i){
		
		p1 = MOs[i-1];
		p2 = MOs[i];
		if (pointnum == 1)
			start = i;

		if(dist(p1,p2) < 0.1){
			mopose.x += MOs[i].x;//The same object;
			mopose.y += MOs[i].y;
			mopose.index = MOs[i].index;
			pointnum += 1;
			theta = theta + fabs(MOs[i].relative_rad - MOs[i-1].relative_rad);

		}
		else{
			mopose.x = mopose.x/(double)pointnum;
			mopose.y = mopose.y/(double)pointnum;
			a = MOs[start].relative_range;
			b = MOs[i-1].relative_range;
			
			//calculate cluster size
			c = sqrt(a*a+b*b-2*a*b*cos(theta));
			//mopose.size = pointnum;
			//mopose.size = c;
			if(pointnum >= 5 & c < 0.5)	{
#if 1
				p3 = MOs[start];
				p5 = MOs[i-1];
				p6 = Laser_xy[MOs[i-1].index + 3];
				if (MOs[start].index > 3){
					p4 = Laser_xy[MOs[start].index - 2];
					if (dist(p3,p4) >= 0.35 || dist(p5, p6) >= 0.35)
						mopose.local_minima = true;
					else
						mopose.local_minima = false;
				}
				else
					mopose.local_minima = false;
#endif
				MOposes.push_back(mopose);
			
			}
		    mopose.x = MOs[i].x;
		    mopose.y = MOs[i].y;
			mopose.index = MOs[i].index;
		    theta = 0;
			pointnum = 1;
			
		}

	}
	mopose.x = mopose.x/(double)pointnum;// For the last object
	mopose.y = mopose.y/(double)pointnum;
	a = MOs[start].relative_range;
	b = MOs[s-1].relative_range;
	c = sqrt(a*a+b*b-2*a*b*cos(theta));
			
	//mopose.size = c;
	//mopose.size = pointnum;
	if(pointnum >= 5 & c < 0.5 ){
#if 1
		p3 = MOs[start];
				p5 = MOs[s-1];
				if (MOs[s-1].index < (u-3))
					p6 = Laser_xy[MOs[s-1].index-2];
				else
					p6 = Laser_xy[MOs[s-1].index];
				if (MOs[start].index > 3){
					p4 = Laser_xy[MOs[start].index - 3];
					if (dist(p3,p4) >= 0.35 || dist(p5, p6) >= 0.35)
						mopose.local_minima = true;
					else
						mopose.local_minima = false;
				}
				else
					mopose.local_minima = true;
#endif
				MOposes.push_back(mopose);
	}

}
void DATMO::DetectPedestrian(vector<Geom2D::Point>& CMOs, vector<Geom2D::Pedestrian>& current_pedestrian){

	
	int i = 0;
	int s = 0;
	int j = 0;
	double id = 1;
	bool check = true;
	Pedestrian temppose;
	//vector<Geom2D::Pedestrian> current_pedestrian1;
	Point p1;
	Point p2;
	s = CMOs.size();
#if 1	
	for(i=0;i<s;++i){		
		if (i == 0){
			if (CMOs[0].local_minima == true){
			temppose.x = CMOs[0].x;
			temppose.y = CMOs[0].y;
			//temppose.id = id;
			temppose.id = 0;
			temppose.present = 1;
			temppose.size = CMOs[0].size;
			current_pedestrian.push_back(temppose);
			id = id + 1;
			}
		}
		else{
			if (dist(CMOs[i],CMOs[i-1]) < 0.6){
				if (CMOs[i].local_minima == true){
				temppose.x = CMOs[i].x;
				temppose.y = CMOs[i].y;
				temppose.size = CMOs[i].size;
				//temppose.id = id - 1;
				temppose.id = 0;
				temppose.present = 1;			
				current_pedestrian.push_back(temppose);
				}
			}
			else {
				if (CMOs[i].local_minima == true){
				temppose.x = CMOs[i].x;
				temppose.y = CMOs[i].y;
				temppose.size = CMOs[i].size;
				//temppose.id = id;
				temppose.id = 0;
				temppose.present = 1;
				current_pedestrian.push_back(temppose);
				id = id + 1;
				}
			}

		}
	}
#endif

}

void DATMO::TrackingPedestrian(vector<Geom2D::Pedestrian>& tracked_pedestrian, vector<Geom2D::Pedestrian>& current_pedestrian, Geom2D::Occupied_ID &current_ID){
	int i = 0;
	int s = 0;
	int c = 0;
	int j = 0;
	int n = 0;
	int m = 0;
	int index = 0;
	int id = 1;
	double distance = 0;
	double shortest_distance = 0;
	double appear_time = 0;
	bool link;
	double Tmean[4];
	double Tsigma[16];
	int iterate = 0;
	double uncertainty = 0;

	Pedestrian temppose;

	Point tracked;
	Point current;

	s = tracked_pedestrian.size();
	c = current_pedestrian.size();
	if (c != 0 & s != 0){
		for(i=0;i<c;++i){
			current.x = current_pedestrian[i].x;
			current.y = current_pedestrian[i].y;
			//find the closest cluster
			for(j=0;j<s;++j){
				tracked.x = tracked_pedestrian[j].Tmean[0];
				tracked.y = tracked_pedestrian[j].Tmean[1];
				distance = dist(current, tracked);
				if (shortest_distance == 0){
					shortest_distance = distance;
					id = tracked_pedestrian[j].id;
					appear_time = tracked_pedestrian[j].appear_time;
					index = j;
					for (n = 0; n < 4; n++)
						Tmean[n] = tracked_pedestrian[j].Tmean[n];
					
					for (n = 0; n < 16; n++)
						Tsigma[n] = tracked_pedestrian[j].Tsigma[n];
				}
				else if (distance < shortest_distance){
					shortest_distance = distance;
					id = tracked_pedestrian[j].id;
					appear_time = tracked_pedestrian[j].appear_time;
					index = j;
					for (n = 0; n < 4; n++)
						Tmean[n] = tracked_pedestrian[j].Tmean[n];
					for (n = 0; n < 16; n++)
						Tsigma[n] = tracked_pedestrian[j].Tsigma[n];
				}
			}
			if (current_ID.used_ID[1] != 0){
				uncertainty = abs(current_ID.used_ID[1] - tracked_pedestrian[index].appear_time)/10*2;
				//if (uncertainty == 0)
				//	uncertainty = 1;
			}
			//nearest neighbor method
			if (shortest_distance < (1.5+uncertainty)){
				if (tracked_pedestrian[index].appear_time > 2 & tracked_pedestrian[index].id == 0){
					if (current_ID.used_ID[1] == 0){
					current_pedestrian[i].id = 1;
					tracked_pedestrian[index].id = 1;
					//current_pedestrian[i].size = uncertainty;
								
					for (n = 0; n < tracked_pedestrian.size(); n++){
						Point a;
						Point b;
						a.x = tracked_pedestrian[index].x;
						a.y = tracked_pedestrian[index].y;
						b.x = tracked_pedestrian[n].x;
						b.y = tracked_pedestrian[n].y;
						if (dist(a,b) < 0.6)
							tracked_pedestrian[n].id = 1;
					}
					current_pedestrian[i].present = 1;
					//current_pedestrian[i].appear_time = tracked_pedestrian[index].appear_time;
					current_pedestrian[i].appear_time = 2;
					//current_pedestrian[i].size = 0;
					current_ID.used_ID[1] = 1;
					for (n = 0; n < 4; n++)
						current_pedestrian[i].Tmean[n]=Tmean[n];
					for (n = 0; n < 16; n++)
						current_pedestrian[i].Tsigma[n] = Tsigma[n];
					
				}
#if 0
				else if (current_ID.used_ID[2] == 0){
					current_pedestrian[i].id = 2;
					tracked_pedestrian[index].id = 2;
					for (n = 0; n < tracked_pedestrian.size(); n++){
						Point a;
						Point b;
						a.x = tracked_pedestrian[index].x;
						a.y = tracked_pedestrian[index].y;
						b.x = tracked_pedestrian[n].x;
						b.y = tracked_pedestrian[n].y;
						if (dist(a,b) < 0.6)
							tracked_pedestrian[n].id = 2;
					}
					current_pedestrian[i].present = 1;
					current_pedestrian[i].appear_time = 2;
					//current_pedestrian[i].size = 0;
					current_ID.used_ID[2] = 1;
					for (n = 0; n < 4; n++)
						current_pedestrian[i].Tmean[n]=Tmean[n];
					for (n = 0; n < 16; n++)
						current_pedestrian[i].Tsigma[n] = Tsigma[n];
					
				}
				else if (current_ID.used_ID[3] == 0){
					current_pedestrian[i].id = 3;
					tracked_pedestrian[index].id = 3;
					for (n = 0; n < tracked_pedestrian.size(); n++){
						Point a;
						Point b;
						a.x = tracked_pedestrian[index].x;
						a.y = tracked_pedestrian[index].y;
						b.x = tracked_pedestrian[n].x;
						b.y = tracked_pedestrian[n].y;
						if (dist(a,b) < 0.6)
							tracked_pedestrian[n].id = 3;
					}
					current_pedestrian[i].present = 1;
					current_pedestrian[i].appear_time = 2;
					//current_pedestrian[i].size = 0;
					current_ID.used_ID[3] = 1;
					for (n = 0; n < 4; n++)
						current_pedestrian[i].Tmean[n]=Tmean[n];
					for (n = 0; n < 16; n++)
						current_pedestrian[i].Tsigma[n] = Tsigma[n];
				}
				else if (current_ID.used_ID[4] == 0){
					current_pedestrian[i].id = 4;
					tracked_pedestrian[index].id = 4;
					for (n = 0; n < tracked_pedestrian.size(); n++){
						Point a;
						Point b;
						a.x = tracked_pedestrian[index].x;
						a.y = tracked_pedestrian[index].y;
						b.x = tracked_pedestrian[n].x;
						b.y = tracked_pedestrian[n].y;
						if (dist(a,b) < 0.6)
							tracked_pedestrian[n].id = 4;
					}
					current_pedestrian[i].present = 1;
					current_pedestrian[i].appear_time = 2;
					//current_pedestrian[i].size = 0;
					current_ID.used_ID[4] = 1;
					for (n = 0; n < 4; n++)
						current_pedestrian[i].Tmean[n]=Tmean[n];
					for (n = 0; n < 16; n++)
						current_pedestrian[i].Tsigma[n] = Tsigma[n];
					
				}
				else{
					current_pedestrian[i].id = 5;
					//update tracked pedestrian
					tracked_pedestrian[index].id = 5;
					for (n = 0; n < tracked_pedestrian.size(); n++){
						Point a;
						Point b;
						a.x = tracked_pedestrian[index].x;
						a.y = tracked_pedestrian[index].y;
						b.x = tracked_pedestrian[n].x;
						b.y = tracked_pedestrian[n].y;
						if (dist(a,b) < 0.6)
							tracked_pedestrian[n].id = 5;
					}
					current_pedestrian[i].present = 1;
					current_pedestrian[i].appear_time = 2;
					//current_pedestrian[i].size = 0;
					current_ID.used_ID[5] = 1;
					for (n = 0; n < 4; n++)
						current_pedestrian[i].Tmean[n]=Tmean[n];
					for (n = 0; n < 16; n++)
						current_pedestrian[i].Tsigma[n] = Tsigma[n];
					
				}
#endif
				}
				else {
						current_pedestrian[i].size = uncertainty;
						current_pedestrian[i].id = id;
						if (tracked_pedestrian[index].present == 0)
							current_pedestrian[i].appear_time =  current_ID.used_ID[id]+1;
						else
							current_pedestrian[i].appear_time =  tracked_pedestrian[index].appear_time + 1;
					
						current_pedestrian[i].present = 1;
						for (n = 0; n < 4; n++)
							current_pedestrian[i].Tmean[n]=Tmean[n];
				
						for (n = 0; n < 16; n++)
							current_pedestrian[i].Tsigma[n] = Tsigma[n];
					
				}

					
			}
			else{	
				current_pedestrian[i].id = 0;
				current_pedestrian[i].present = 1;
				current_pedestrian[i].appear_time = 0;
				//current_pedestrian[i].size = 0;

			}
			shortest_distance = 0;
			appear_time = 0;
			//id = 1;
			index = 0;
		}
	}
	else if (s == 0){
		for(i=0;i<c;++i){
			current_pedestrian[i].present = 1;
			current_pedestrian[i].size = 0;
			current_pedestrian[i].appear_time = 0;
		}
	}
	filter(current_pedestrian,current_ID);
#if 1
	for(i=0;i<s;++i){
		link = false;
		for(j=0;j<c;++j){
			if (tracked_pedestrian[i].id == current_pedestrian[j].id){
				link = true;
				//current_pedestrian[j].appear_time += 1;
			}
		}
		if (link == false){
			if (current_ID.used_ID[tracked_pedestrian[i].id] - tracked_pedestrian[i].appear_time < 20){
			temppose.x = tracked_pedestrian[i].x;
			temppose.y = tracked_pedestrian[i].y;
			temppose.id = tracked_pedestrian[i].id;
			//temppose.size = tracked_pedestrian[i].size;
			temppose.size = uncertainty;
			temppose.present = 0;
			temppose.appear_time = tracked_pedestrian[i].appear_time;
			//temppose.appear_time = current_ID.used_ID[tracked_pedestrian[i].id];
			for (n = 0; n < 4; n++)
				temppose.Tmean[n]=tracked_pedestrian[i].Tmean[n];
				
			for (n = 0; n < 16; n++)
				temppose.Tsigma[n] = tracked_pedestrian[i].Tsigma[n];
			current_pedestrian.push_back(temppose);
			}
		}
	}
#endif
}


void DATMO::filter(vector<Geom2D::Pedestrian>& current_pedestrian, Geom2D::Occupied_ID & current_ID){
	int n = 0;
	int m = 0;
	int shortest = 0;
	int second_shortest = 0;
	Point filter1;
	Point filter2;
	Point prediction;
	double distance;
	double distance1;
	for (n = 0; n < current_pedestrian.size(); n++){
		if (current_pedestrian[n].id != 0){
		prediction.x = current_pedestrian[n].Tmean[0];
		prediction.y = current_pedestrian[n].Tmean[1];
		filter1.x = current_pedestrian[n].x;
		filter1.y = current_pedestrian[n].y;
		distance = dist(prediction, filter1);
		shortest = n;
		for (m = 0; m < current_pedestrian.size(); m++){
			if (m != n & current_pedestrian[n].id == current_pedestrian[m].id){
				filter2.x = current_pedestrian[m].x;
				filter2.y = current_pedestrian[m].y;
				distance1 = dist(prediction, filter2);
				if (distance1 < distance)
					shortest = m;
			}
		}
		for (m = 0; m < current_pedestrian.size(); m++){
		if (m != shortest & current_pedestrian[m].id == current_pedestrian[shortest].id){
			filter1.x = current_pedestrian[shortest].x;
			filter1.y = current_pedestrian[shortest].y;
			filter2.x = current_pedestrian[m].x;
			filter2.y = current_pedestrian[m].y;
			if (dist(filter1, filter2) > 0.65){
				current_pedestrian[m].id = 0;
				current_pedestrian[m].present = 1;
				current_pedestrian[m].appear_time = 0;
				current_pedestrian[m].size = 0;
#if 0
				if (current_ID.used_ID[1] == 0 ){
					current_pedestrian[second_shortest].id = 1;
					current_pedestrian[second_shortest].present = 1;
					current_pedestrian[second_shortest].appear_time = 0;
					current_ID.used_ID[1] = 1;
					//current_pedestrian[i].last_appear = 0;
						}
						else if (current_ID.used_ID[2] == 0){
							current_pedestrian[second_shortest].id = 2;
							current_pedestrian[second_shortest].present = 1;
							current_pedestrian[second_shortest].appear_time = 0;
					current_ID.used_ID[2] = 1;
					//current_pedestrian[i].last_appear = 0;
						}
						else if (current_ID.used_ID[3] == 0){
							current_pedestrian[second_shortest].id = 3;
							current_pedestrian[second_shortest].present = 1;
							current_pedestrian[second_shortest].appear_time = 0;
					current_ID.used_ID[3] = 1;
					//current_pedestrian[i].last_appear = 0;
						}
						else if (current_ID.used_ID[4] == 0){
							current_pedestrian[second_shortest].id = 4;
							current_pedestrian[second_shortest].present = 1;
							current_pedestrian[second_shortest].appear_time = 0;
					current_ID.used_ID[4] = 1;
					//current_pedestrian[i].last_appear = 0;
						}
						else{
							current_pedestrian[second_shortest].id = 5;
							current_pedestrian[second_shortest].present = 1;
							current_pedestrian[second_shortest].appear_time = 0;
					current_ID.used_ID[5] = 1;
					//current_pedestrian[i].last_appear = 0;
						}
#endif
			}
		}
		}
		}
		}
}
		

void DATMO::writePosition(vector<Geom2D::Pedestrian>& current_pedestrian){
	FILE * pFile;
	int i = 0;
	int s;
	bool found = false;
	double x;
	double y;
	char cha_x[10];
	char cha_y[10];
	s = current_pedestrian.size();
	for (i = 0; i < s; i++){
		if (current_pedestrian[i].id == 1){
			x = current_pedestrian[i].x;
			y = current_pedestrian[i].y;
			found = true;
			break;
		}
	}
	if (found == false){
		x = 0;
		y = 0;
	}
	//x = 0.25;	
	//sprintf(cha_x,"%.2f",x);
	//sprintf(cha_y,"%.2f",y);
	pFile = fopen( "d:\\position.txt","a");
	fprintf(pFile,"%.2f ",y);
	//fwrite (cha_x, 1, sizeof(cha_x), pFile);
	//fwrite (cha_y, 1, sizeof(cha_y), pFile);
	fclose(pFile);
}


void DATMO::getStaticReadings(vector<double>& range, vector<double>& rad){

	int s=Lastrange.size();
	range.clear();
	rad.clear();


	for(int i=0;i<s;++i){
		if(rtype[i] != 1){
			range.push_back(Lastrange[i]);
			rad.push_back(Lastrad[i]);
		}
		/*else{
			range.push_back(-Lastrange[i]);
			rad.push_back(-Lastrad[i]);
			
		}*/
	}

}



void DATMO::getMOs(vector<Point>& MOs){

	Point nMO;
	int s=Lastrange.size();
	MOs.clear();

	Transform2D t(Lastpose);

	for(int i=0;i<s;++i){
		if(rtype[i] == 1){
			nMO.x = Lastrange[i]*cos(Lastrad[i]);
			nMO.y = Lastrange[i]*sin(Lastrad[i]);
			t.transform_to_global(nMO);
			nMO.relative_range = Lastrange[i];
			nMO.relative_rad = Lastrad[i];
			nMO.index = i;
			MOs.push_back(nMO);
		}
	}

}
void DATMO::getCenter(Point & center){
	
	Transform2D t(Lastpose);
	t.transform_to_global(center);

}
                                                
void DATMO::getCMOs(vector<Point>& CMOs){ 

	int s=MOposes.size();
	CMOs.resize(s);
	copy(MOposes.begin(),MOposes.end(),CMOs.begin());
}


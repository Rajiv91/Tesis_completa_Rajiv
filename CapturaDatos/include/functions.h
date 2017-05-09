
#ifndef __ETIQUETACION__
#define __ETIQUETACION__
#include <cv.h>
#include <highgui.h>
//#include <Mosaic.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <opencv2/contrib/contrib.hpp>
#include <time.h>

using namespace std;
using namespace cv;

#define SX 6 
#define SY 7 
#define SZ 8 

void getCoordinate(Point3f& coordinates, string line)
{
  int pos=0, pos_ant=0;
  string SxTemp, SyTemp, SzTemp;
  for(int i=0; i<9; i++)
  {
    pos=line.find(',', pos_ant);
    if(i==SX)
      SxTemp=line.substr(pos_ant, pos-pos_ant).c_str();
    if(i==SY)
      SyTemp=line.substr(pos_ant, pos-pos_ant).c_str();
    if(i==SZ)
      SzTemp=line.substr(pos_ant, pos-pos_ant).c_str();
    pos_ant=pos+1;
  }
  coordinates.x=atof(SxTemp.c_str());
  coordinates.y=atof(SyTemp.c_str());
  coordinates.z=atof(SzTemp.c_str());
  //coordinates.x=-W/2;
  //coordinates.y=CY;
  cout<<"Sx = "<<coordinates.x<<", Sy = "<<coordinates.y<<", Sz = "<<coordinates.z<<endl;
}


#endif

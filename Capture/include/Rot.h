#ifndef __ROT__
#define __ROT__

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include "Tools.h"

using namespace std;
using namespace cv;


class Rotation
{
  public:
    P* ptrP1;//Apuntador a P1
    P* ptrP2;//Apuntador a P2
    Mat mRot;
    Mat Rx, Ry;
    Rotation(P*, P*);
    double thetaX;//Diferencia en X
    double thetaY;//Diferencia en Y
    //void setVals(Params* ptrParams);
};

Rotation::Rotation(P* ptrP1_2, P* ptrP2_2)
{
  ptrP1=ptrP1_2;
  ptrP2=ptrP2_2;
  thetaX=ptrP1->phix-ptrP2->phix;
  thetaY=ptrP1->phiy-ptrP2->phiy;
  Rx=(Mat_<float>(3, 3)<<1, 0, 0, 0, cos(thetaX), -sin(thetaX), 0, sin(thetaX), cos(thetaX));
  Ry=(Mat_<float>(3, 3)<<cos(thetaY), 0, sin(thetaY), 0, 1, 0, -sin(thetaY), 0, cos(thetaY));
  mRot=Rx*Ry;
}

class RotR1
{
  public:
    P* myP;
    vector<vector<Rotation *> > vRotR1;
    RotR1(P*, vector<vector<P *> >& );
    void saveRots(ofstream &file, float** vThres);

};

RotR1::RotR1(P* ptrP1_3, vector<vector<P *> >& myVecP)
{
  myP=ptrP1_3;
  vRotR1.resize(myVecP.size());//Reservamos memoria
  for(int i=0; i<myVecP.size(); i++)
  {
    vRotR1[i].resize(myVecP[i].size());
    for(int j=0; j<myVecP[i].size(); j++)
    {
      vRotR1[i][j]=new Rotation(ptrP1_3, myVecP[i][j]);
    }
  }
}

void RotR1::saveRots(ofstream &file, float** vThres)
{
  for(int i=0; i<vRotR1.size(); i++)
  {
    for(int j=0; j<vRotR1[i].size(); j++)
    {
      if(fabs(vRotR1[i][j]->thetaX)>*vThres[0] and fabs(vRotR1[i][j]->thetaY)>*vThres[1])
      {
        if(myP->visualField and vRotR1[i][j]->ptrP2->visualField)//Está dentro del campo visual
        {
          file<<myP->Fx<<", "<<myP->Fy<<", "<<myP->Fz<<", "<<vRotR1[i][j]->ptrP2->Fx<<", "<<vRotR1[i][j]->ptrP2->Fy<<", "<<vRotR1[i][j]->ptrP2->Fz<<", "<<myP->ptrS->Sx<<", "<<myP->ptrS->Sy<<", "<<myP->ptrS->Sz<<", "<<vRotR1[i][j]->thetaX<<", "<<vRotR1[i][j]->thetaY<<endl;
        //file<<myP->Px<<", "<<myP->Pz<<", "<<vRotR1[i][j]->ptrP2->Px<<", "<<vRotR1[i][j]->ptrP2->Pz<<", "<<myP->ptrS->Sx<<", "<<myP->ptrS->Sy<<", "<<vRotR1[i][j]->thetaX<<", "<<vRotR1[i][j]->thetaY<<endl;
        }
      }
    }
   //     file<<endl;
  }
  //file<<endl;
  
}

void saveRots(vector<vector<RotR1> >& vAllRot, ofstream &out, float** vThres)
{
  for(int i=0; i<vAllRot.size(); i++)
  {
    for(int j=0; j<vAllRot[i].size(); j++)
    {
      vAllRot[i][j].saveRots(out, vThres);
    }
  }
}

/*void RotR1::saveRots(ofstream &file, float** vThres)
{
  file<<"***********De Px = "<<myP->Px<<", Py = "<<myP->Py<<", Pz = "<<myP->Pz<<"*************"<<endl;
  for(int i=0; i<vRotR1.size(); i++)
  {
    for(int j=0; j<vRotR1[i].size(); j++)
    {
      if(fabs(vRotR1[i][j]->thetaX)>*vThres[0] and fabs(vRotR1[i][j]->thetaY)>*vThres[1])
      {
        file<<"Hacia Px = "<<vRotR1[i][j]->ptrP2->Px<<", Py = "<<vRotR1[i][j]->ptrP2->Py<<", Pz = "<<vRotR1[i][j]->ptrP2->Pz<<",\tpos (i, j) = "<<i<<", "<<j<<endl;
        file<<"thetaX = "<<vRotR1[i][j]->thetaX<<", thetaY = "<<vRotR1[i][j]->thetaY<<".   \n";
        file<<"Rotación RxRy = "<<endl<<vRotR1[i][j]->mRot<<"\n";
      }
    }
        file<<endl<<endl;
  }

  
}

void saveRots(vector<vector<RotR1> >& vAllRot, char* dirName, float** vThres)
{

  for(int i=0; i<vAllRot.size(); i++)
  {
    for(int j=0; j<vAllRot[i].size(); j++)
    {
      string fileName;//="rotaciones1.txt";
      stringstream stream;
      stream<<dirName<<i<<"_"<<j<<".txt";
      fileName=stream.str();
      ofstream out(fileName);
      vAllRot[i][j].saveRots(out, vThres);
      out.close();
    }
  }
}*/





#endif

#ifndef __TOOLS__
#define __TOOLS__

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <cstdlib>
#include <cstdio>

#define VGA 0
#define HD 0
#define FULL_HD 1
#define IM_WIDTH_VGA 640
#define IM_HEIGHT_VGA 480
#define IM_WIDTH_HD 1280
#define IM_HEIGHT_HD 720
#define IM_WIDTH_FULLHD 1920
#define IM_HEIGHT_FULLHD 1080


using namespace std;
using namespace cv;

float W=.48;//1.095;
  float L=.27;//0.615;
  float Ly=1.39;
 float Dz=2;
 float Hy=1;//Debería ser 1.65m
 float a=0;
 float cy=0.037;
 float x=0;//W/2;
 float y=0;//L/2;//x, y con respecto al grid de la pantalla
 float dcu=0.;//Distancia del eje de rotación (y) de la unidad pan-tilt a la cámara

struct Params//Parámetros de las dimensiones de la persona, la pantalla donde se despliega el objeto...
{
  float W;
  float L;
  float Ly;
  float Dz;
  float Hy;
  float a;
  float cy;
  float y;//x, y con respecto al grid de la pantalla
  float x;
  float dcu;

  Params(float W2, float L2, float Ly2, float Dz2, float Hy2, float a2, float cy2, float y2, float x2, float dcu2)
  {
  W=W2;
  L=L2;
  Ly=Ly2;
  Dz=Dz2;
  Hy=Hy2;
  a=a2;
  cy=cy2;
  y=y2;//x, y con respecto al grid de la pantalla
  x=x2;
  dcu=dcu2;
  //   cout<<"struct Params. "<<Dz2<<endl; 
  }

};

class S
{
  public:
    float Sx;
    float Sy;
    float Sz;
    Mat rotX;
    S(float);
    S(Point3f&);
    void setVals(Params* ptrParams, float );

};

S::S(Point3f& coordinate3d)//Ya la coordenada 3d con respecto a la cámara
{
  Sx=coordinate3d.x;
  Sy=coordinate3d.y;
  Sz=coordinate3d.z;
  
}

S::S(float rotPT)//objeto que representa el objeto en la pantalla
{
  Sx=0;
  Sy=0;
  Sz=0;
  rotX=(Mat_<float>(3,3)<<1, 0, 0,
                          0, cos(rotPT), -sin(rotPT),
                          0, sin(rotPT), cos(rotPT));
  
}

void S::setVals(Params* ptrParams, float rotPT)
{
  //Se pasan al marco de referencia de la cámara, x y y tienen como marco de referencia la esquina superior izq de la pantalla
  //Sx=ptrParams->x-(ptrParams->W/2);//Suponiendo que la cámara se pone en el centro de la pantalla
  Sx=(ptrParams->W/2)-ptrParams->x;//Suponiendo que la cámara se pone en el centro de la pantalla
  //Sz=-(ptrParams->cy+ptrParams->y);
  //Sy=-(ptrParams->dcu+ptrParams->cy+ptrParams->y);
  Sy=(ptrParams->dcu+ptrParams->cy+ptrParams->y);//Se le suma la distancia de la pantalla a la cámara en Y
  Sz=0;

//  Point3f pS(Sx, Sy, Sz);
  Mat pS=(Mat_<float>(3,1)<<Sx,
                            Sy,
                            Sz);

  //Trasladamos el punto hacia el eje de rotación de la cámara
  pS.at<float>(1,0)-=ptrParams->dcu;
  //Rotamos el punto tantos grados que rote la unidad pt alrededor del eje X
  pS=rotX*pS;
  //Devolvemos el punto sobre el eje 
  pS.at<float>(1,0)+=ptrParams->dcu;

  Sx=pS.at<float>(0,0);
  Sy=pS.at<float>(1,0);
  Sz=pS.at<float>(2,0);

}

class P
{
  public:
  float Px;  
  float Py;
  float Pz;
  float Fx;  //Del piso, floor
  float Fy;
  float Fz;
  float *vP[3];//={Sx, Sy, Sz};
  float phix;
  float phiy;
  float gazeX;
  float gazeZ;
  bool visualField;//Está dentro del campo visual
  S* ptrS;
  P();
  void computeA(S* myS);
  void isInVisualField(Mat& K);

};

P::P()
{
  Px=0;
  Py=0;
  Pz=0;
  Fx=0;
  Fy=0;
  Fz=0;
  vP[0]=&Px;
  vP[1]=&Py;
  vP[2]=&Pz;
  phix=0;
  phiy=0;
  gazeX=0;
  gazeZ=0;
  visualField=false;

}

void P::computeA(S* myS)//Calcula los ángulos phix y phiy de cada P
{
  float vx=myS->Sx-Px;
  //cout<<"myS->Sx = "<<myS->Sx<<endl<<"Px = "<<Px<<endl;
  float vy=myS->Sy-Py;
  float vz=myS->Sz-Pz;
  float norm=sqrt(vx*vx+vy*vy+vz*vz);
  ptrS=myS;
  phix=acos(vx/norm);
  phiy=acos(vy/norm);
  //cout<<"phix = "<<phix<<endl;

}


struct PosP//Las posiciones de las P en el área con respecto a un marco de referencia
{
  float startX;
  float skipX;
  float startZ;
  float skipZ;
  PosP(float startX2, float skipX2, float startZ2, float skipZ2) {
  startX=startX2;
  skipX=skipX2;
  startZ=startZ2;
  skipZ=skipZ2;
    
  }
};

/*void iniP(PosP* ptrPosP, vector< vector<P *> > myVecP)
{
  
  float z=ptrPosP->startZ;
  for(int i=0; i<myVecP.size(); i++, z+=ptrPosP->skipZ)
  {
    float x=ptrPosP->startX;
    for(int j=0; j<myVecP[0].size(); j++, x+=ptrPosP->skipX)
    {
      //myVecP[i][j]=new P();
      myVecP[i][j]->Px=x;
      myVecP[i][j]->Pz=z;
    }
  }

      //cout<<"("<<myVecP[5][5]->Px<<", "<<myVecP[0][0]->Pz<<"),\t"<<endl;
}*/

void printPos(vector< vector<P *> > myVecP)
{
  
  for(int i=0; i<myVecP.size(); i++)
  {
    
    for(int j=0; j<myVecP[0].size(); j++)
    {
      cout<<"("<<myVecP[i][j]->Px<<", "<<myVecP[i][j]->Py<<", "<<myVecP[i][j]->Pz<<").- "<<myVecP[i][j]->phix<<","<<myVecP[i][j]->phiy<<"\t";
    } 
    cout<<endl;
  
  }

}

void computeAngles(vector< vector<P *> > &myVecP, S* myS)
{

  for(int i=0; i<myVecP.size(); i++)
  {
    for(int j=0; j<myVecP[0].size(); j++)
    {
      myVecP[i][j]->computeA(myS);
    } 
  
  }
}

void setRTK(Mat& R, Mat& T, Mat& K)
{
  
#if VGA
#elif HD
#else //FULL_HD
  K=(Mat_<float>(3,3)<<1.3894941650485114e+03, 0, 9.3813807262703551e+02,
                        0, 1.3894941650485114e+03, 5.1721974936200786e+02,
                        0, 0, 1);
  /*
  K.at<float>(0,0)=1.3894941650485114e+03;
  K.at<float>(0,1)=0;
  K.at<float>(0,2)=9.3813807262703551e+02;

  K.at<float>(1,0)=0;
  K.at<float>(1,1)=1.3894941650485114e+03;
  K.at<float>(1,2)=5.1721974936200786e+02;

  K.at<float>(2,0)=0;
  K.at<float>(2,1)=0;
  K.at<float>(2,2)=1;
  */


  //cout<<K<<endl;
#endif
  R=(Mat_<float>(3,3)<<1.0279165640074464e+00, 4.5202108407790490e-03, -3.1629671165730858e-02,
                      2.4089452053237613e-02, 5.6565105440460550e-01, 8.5085506480524475e-01,
                      2.0989680890192895e-02, -8.5118400519448045e-01, 5.6526998048841692e-01);
  T=(Mat_<float>(3,1)<<-3.6657841287256104e-01,
                      -9.7685753965074021e-02,
                      2.1662862665611806e+00);
}

void P::isInVisualField(Mat& K)
{
  int width, height;
#if VGA
#elif HD
#else //FULL_HD
  width=IM_WIDTH_FULLHD;
  height=IM_HEIGHT_FULLHD;
#endif
  Mat tempF=(Mat_<float>(3, 1)<<Fx,
                                Fy,
                                Fz);
  Mat tempP=(Mat_<float>(3, 1)<<Px,
                                Py,
                                Pz);
  tempF=K*tempF;//Se proyecta en la imagen
  tempF(Rect(0, 0, 1, 3)) /= tempF.at<float>(2, 0);
  tempP=K*tempP;
  tempP(Rect(0, 0, 1, 3)) /= tempP.at<float>(2, 0);

  //if((int)tempF.at<float>(0,0)>0 and (int)tempF.at<float>(0,0)<width and (int)tempF.at<float>(1,0)>0 and (int)tempF.at<float>(1,0)<height)//Está dentro del ancho y largo de la imagen el piso 
  //{
    if((int)tempP.at<float>(0,0)>0 and (int)tempP.at<float>(0,0)<width and (int)tempP.at<float>(1,0)>0 and (int)tempP.at<float>(1,0)<height)//lo mismo pero con la cabeza
    {
      visualField=true;
    }
  //}


  
}




#endif

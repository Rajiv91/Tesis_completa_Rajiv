//Dimensiones del tablero 12x12cm
#include <iostream> 
#include <cstdlib>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <highgui.h>
#include "opencv2/opencv.hpp"
#include "time.h"
#include <opencv2/highgui/highgui.hpp>
#include <Tools.h>
#include <Tools2.h>
#include <functions.h>
#include <levenberg.h>


using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  Mat frame, frameOriginal;
  VideoCapture capture;
  char key; 
  int filsGrid=8;
  int colsGrid=6;
  int nP=filsGrid*colsGrid;//número de puntos
    //*************Cámara**********
    cvNamedWindow("frame",WINDOW_NORMAL );//WINDOW_NORMAL);//WINDOW_AUTOSIZE );
    if (!capture.open(0))
    {
      cout<<"no pudo abrir la camara"<<endl; //si no se puede acceder a la cámara USB usa la webcam de la laptop
      return -1;
    }
        cout<<"Se abrió la cámara"<<endl;
    setRes(capture);//Se configura la resolución con la que se va a trabajar
    setCam(capture);//Se alinea la cámara con el tablero
    Mat K, scnPts, imgPts, RT, dummy, testPoints;
    vector<Point2d>P;
    for(int i=0; i<nP; i++)
      P.push_back(Point2d(0,0));//Se reserva memoria

    findCorners(capture, filsGrid, colsGrid, P);//Encuentra las intersecciones del tablero
   
    //Se crea la matriz con los puntos detectados del tablero automaticamente
    setImgP(P, imgPts);

    //cout<<"imgPts = "<<imgPts<<endl;
    buildScnP( filsGrid, colsGrid, scnPts);//Se crea la matriz con los puntos 3d de la escena

    //cout<<"scnPts = "<<scnPts<<endl;
    ParamsK pK(616.164, 616.82, 0, 325.528, 228.66);
     K=(Mat_<double>(3, 3)<<pK.fx, pK.gamma, pK.cx, 0, pK.fy,  pK.cy, 0, 0, 1);
    dummy=(Mat_<double>(3, 4)<<1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    cout<<"K = "<<K<<endl<<"Dummy = "<<dummy<<endl;

    Point3f N;
    double D;
    //cout<<"scnPts = "<<scnPts<<endl<<"imgPts="<<imgPts<<endl;

    //Se pasan a metros
    imgPts=K.inv()*imgPts;
    //Normalizamos las coordenadas obtenidas.
    for (int i=0;i<imgPts.cols;++i)
        imgPts(Rect(i, 0, 1, 3)) /= imgPts.at<double>(2, i);

    //Invocamos funciones que calcula la ecuación del plano en el marco de referencia de la cámara.
    Mat R, T, marcas3d, marcas2d;
    getPlaneOrientation2(scnPts, imgPts, N, D, R, T);//La función también calcula la R y T
    //Construimos la matriz G
    Size_<int> sizeG(4,4);
    Mat G(sizeG, R.type());
    buildG(G, R, T);
    cout<<endl<<"R= "<<R<<endl<<"T= "<<T<<endl<<"G= "<<G<<endl;

    //Se obtiene la linea de fuga
    Point3f vanishingLine;
    getVanishingLine (K, N, vanishingLine);//La linea de fuga es devuelta en m
    Scalar_ <uchar> colors[4]; 
    colors[0] = Scalar_<uchar> (0,0,255);
    colors[1] = Scalar_<uchar> (255,0,0);
    colors[2] = Scalar_<uchar> (0,255,0);
    colors[3] = Scalar_<uchar> (255,255,255);

    marcas3d=G*scnPts;//Rotamos y trasladamos los puntos 3d 
    marcas2d=K*dummy*marcas3d;//Los proyectamos en la imagen
//Deshomoge...
    for (int i=0;i<marcas2d.cols;++i)
        marcas2d(Rect(i, 0, 1, 3)) /= marcas2d.at<double>(2, i);

#if 1
    vector<Point2f>vps2d;
    for(int i=0; i<nP; i++)
      vps2d.push_back(Point2f(0,0));
    pointsSelect3dTo2D(P, vps2d, K, N, D, dummy);//Con los puntos seleccionados del tablero se halla la intersección en el plano y se proyectan en 2D

    //cout<<"Número de marcas verdes: "<<marcas2d.cols<<endl;
      capture>>frame;
      frameOriginal=frame.clone();
      for(int j=0; j<marcas2d.cols; j++)
      {
        drawLine(frame, vanishingLine, colors[0]);//Se pinta la linea de fuga
        for(int i=0; i<vps2d.size(); i++)
          circle(frame, vps2d[i],5, Scalar(0, 0, 200) ,5);//Se pintan los puntos de las intersecciones

        Point tempPoint(marcas2d.at<double>(0, j), marcas2d.at<double>(1, j));
        circle(frame, tempPoint,5, Scalar(0, 200, 0) ,10);//Se pintan los puntos rotados y trasladados
      }
    
      imshow("frame", frame);
      key=waitKey(0);
      levenbergMain(R, T, K, frameOriginal, imgPts, scnPts, P);

#endif

  return 0;
}

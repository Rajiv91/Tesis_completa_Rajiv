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


using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  Mat frame;
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
    Mat R, T;
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

   PosP posP(.0, 0.12, 0.0 , .12);
    int cols=4;//4;
    int rows=3;//18;
   
  float y=posP.startZ;
  float z=0;
  float xTemp, yTemp, zTemp;
  Mat marcas3d, colTemp, marcas2d, center3d, center2d; 
  xTemp=0;
  zTemp=1;
//  yTemp=(D-N.x*xTemp-N.z*zTemp)/N.y;

  center3d =(Mat_<double>(4,1)<<xTemp,yTemp,zTemp,1);

  for(int i=0; i<rows; i++, y+=posP.skipZ)
  {
    float x=posP.startX;
    for(int j=0; j<cols; j++, x+=posP.skipX )
    {
      //y=(D-N.x*x-N.z*z)/N.y;
      //z=(D-N.x*x-N.y*y)/N.z;
    z=0;
      if(i==0 and j==0)
      {
        marcas3d=(Mat_<double>(4, 1)<<x, y, z, 1);
      }
      else
      {
        colTemp=(Mat_<double>(4, 1)<<x, y, z, 1);
        hconcat( marcas3d, colTemp,marcas3d);
      }
    }
  }
    //cout<<"Matriz marcas3d: "<<endl<<marcas3d<<endl;
    marcas3d=G*scnPts;
    marcas2d=K*dummy*marcas3d;
    center2d=K*dummy*center3d;
#if 0
//Deshomoge...
    center2d(Rect(0, 0, 1, 3)) /= center2d.at<double>(2, 0);
    Point centerP(center2d.at<double>(0,0), center2d.at<double>(1,0));
    cout<<"Punto central en la imagen: "<<centerP<<endl;
    cout<<"Punto central en 3d: "<<center3d<<endl;
    for (int i=0;i<marcas2d.cols;++i)
        marcas2d(Rect(i, 0, 1, 3)) /= marcas2d.at<double>(2, i);

  //cout<<"Marcas 2d rows: "<<marcas2d.rows<<endl<<"cols: "<<marcas2d.cols<<endl;

    Point3f cornerTL, cornerTR, cornerBL, cornerBR;
    vector<Point3f*>vCorners;
    vCorners.push_back(&cornerTL);
    vCorners.push_back(&cornerBL);
    vCorners.push_back(&cornerBR);
    vCorners.push_back(&cornerTR);

    vector<Point2f>vCorners2D;
    vCorners2D.push_back(Point2f(0,0));
    vCorners2D.push_back(Point2f(0,0));
    vCorners2D.push_back(Point2f(0,0));
    vCorners2D.push_back(Point2f(0,0));

    getIntersec(frame, vCorners, vanishingLine, K, N, D);//Devuelve las intersecciones al plano
    //getIntersec2(frame, vCorners, vanishingLine, K, N, D, P);
    points3dTo2d(vCorners, K, dummy, vCorners2D);//Convierte el vector de las intersecciones de 3d a 2d para visualización
    vector<Point2f>vps2d;
    for(int i=0; i<nP; i++)
      vps2d.push_back(Point2f(0,0));
    pointsSelect3dTo2D(P, vps2d, K, N, D, dummy);//Con los puntos seleccionados con el mouse se halla la intersección en el plano y se proyectan en 2D

        //Point tempPoint(P2DCorner.at<double>(0, 0), P2DCorner.at<double>(1, 0));
        //circle(frame, vCorners2D[0],5, Scalar(200, 0, 0) ,10);
        //cout<<"Punto en la imagen: "<<vCorners2D[0]<<endl;
        //cout<<"Matriz 3d en la imagen"<<endl<<marcas2d<<endl;
//Pinta en el frame
/*cout<<endl<<endl<<"Marcas verdes"<<endl;
    for(int i=0; i<8; i++)
    {
        Point3f tempPoint(marcas3d.at<double>(0, i), marcas3d.at<double>(1, i), marcas3d.at<double>(2, i));
        //Point2f tempPoint(marcas2d.at<double>(0, i), marcas2d.at<double>(1, i));
        cout<<tempPoint<<", "<<endl;
    }*/
    cout<<endl<<endl;
    //cout<<"Número de marcas verdes: "<<marcas2d.cols<<endl;
    for(;;)
    {
      capture>>frame;
      //flip(frameLine, frameLine, 1);
      for(int j=0; j<marcas2d.cols; j++)
      {
        drawLine(frame, vanishingLine, colors[0]);
        for(int i=0; i<vCorners2D.size(); i++)
          circle(frame, vCorners2D[i],5, Scalar(200, 0, 0) ,10);
        for(int i=0; i<vps2d.size(); i++)
          circle(frame, vps2d[i],5, Scalar(0, 0, 200) ,5);

        Point tempPoint(marcas2d.at<double>(0, j), marcas2d.at<double>(1, j));
        circle(frame, tempPoint,5, Scalar(0, 200, 0) ,10);
        //circle(frame, centerP,5, Scalar(200, 200, 200) ,10);//Punto central de la cámara
      }
    
      imshow("frame", frame);
      key=waitKey(1);
      if(key=='q') break;
    }
#endif

  return 0;
}

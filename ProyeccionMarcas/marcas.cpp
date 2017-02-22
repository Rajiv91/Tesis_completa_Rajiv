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

int main(int argc, char **argv)
{

//****************Cámara****
  Mat frame;
  VideoCapture capture;
  char key; 
  int filsGrid=8;
  int colsGrid=6;
  int nP=filsGrid*colsGrid;//número de puntos
    //*************Cámara**********
    cvNamedWindow("frame",WINDOW_NORMAL );//WINDOW_NORMAL);//WINDOW_AUTOSIZE );
    if (!capture.open(1))
    {
      cout<<"no pudo abrir la camara, se usara la webcam integrada"<<endl; //si no se puede acceder a la cámara USB usa la webcam de la laptop
      capture.open(0);
    }
    if(!capture.isOpened())
    {
      cout<<"No abre"<<endl;
      return -1;
    }
        cout<<"Se abrió la cámara"<<endl;
    #if FULL_HD
    capture.set(CV_CAP_PROP_FRAME_WIDTH,1920);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    #elif HD
    capture.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    #else
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    #endif

    setCam(capture);//Se alinea la cámara con el tablero
    Mat K, scnPts, imgPts, RT, dummy, testPoints, I, Corners, Icol;
    vector<Point2d>P;
    capture>>Icol;//Se captura el tablero para encontrar las intersecciones
    //Icol=imread(argv[1],1);
    cvtColor(Icol, I, CV_RGB2GRAY);
    cvNamedWindow("chessboard",WINDOW_NORMAL );
    if(findChessboardCorners(I, Size(colsGrid, filsGrid), Corners))
    {
            cornerSubPix(I, Corners, Size(10,10) , Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            for (int j = 0; j < Corners.rows; ++j)
            {
                Vec2f *apuv = Corners.ptr<Vec2f>(j);
                circle(Icol, Point((*apuv)[0], (*apuv)[1]) , 5,  Scalar_ <uchar> (255,64,255), 3);
                P.push_back(Point2d((*apuv)[0], (*apuv)[1]));
                stringstream textTemp;
                textTemp<<j;
                putText(Icol,textTemp.str(),  Point((*apuv)[0], (*apuv)[1]),FONT_HERSHEY_COMPLEX_SMALL,1,CV_RGB(0, 255, 0),2 );
            }
            imshow("chessboard", Icol);
      cout<<"Se encontaron "<<P.size()<<" intersecciones"<<endl;
      waitKey(0);
    }
    else
    {
      cout<<"No se encontraron intersecciones!!"<<endl;
      exit(0);
    }
    
    //Se crea la matriz con los puntos detectados del tablero automaticamente
    imgPts=(Mat_<double>(3,1)<<P[0].x,
                                P[0].y,
                                1);
    for(int i=1; i<Corners.rows; i++)
    {
      Mat temp=(Mat_<double>(3,1)<<P[i].x,
                                  P[i].y,
                                  1);
      hconcat(imgPts,temp,imgPts);
    }
    cout<<"imgPts = "<<imgPts<<endl;
    float startX3d, startY3d;
    startX3d=startY3d=0;
    float x3d=startX3d;
    float y3d=startY3d;
    float skipX3d, skipY3d;
    skipX3d=skipY3d=.12;
    y3d=startY3d;
    //Armamos la matriz con los puntos 3d
    for(int i=0 ; i<filsGrid; i++, y3d+=skipY3d)
    {
      x3d=startX3d;
      for(int j=0 ; j<colsGrid; j++, x3d+=skipX3d)
      {
        if(i==0 and j==0)
          scnPts = (Mat_<double>(4,1) << x3d,y3d, 0, 1);
        else
        {  
          Mat temp=(Mat_<double>(4,1)<<x3d, y3d, 0, 1);
          hconcat(scnPts, temp, scnPts);
        }
      }
    }
    cout<<"scnPts = "<<scnPts<<endl;
    //return 0;
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

    //Invocamos funciones que calcula la ecuación del plano en el marco
    //de referencia de la cámara.
    Mat R, T;
    getPlaneOrientation2(scnPts, imgPts, N, D, R, T);
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

    cout << "N = " << N << endl;
    cout << "D = " << D << endl;

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


  return 0;
}

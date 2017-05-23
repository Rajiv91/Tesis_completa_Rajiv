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
  int width, height;
  int filsGrid=8;
  int colsGrid=6;
  int nP=filsGrid*colsGrid;//número de puntos
    //*************Cámara**********
    cvNamedWindow("frame",WINDOW_NORMAL );//WINDOW_NORMAL);//WINDOW_AUTOSIZE );
    if (!capture.open(1))
    {
      cout<<"no pudo abrir la camara"<<endl; //si no se puede acceder a la cámara USB usa la webcam de la laptop
      return -1;
    }
        cout<<"Se abrió la cámara"<<endl;
    setRes(capture);//Se configura la resolución con la que se va a trabajar
    width=capture.get(CV_CAP_PROP_FRAME_WIDTH);
    height=capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    setCam(capture);//Se alinea la cámara con el tablero
    Mat K, scnPts, imgPts, RT, dummy, testPoints, distCoeffs, Kn, Mx, My;
    vector<Point2d>P;

    //ParamsK pK(614.164, 616.82, 0, 325.528, 228.66);
    ParamsK pK(1.3894941650485114e+03, 1.3894941650485114e+03, 0, 9.3813807262703551e+02, 5.1721974936200786e+02);//1080 resolución
    //ParamsK pK(9.2607934067336703e+02, 9.2607934067336703e+02, 0, 6.2871749856067345e+02, 3.4825100006740143e+02);//720
    //distCoeffs=(Mat_<double>(1,5)<<0.08330, -0.13005, -0.00592, 0.00336,0);
    distCoeffs=(Mat_<double>(1,5)<<1.1426750053108892e-01, -3.9359465775057556e-01,  1.0102007284106844e-03, -1.2427560403415342e-03,5.0042410869717813e-01);//1080p coeficientes
    //distCoeffs=(Mat_<double>(1,5)<<1.5067768106593274e-01, -3.1447277061339596e-01,  -4.8469436193665445e-04, 4.4105030703923609e-03,1.4270386203296379e-01);//720p coeficientes

    //ParamsK pK( 5.9426213201333087e+02, 5.9426213201333087e+02, 0, 3.1900282268700198e+02, 2.3307304762949084e+02);
     //distCoeffs=(Mat_<double>(5,1)<<-1.4020356562058382e-02, 7.6780093149023665e-01, -3.7043726642921131e-04, 4.7992812746827642e-03,-2.3914918133183756e+00);
     K=(Mat_<double>(3, 3)<<pK.fx, pK.gamma, pK.cx, 0, pK.fy,  pK.cy, 0, 0, 1);

    dummy=(Mat_<double>(3, 4)<<1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    cout<<"K = "<<K<<endl<<"Dummy = "<<dummy<<endl;
    Size ImageSize(width, height);
    Kn = getOptimalNewCameraMatrix(K, distCoeffs, ImageSize, 1, ImageSize, 0);
    initUndistortRectifyMap(K, distCoeffs, Mat::eye(3, 3, CV_32FC1), Kn, ImageSize, CV_16SC2, Mx, My);
    cout << "Kn = " << Kn << endl << endl;
    Mat I, O;
    capture>>I;
      //I = imread("chessboard.jpg", 1);
      //I = imread("/home/rajiv/Documentos/seminario3/chessboard1080/chessboard1_1080.jpg", 1);//el 2 es el que presenta más error, el 5
    //I=imread("/home/rajiv/Documentos/seminario3/logitech720Selec/L_000025.jpg",1);
      remap (I, O, Mx, My, INTER_LINEAR, BORDER_TRANSPARENT);
    //cvNamedWindow("antes",0);
    cvNamedWindow("despues",0 );
      //imshow("antes", I);
      imshow("despues", O);
      waitKey();


    for(int i=0; i<nP; i++)
      P.push_back(Point2d(0,0));//Se reserva memoria

    findCorners(O, filsGrid, colsGrid, P);//Encuentra las intersecciones del tablero
   
    //Se crea la matriz con los puntos detectados del tablero automaticamente
    setImgP(P, imgPts);

    //cout<<"imgPts = "<<imgPts<<endl;
    buildScnP( filsGrid, colsGrid, scnPts);//Se crea la matriz con los puntos 3d de la escena


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
    //frame=imread("chessboard.jpg",1);
    //frame=imread("/home/rajiv/Documentos/seminario3/chessboard720/chessboard3_720.jpg", 1);
    frame=O;
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
      cout<<"Antes de levenberg"<<endl;
      //cout<<"scnPts = "<<endl<<scnPts<<endl;
      key=waitKey(0);
      levenbergMain(R, T, K, frameOriginal, imgPts, scnPts, P);

#endif

  return 0;
}

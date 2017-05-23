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


using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  Mat frame, frameOriginal;
  VideoCapture capture;
  string line;
  Mat mPoints, K, mPointsP;
  K=(Mat_<float>(3,3)<<1.3894941650485114e+03, 0, 9.3813807262703551e+02,
                        0, 1.3894941650485114e+03, 5.1721974936200786e+02,
                        0, 0, 1);//1080
  capture.open(1);
  capture.set(CV_CAP_PROP_FRAME_WIDTH,1920);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

  capture>>frame;
  //frame = imread("/home/rajiv/Documentos/seminario3/experimentosMay17/tp-350_1080.jpg", 1);
  namedWindow("frame",0 );
  int coordinates=3;
  int pos, pos_ant;
  Vec3f vData;
  vector <string> v;
  //Lee el archivo con las mejores rotaciones
  char *dirFile;
  if(argc>1)
    dirFile=argv[1];
  else
  {
    cout<<"faltan parámetros"<<endl;
    exit(0);
  }
  cout<<dirFile<<endl;
  ifstream in(dirFile);
  while(getline(in, line))
    v.push_back(line);//Se cargan en un vector de strings los datos
  
  //for(int i=0; i<v.size(); i++)
    //cout<<v[i]<<endl;
  for(int i=0; i<v.size(); i++)
  {
  pos_ant=0;
    for (int j=0; j<coordinates; j++)
    {
	pos=v[i].find(',', pos_ant);
        vData[j]=(float)(atof(v[i].substr(pos_ant, pos-pos_ant).c_str()));
	pos_ant=pos+1;
    }
    if(i==0)//Primer punto
    {
      mPoints=(Mat_<float>(3, 1)<<vData[0],
                                  vData[1],
                                  vData[2]);
    }
    else
    {
      Mat temp=(Mat_<float>(3, 1)<<vData[0],
                                  vData[1],
                                  vData[2]);
      hconcat(mPoints,temp,mPoints);//Se arma la matriz con los puntos
    }
    //cout<<vData<<endl;
    if(i==v.size()-1)//Si es la última línea
    {
      for (int j=0; j<coordinates; j++)
      {
	pos=v[i].find(',', pos_ant);
        vData[j]=(float)(atof(v[i].substr(pos_ant, pos-pos_ant).c_str()));
	pos_ant=pos+1;
      }
      Mat temp=(Mat_<float>(3, 1)<<vData[0],
                                  vData[1],
                                  vData[2]);
      hconcat(mPoints,temp,mPoints);
      //cout<<"último punto"<<endl<<temp<<endl;
    }
  }
  //cout<<"Matriz de puntos "<<endl<<mPoints<<endl;
 // Se multiplican por la K para proyectar en la imagen, pasar de m a pixeles
 mPointsP=K*mPoints;
 //Deshomoge...
    for (int i=0;i<mPoints.cols;++i)
        mPointsP(Rect(i, 0, 1, 3)) /= mPointsP.at<float>(2, i);//En pixeles
//Se pintan los puntos en la imagen
    stringstream nPoint, pointCo;
    Point3f N=Point3d(-1.513929e-02, 9.542600e-01, 3.126682e-01), nNorm, pTemp2, pHeight;
    double d=  2.157112;
    float x, y, z, startXZ=0, norma;
    Mat headP;
    norma=norm(N);
    nNorm=Point3f(N.x/norma, N.y/norma, N.z/norma);
    nNorm=N;

    float heightP=1.67;//Altura de la persona
    pHeight=nNorm*-heightP;
    
    Point pTemp, hP;
    for(;;)
    {
      capture>>frame;

    for (int i=0;i<mPoints.cols;++i)
    {
      //capture>>frame;
      nPoint.str("");
      pointCo.str("");
      nPoint<<i;
      Point tempPoint(mPointsP.at<float>(0,i ), mPointsP.at<float>(1,i));
      Point3f tempPoint2(mPoints.at<float>(0,i ), mPoints.at<float>(1,i), mPoints.at<float>(2,i));
      pointCo<<tempPoint2;
      putText(frame,nPoint.str(),Point(tempPoint.x-5, tempPoint.y-10), FONT_ITALIC,3,CV_RGB(100, 0, 255),2);
      putText(frame,pointCo.str(),Point(tempPoint.x-5, tempPoint.y+25),FONT_ITALIC,1,CV_RGB(0, 51, 0),2);
      circle(frame, tempPoint,5, Scalar(0, 0, 250) ,10);
      cout<<tempPoint<<endl;
      //Dibujamos la altura de la persona
      tempPoint2+=pHeight;//Trasladamos hacia la marca del piso la altura de las personas
      Mat headP=(Mat_<float>(3,1)<<tempPoint2.x,
                                  tempPoint2.y,
                                  tempPoint2.z);
      headP=K*headP;//Se proyecta en la imagen
      headP(Rect(0, 0, 1, 3)) /= headP.at<float>(2, 0);
      hP=Point(headP.at<float>(0, 0), headP.at<float>(1, 0));
      //cv::line(frame, hP,tempPoint, Scalar(9, 167, 178) ,3);


      //Dibujamos alrededor del punto líneas que yacen en el plano
      x = mPoints.at<float>(0,i);
      z = mPoints.at<float>(2,i);
      //Reglas en z
      startXZ=-.5;
      for(int j=0; j<11; j++)
      {
        //capture>>frame;
        if(j==0)
          z+=startXZ;//10cm

        else
          z+=.1;//Saltos de 10 cm

        //cout<<"z="<<z<<endl;
        y=(d-N.x*x-N.z*z)/N.y;
        Mat tempP=(Mat_<float>(3, 1)<<x,
                                  y,
                                  z);
        tempP=K*tempP;
        tempP(Rect(0, 0, 1, 3)) /= tempP.at<float>(2, 0);
        pTemp=Point(tempP.at<float>(0, 0), tempP.at<float>(1, 0));
        circle(frame, pTemp,5, Scalar(30, 45, 131), 2);
        cv::line(frame, tempPoint, pTemp, Scalar(30, 45, 131), 1, CV_AA);
      }

      //Reglas en x
      z = mPoints.at<float>(2,i);
      for(int j=0; j<11; j++)
      {
        if(j==0)
          x+=startXZ;//10cm

        else
          x+=.1;//Saltos de 10 cm

        //cout<<"z="<<z<<endl;
        y=(d-N.x*x-N.z*z)/N.y;
        Mat tempP=(Mat_<float>(3, 1)<<x,
                                  y,
                                  z);
        tempP=K*tempP;
        tempP(Rect(0, 0, 1, 3)) /= tempP.at<float>(2, 0);
        pTemp=Point(tempP.at<float>(0, 0), tempP.at<float>(1, 0));
        circle(frame, pTemp,5, Scalar(30, 45, 131) ,2);
        cv::line(frame, tempPoint, pTemp, Scalar(30, 45, 131), 1, CV_AA);

      }

      //imshow("frame", frame);
      //waitKey(1);

    }
    imshow("frame", frame);
    waitKey(100);
    //sleep(1);
    }//for(;;)



  return 0;
}

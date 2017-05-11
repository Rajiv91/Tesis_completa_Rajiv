#ifndef __TOOLS2__
#define __TOOLS2__

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <cstdlib>
#include <cstdio>
#define IM_WIDTH_FULLHD 1920
#define IM_HEIGHT_FULLHD 1080


using namespace std;
using namespace cv;

void setCam(VideoCapture& capture)
{
    Mat frameLine;
    cvNamedWindow("line", WINDOW_NORMAL);
    //Acomodar la cámara
    for(;;)
    {
      capture>>frameLine;
      //flip(frameLine, frameLine, 1);
      line( frameLine, Point(IM_WIDTH_FULLHD/2, 0), Point(IM_WIDTH_FULLHD/2,  IM_HEIGHT_FULLHD), Scalar( 0, 0, 0 ), 5 );
      imshow("line", frameLine);
      char keyExit=waitKey(1);
      if(keyExit=='q') break;
    }
    destroyWindow("line");
  
}

void setP(vector<Point2d>& P)
{
  P[0]=Point2d(815,880);
  P[1]=Point2d(800,924);
  P[2]=Point2d(780, 979);
  P[3]=Point2d(754, 1054);
  P[4]=Point2d(957, 878);
  P[5]=Point2d(959, 927);
  P[6]=Point2d(957, 979);
  P[7]=Point2d(959, 1057);
  P[8]=Point2d(1099, 881);
  P[9]=Point2d(1114, 928);
  P[10]=Point2d(1134, 982);
  P[11]=Point2d(1160, 1058);
}
void fillMats(vector<Mat>& vMat)
{
  float width=0.33;//Ancho del piso
  float startX=-.99;
  float skipX=.33;
/*  for(int i=0; i<vMat.size(); i++)
  {
    Mat temp=(Mat_<double>(4,1)<<)
    vMat[i]=
    if(i!=0 and (i%3))
  }*/
}

void buildG(Mat& G, Mat&R, Mat& T)
{
  //cout<<endl<<"R dentro de la función= "<<R<<endl;
  /*Rect roi;
  G(roi(0,0, 3, 3));
  roi=R.clone();*/
  //G(Rect(1,0, 1, 3))=T.clone();
  //G=(Mat_<double>(4,1)<<R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), 0)
   //for(int i=1; i<3)
    //hconcat();
  
  //Rotation Mat
  G.at<double>(0, 0)=R.at<double>(0,0);
  G.at<double>(1, 0)=R.at<double>(1,0);
  G.at<double>(2, 0)=R.at<double>(2,0);
  G.at<double>(3, 0)=0;

  G.at<double>(0, 1)=R.at<double>(0,1);
  G.at<double>(1, 1)=R.at<double>(1,1);
  G.at<double>(2, 1)=R.at<double>(2,1);
  G.at<double>(3, 1)=0;

  G.at<double>(0, 2)=R.at<double>(0,2);
  G.at<double>(1, 2)=R.at<double>(1,2);
  G.at<double>(2, 2)=R.at<double>(2,2);
  G.at<double>(3, 2)=0;

  G.at<double>(0, 3)=T.at<double>(0,0);
  G.at<double>(1, 3)=T.at<double>(1,0);
  G.at<double>(2, 3)=T.at<double>(2,0);
  G.at<double>(3, 3)=1;
  //cout<<"G dentro de la función= "<<endl<<G<<endl;
}




#endif

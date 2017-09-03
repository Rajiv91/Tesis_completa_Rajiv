#ifndef __FUNCTIONS__
#define __FUNCTIONS__

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <cstdlib>
#include <cstdio>

#define HD 0
#define FULL_HD 1
#define IM_WIDTH_VGA 640
#define IM_HEIGHT_VGA 480
#define IM_WIDTH_HD 1280
#define IM_HEIGHT_HD 720
#define IM_WIDTH_FULLHD 1920
#define IM_HEIGHT_FULLHD 1080
#define IM_FIELD 0
#define IM_WIDTH 75
#define IM_HEIGHT 75
#define SCREEN_FIELD 5

#define W 2.375//Pintarrón en m

using namespace std;
using namespace cv;

int size=200;

void saveIm(string& tempS, Mat& trainingData, int nSample, int& nFeature)
{
  Mat frame=imread(tempS, 0);//Se carga en escala de grises
  //int nFeature=0;
  //namedWindow("window", 0);
  for(int i=0; i<frame.rows; i++)
  {
    for(int j=0; j<frame.cols; j++)
    {
      trainingData.at<float>(nSample, nFeature)=(float)frame.at<uchar>(i,j);
      nFeature++;
    }
  }
  //cout<<trainingData(Rect(0,0, nFeature, 1))<<endl;
  //imshow("window", frame );
  //waitKey();
  //cout<<"características = "<<nFeature<<endl;
}

//Etiqueta dependiendo la región de la pantalla en la que se encuentre
void labelScreen(float Sx, float Sy, float Sz, int nSample, Mat& trainingClasses)
{
  float screenCenter=(float)W/2.0;
  float screenSegment=(float)W/3.0;
  float firstSegment=-screenCenter+screenSegment;
  float secondSegment=-screenCenter+2*screenSegment;
  float thirdSegment=screenCenter;
  //|first seg. |second seg| third seg|
  //cout<<"first seg = "<<firstSegment<<endl<<"second seg = "<<secondSegment<<endl<<"third seg = "<<thirdSegment<<endl;
  //Para 3 segmentos de pantalla
  if(Sx<firstSegment)
  {
    trainingClasses.at<float>(nSample, 0)=1.0;
    trainingClasses.at<float>(nSample, 1)=0.0;
    trainingClasses.at<float>(nSample, 2)=0.0;
  }
  else if(Sx>firstSegment and Sx<secondSegment)
  {
    trainingClasses.at<float>(nSample, 0)=0.0;
    trainingClasses.at<float>(nSample, 1)=1.0;
    trainingClasses.at<float>(nSample, 2)=0.0;
  }
  else
  {
    trainingClasses.at<float>(nSample, 0)=0.0;
    trainingClasses.at<float>(nSample, 1)=0.0;
    trainingClasses.at<float>(nSample, 2)=1.0;
  }


//Para 2 segmentos en pantalla
  /*if(Sx<0)
  {
    trainingClasses.at<float>(nSample, 0)=0.0;
    trainingClasses.at<float>(nSample, 1)=1.0;
  }
  else
  {
    trainingClasses.at<float>(nSample, 0)=1.0;
    trainingClasses.at<float>(nSample, 1)=0.0;
  }*/

  //trainingData.at<float>(nSample, nFeature)=atof(sample.substr(pos_ant, pos-pos_ant).c_str());
}

void setSample(string& sample, Mat& trainingData, Mat& trainingClasses, int nSample, int colsFile)//Coloca la muestra en la fila correspondiente
{
  //cout<<sample<<endl;
  //Abrir la cámara en escala de grises
  int pos=0, pos_ant=0;
  string tempS;
  int nFeature=0;
  float Sx, Sy, Sz;
  for(int i=0; i<colsFile-2; i++)
  {
    pos=sample.find(',', pos_ant);
    if(i==IM_FIELD)
    {
      tempS=sample.substr(pos_ant, pos-pos_ant).c_str();//Se guarda la imagen como un vector
      saveIm(tempS, trainingData, nSample, nFeature);

    }
    else if (i>IM_FIELD and i<SCREEN_FIELD)//son características normales
    {
      trainingData.at<float>(nSample, nFeature)=atof(sample.substr(pos_ant, pos-pos_ant).c_str());
      //cout<<trainingData.at<float>(nSample, nFeature)<<endl;
      //Mat temp=trainingData(Rect(0, nSample, trainingData.cols, 1));
      //cout<<temp<<endl;//<<tempS<<endl;
      nFeature++;
      
    }
    else//Las etiquetas de la pantalla
    {
      Sx=atof(sample.substr(pos_ant, pos-pos_ant).c_str());
      pos_ant=pos+1;

      pos=sample.find(',', pos_ant);
      Sy=atof(sample.substr(pos_ant, pos-pos_ant).c_str());
      pos_ant=pos+1;

      pos=sample.find(',', pos_ant);
      Sz=atof(sample.substr(pos_ant, pos-pos_ant).c_str());
      labelScreen(Sx, Sy, Sz, nSample, trainingClasses);
      //cout<<"Sx = "<<Sx<<endl<<"Sy = "<<Sy<<endl<<"Sz = "<<Sz<<endl;
      //nFeature++;
    }
    pos_ant=pos+1;
    
  }
  //cout<<tempS<<endl;
}


/*float evaluate(Mat& predicted, Mat& actual) {
	assert(predicted.rows == actual.rows);
	int t = 0;
	int f = 0;
	for(int i = 0; i < actual.rows; i++) {
		float p = predicted.at<float>(i,0);
		float a = actual.at<float>(i,0);
		if((p >= 0.5 && a >= 0.5) || (p <= 0.5 &&  a <= 0.5)) {
			t++;
		} else {
			f++;
		}
	}
	return (t * 1.0) / (t + f);
}*/

int maxPos(Mat& outNeurons)
{
  float max=outNeurons.at<float>(0,0);
  int maxIdx=0;
  for(int i=0; i<outNeurons.cols; i++)
  {
    if(outNeurons.at<float>(0,i)>max)
    {
      max=outNeurons.at<float>(0,i);
      maxIdx=i;
    }    
  }
  return maxIdx;
}

float evaluate(Mat& predicted, Mat& actual) {
	assert(predicted.rows == actual.rows);
	int t = 0;
	int f = 0;
        Mat roiTemp;
        //cout<<"num rows = "<<actual.rows<<endl;

	for(int i = 0; i < actual.rows; i++) 
        {
          roiTemp=predicted(Rect(0, i, predicted.cols, 1));
		int p = maxPos(roiTemp);
                  //predicted.at<float>(i,0);
          roiTemp=actual(Rect(0, i, actual.cols, 1));
		int a = maxPos(roiTemp);
                cout<<"p = "<<p<<" a = "<<a<<endl;
                  //actual.at<float>(i,0);
		if(p == a)
                {
			t++;
		} 
                else 
                {
			f++;
		}
	}
	return (t * 1.0) / (t + f);
}

// function to learn
int f(float x, float y, int equation) {
	switch(equation) {
	case 0:
		return y > sin(x*10) ? 0 : 1;
		break;
	case 1:
		return y > cos(x * 10) ? 0 : 1;
		break;
	case 2:
		return y > 2*x ? 0 : 1;
		break;
	case 3:
		return y > tan(x*10) ? 0 : 1;
		break;
	default:
		return y > cos(x*10) ? 0 : 1;
	}
}

Mat labelData(Mat points, int equation) {
	Mat labels(points.rows, 1, CV_32FC1);
	for(int i = 0; i < points.rows; i++) 
        {
			 float x = points.at<float>(i,0);
			 float y = points.at<float>(i,1);
			 labels.at<float>(i, 0) = f(x, y, equation);
	}
	return labels;
}

void plot_binary(Mat& data, Mat& classes, string name) {
	Mat plot(size, size, CV_8UC3);
	plot.setTo(Scalar(255.0,255.0,255.0));
	for(int i = 0; i < data.rows; i++) {

		float x = data.at<float>(i,0) * size;
		float y = data.at<float>(i,1) * size;

		if(classes.at<float>(i, 0) > .5) {
			circle(plot, Point(x,y), 2, CV_RGB(255,0,0),1);
		} else {
			circle(plot, Point(x,y), 2, CV_RGB(0,255,0),1);
		}
	}
	imshow(name, plot);
        waitKey(0);
}


#endif

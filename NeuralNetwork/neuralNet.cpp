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
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <functions.h>
#include "ml.h"

#define IM_WIDTH 75
#define IM_HEIGHT 75



using namespace std;
using namespace cv;

int numTrainingPoints=200;
int numTestPoints=2000;
//int size=200;
int eq=0;


int main(int argc, char **argv)
{

  
  timespec inicio, fin;
  double tiempo;
  ifstream in(argv[1]);
  string line;
  vector <string> vData;
  //nFeatures=
  int colsFile=0;
  while(getline(in, line))
    vData.push_back(line);//Guardamos en vData las rutas imágenes y otras características

  random_shuffle (vData.begin(), vData.end());//revolver elementos
  string firstLine=vData[0];
  
  for(int i=0; i<firstLine.length(); i++)//Determina el número de columnas
  {
    if(firstLine[i]==',')
      colsFile++;
  }
  colsFile++;
  int nFeatures=colsFile+(IM_WIDTH*IM_HEIGHT)-3-1;//Le quitamos lo de SX, SY, SZ y el path del archivo
  int numSamplesTest=40;
  int numSamplesTraining=vData.size()-numSamplesTest;
  //cout<<firstLine.length()<<endl<<firstLine<<endl<<colsFile<<endl<<nFeatures<<endl;
  int nClasses=4;//Número de clases a utilizar

  Mat trainingData(numSamplesTraining, nFeatures, CV_32FC1);
  Mat testData(numSamplesTest, nFeatures, CV_32FC1);
  Mat trainingClasses(trainingData.rows, nClasses, CV_32FC1);
  Mat testClasses(testData.rows, nClasses, CV_32FC1);

  bool flagFirst=true;
  int nSample2=0;
  for(int nSample=0; nSample<vData.size(); nSample++)
  {
    if(nSample<numSamplesTraining)//Los de entrenamiento
      setSample(vData[nSample], trainingData, trainingClasses, nSample, colsFile);//Coloca la muestra en la fila correspondiente
    else
    {
      setSample(vData[nSample], testData, testClasses, nSample2, colsFile);//Coloca la muestra en la fila correspondiente
      nSample2++;
    }
  }
  //cout<<trainingData<<endl;

  CvANN_MLP mlp;
  
  CvTermCriteria criteria;
  CvANN_MLP_TrainParams params;
  criteria.max_iter = 1000;
  criteria.epsilon = 0.00001f;
  criteria.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
  params.train_method = CvANN_MLP_TrainParams::BACKPROP;
  params.bp_dw_scale = 0.05f;
  params.bp_moment_scale = 0.05f;
  params.term_crit = criteria;

  //Topología de la red
  Mat layers= Mat(5,1, CV_32SC1);
  layers.row(0) = cv::Scalar(trainingData.cols);
  layers.row(1) = cv::Scalar(25);//Buenos resultados con 2 capas ocultas: 3-3, mejores con 20-15, 30-10,  25-16, 24-16=87.5, 
  layers.row(2) = cv::Scalar(20);
  layers.row(3) = cv::Scalar(25);
  layers.row(4) = cv::Scalar(trainingClasses.cols);
  mlp.create(layers);
  //cout<<"size trainingData = "<<trainingData.size()<<endl<<"testData = "<<testData.size()<<endl<<"cols = "<<trainingData.cols<<endl;
  cout<<"entrenando..."<<endl;
  mlp.train(trainingData, trainingClasses, Mat(), Mat(), params);
  cout<<"Entrenamiento completado"<<endl;
  ///*
  Mat response(1, trainingClasses.cols, CV_32FC1);
  Mat predicted(testClasses.rows, testClasses.cols, CV_32F);

  for(int i = 0; i < testData.rows; i++)
  {
        Mat response(1, testClasses.cols, CV_32FC1);
        Mat sample = testData.row(i);

        if(i==0)
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &inicio);
        mlp.predict(sample, response);
        if(i==0)
        {
          clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &fin);
               tiempo = (double)(fin.tv_sec - inicio.tv_sec) * 1.0e3    // segundos a milisegundos
                                      + (double)(fin.tv_nsec - inicio.tv_nsec)/1.0e6;      // nanosegundos a milisegundos

        }
        predicted.at<float>(i,0) = response.at<float>(0,0);
        predicted.at<float>(i,1) = response.at<float>(0,1);
        predicted.at<float>(i,2) = response.at<float>(0,2);
        predicted.at<float>(i,3) = response.at<float>(0,3);
        cout<<predicted.at<float>(i,0)<<", "<<predicted.at<float>(i,1)<<", "<<predicted.at<float>(i,2)<<", "<<predicted.at<float>(i,3)<<"---------"
        << testClasses.at<float>(i,0)<<", "<<testClasses.at<float>(i,1)<<", "<<testClasses.at<float>(i,2)<<", "<<testClasses.at<float>(i,3)<<endl;
        //cout<<predicted.at<float>(i,0)<<"----------\t"<<testClasses.at<float>(i,0)<<endl;

  }
  cout<<"desempeño = "<<evaluate(predicted, testClasses)<<endl;
  cout << endl << "tiempo empleado: " << tiempo << " milisegundos..." << endl << endl;
  //cout<<predicted<<endl;
  //*/
#if 0


  Mat trainingData(numTrainingPoints, 2, CV_32FC1);
  Mat testData(numTestPoints, 2, CV_32FC1);
  randu(trainingData,0,1);
  randu(testData,0,1);
  Mat trainingClasses = labelData(trainingData, eq);
  Mat testClasses = labelData(testData, eq);
  
  mlp.train(trainingData, trainingClasses, Mat(), Mat(), params);

  Mat response(1, 1, CV_32FC1);
  Mat predicted(testClasses.rows, 1, CV_32F);
  for(int i = 0; i < testData.rows; i++)
  {
          Mat response(1, 1, CV_32FC1);
          Mat sample = testData.row(i);

          mlp.predict(sample, response);
          predicted.at<float>(i,0) = response.at<float>(0,0);

  }
  cout<<predicted<<endl;

  cout << "Accuracy_{MLP} = " << evaluate(predicted, testClasses) << endl;
  plot_binary(testData, predicted, "Predictions Backpropagation");

  cout<<"hola"<<endl;



#endif
  return 0;
}




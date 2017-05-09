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
#define ROWS_GRID 1000//1000
#define COLS_GRID 1780//1560
#define CIRCLE_RAD 20
#define HD 0
#define FULL_HD 1
#define IM_WIDTH_VGA 640
#define IM_HEIGHT_VGA 480
#define IM_WIDTH_HD 1280
#define IM_HEIGHT_HD 720
//Dimensiones de la pantalla
#define W 1.095
#define L 0.615
#define CY 0.028
#define DCU 0.1

#define SX 6 
#define SY 7 
#define SZ 8 

using namespace std;
using namespace cv;
float dimXPix=W/COLS_GRID;
float dimYPix=L/ROWS_GRID;

char currentPath[100]= "/home/rajiv/Documentos/TesisMCCRajiv/Capture/";
void *getCoordinate(void *ptr);
void *getFrame(void *ptr);
pthread_mutex_t mutex1= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  condition_var = PTHREAD_COND_INITIALIZER;

bool flagExit=false;
bool flagChange=false;

int fd1;
char* buff;
int rd=0;
int nbytes,tries;
Mat frame;
VideoCapture capture;
void paintGrid(Mat &Grid, Point2f coordinate, int width, int height);
void setRes(VideoCapture& cap);

int main(int argc, char **argv)
{
  if(argc<3)
  {
    cout<<"Faltan parámetros!!"<<endl;
    exit(0);
  }

  int width, height;
  float rotPT=-30;//Rotación en grados de la unidad pan-tilt, negativa por regla de la mano derecha
  rotPT=rotPT*M_PI/180;
  

  width =COLS_GRID/100;
  height=ROWS_GRID/ 100;
  ifstream in(argv[1]);
  ofstream out(argv[2]);
  
  Point3f coordinate;
  char key; 
  pthread_t thread1;
  Mat Grid;
  Grid =Mat::zeros(ROWS_GRID,COLS_GRID, CV_8UC3);
  cout<<Grid.cols<<endl;
  cvNamedWindow("Grid",WINDOW_NORMAL);

    //*************Cámara**********
    cvNamedWindow("frame",WINDOW_NORMAL);//WINDOW_AUTOSIZE );
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
    setRes(capture);
    capture>>frame;

  const char *message1="Iniciando thread 1...";
  pthread_create(&thread1, NULL, &getFrame, (void *)message1);
  int contFrames=0;
  char fileNameTemp[100];
  char fileName[20]= "Imagen";
  char ext[]=".jpg";
  string line;
  Mat localFrame;

  //Obtiene las coordenadas
  while(getline(in, line))
  {
    getCoordinate(coordinate, line);
    paintGrid(Grid, coordinate, width, height);
    imshow("Grid", Grid);
    Grid =Mat::zeros(ROWS_GRID,COLS_GRID, CV_8UC3);
  }


  return 0;
}


void setRes(VideoCapture& cap)
{
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
}


void *getFrame(void *ptr)
{
  cout<<"Iniciando hilo de la captura"<<endl;
  
    cvNamedWindow("nuevo",WINDOW_NORMAL);//WINDOW_AUTOSIZE );

  for(;;)
  {
      pthread_mutex_lock( &mutex1 );      
      capture>>frame;
      flip(frame, frame, 1);
      imshow("nuevo", frame);
      pthread_cond_signal( &condition_var );
      pthread_mutex_unlock( &mutex1 );
      
    waitKey(10);
  }
}

void paintGrid(Mat &Grid, Point3f coordinate, int width, int height, float rotPT)
{
  rotPT-=rotPT;//La rotación ahora es en sentido contrario para devolver los puntos 
  Grid =Mat::zeros(ROWS_GRID,COLS_GRID, CV_8UC3);
  int xGrid, yGrid;
  //Se pasa del marco de referencia de la cámara (rotado) a la esquina superior de la pantalla
  Mat pS=(Mat_<float>(3,1)<<coordinate.x,
                            coordinate.y,
                            coordinate.z);
  //Se arma la la matriz de rotación
  Mat rotX=(Mat_<float>(3,3)<<1, 0, 0,
                          0, cos(rotPT), -sin(rotPT),
                          0, sin(rotPT), cos(rotPT));

  //Trasladamos el punto hacia el eje de rotación de la cámara
  pS.at<float>(1,0)-=DCU;
  //Rotamos el punto tantos grados que rote la unidad pt alrededor del eje X
  pS=rotX*pS;
  //Devolvemos el punto sobre el eje 
  pS.at<float>(1,0)+=DCU;
  cout<<"Puntos en la pantalla con la cámara como marco = "<<endl<<pS<<endl;

  //Se cambia el marco de referencia y se pasa a pixeles
  xGrid=(coordinate.x+W/2.0)/dimXPix;
  yGrid = -(coordinate.y+CY)/dimYPix;
  //cout<<"xGrid: "<<xGrid<<", yGrid: "<<yGrid<<endl;
  Mat roi=Grid(Rect(xGrid, yGrid, width, height));
  //roi.setTo(255);
  //circle(Grid, /*Point(20, 20)*/Point(xGrid+CIRCLE_RAD, yGrid+CIRCLE_RAD), CIRCLE_RAD, Scalar(180, 100, 140) ,10);
  circle(Grid, Point(xGrid+CIRCLE_RAD, yGrid+CIRCLE_RAD), CIRCLE_RAD, Scalar(rand()%256, rand()%256, rand()%256) ,10);
 // cout<<"coordenada "<<coordinate<<endl;

}


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
#include "Tools.h"
#include "Rot.h"

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
//#define W .48//Pantalla de 21 pulgadas
#define W 2.375//Pintarrón
//#define L .27//Pantalla
#define L 1.17
//#define CY 0.037
#define CY .2
#define DCU 0.06

#define F1X 0 
#define F1Y 1 
#define F1Z 2
#define F2X 3 
#define F2Y 4 
#define F2Z 5
#define SX 6 
#define SY 7 
#define SZ 8 

using namespace std;
using namespace cv;
float dimXPix=W/COLS_GRID;
float dimYPix=L/ROWS_GRID;

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
void paintGrid(Mat &Grid, Point3f coordinate, int width, int height, float rotP, Point2f& coordinate2d);
void setRes(VideoCapture& cap);
void getCoordinate(Point3f& coordinates, string line);
void getMarkF(P* myP1, P* myP2, Point3f& pHeight, string line);


char currentPath[100]= "/home/rajiv/Documentos/TesisMCCRajiv/Capture/pictures/expJun5Omar/";//Directorio para guardar las fotos
int main(int argc, char **argv)
{
  char fileName[20]= "Imagen_Omar";
  //***************muy importante**************
  float heightP=1.73;//Altura de la persona con respecto a los ojos!!!!!!!!!!!!!!!!!!!!!!!
  //***************muy importante**************
  
  if(argc<3)
  {
    cout<<"Faltan parámetros!!"<<endl;
    exit(0);
  }

  int width, height;
  float rotPT=-18.5294;//Rotación en grados de la unidad pan-tilt, negativa por regla de la mano derecha
  rotPT=rotPT*M_PI/180;
  cout<<"ángulo = "<<rotPT<<endl;  

  width =COLS_GRID/100;
  height=ROWS_GRID/ 100;
  ifstream in(argv[1]);
  ofstream out(argv[2]);
  //Variables para el cálculo de rotación de la mirada
  
  Point3f N=Point3d(-1.513929e-02, 9.542600e-01, 3.126682e-01), nNorm, pTemp2, pHeight;
  double d= 2.157112;
  float x, y, z, startXZ=0, norma;
  Mat headP;
  norma=norm(N);
  nNorm=Point3f(N.x/norma, N.y/norma, N.z/norma);


  pHeight=nNorm*-heightP;//Ahora solo hay que trasladarlo hacia la marca en el piso
  Point pTemp, hP;
  //***********

  Point3f coordinate;
  Point2f coordinate2d;
  char key; 
  pthread_t thread1;
  Mat Grid;
  Grid =Mat::zeros(ROWS_GRID,COLS_GRID, CV_8UC3);
  cout<<Grid.cols<<endl;
  cvNamedWindow("Grid",WINDOW_NORMAL);


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
    //imshow("tempWind", frame);
    //waitKey(0);

  const char *message1="Iniciando thread 1...";
  pthread_create(&thread1, NULL, &getFrame, (void *)message1);
  int contFrames=0;
  char fileNameTemp[100];
  char ext[]=".jpg";
  string line;
  Mat localFrame;
  int count=0;
  int posPiso=0;

  //Obtiene las coordenadas
  while(getline(in, line))
  {
  cout<<"debug..."<<endl;
    cout<<endl<<"\t\tNúmero de pos. en pantalla = "<<count<<endl;
    count++;
    getCoordinate(coordinate, line);
    paintGrid(Grid, coordinate, width, height, rotPT, coordinate2d);//Aqui se recupera también la pos 2d de la fig. en pantalla
    imshow("Grid", Grid);
    Grid =Mat::zeros(ROWS_GRID,COLS_GRID, CV_8UC3);
    for(int nPos=0; nPos<2; nPos++)//Dos capturas de rostro para la misma pos. en pantalla
    {
      //cout<<"Captura "<<nPos+1<<" para la pos. en pantalla: "<<coordinate2d<<endl;

      bool flagNextImage=false;
      cout<<"\t\tpos en el piso = "<<posPiso<<endl;

      while(!flagNextImage)
      {
        key=waitKey(10);
        if(key=='c') 
        {
          cout<<"Capturar"<<endl;
          pthread_mutex_lock( &mutex1);
          pthread_cond_wait( &condition_var, &mutex1 );
          frame.copyTo(localFrame);
          pthread_mutex_unlock( &mutex1 );
          imshow("frame", localFrame);
          stringstream numString;
          numString<<contFrames;
          Rotation *myRot;
          P* myP1;
          P* myP2;
          S* myS;
          //Calculo de la rotación de la mirada
          if(nPos==0)
          {
            myP1=new P();
            myP2=new P();
            myS=new S(coordinate);//Objeto en pantalla
            getMarkF(myP1, myP2, pHeight, line);//Obtiene la marca en el piso y la guarda en myP
            //cout<<"myP1->phix = "<<myP1->phix<<endl;
            myP1->computeA(myS);//Calcula el ángulo de mirada phix y phiy de P1 a S
            myP2->computeA(myS);
            /*cout<<"P1 phix = "<<myP1->phix<<", P1->Fx = "<<myP1->Fx<<", P1->Fy = "<<myP1->Fy<<", P1->Fz = "<<myP1->Fz<<", myP1->Px = "<<myP1->Px<<", myP1->Py = "<<myP1->Py<<", myP1->Pz = "<<myP1->Pz<<endl;
            cout<<"P2 phix = "<<myP2->phix<<", P2->Fx = "<<myP2->Fx<<", P2->Fy = "<<myP2->Fy<<", P2->Fz = "<<myP2->Fz<<", myP2->Px = "<<myP2->Px<<", myP2->Py = "<<myP2->Py<<", myP2->Pz = "<<myP2->Pz<<endl;
            cout<<"myS->Sx = "<<myS->Sx<<", myS->Sy = "<<myS->Sy<<", myS->Sz = "<<myS->Sz<<endl;*/
            myRot=new Rotation(myP1, myP2);//Calcula la rotación que hay entre P1 y P2
            //cout<<"thetaX = "<<myRot->thetaX<<", thetaY = "<<myRot->thetaY<<endl;
          }
          snprintf(fileNameTemp, 99,"%s%s%06d%s",currentPath,fileName,contFrames, ext);
          cout<<"Nombre: "<<fileNameTemp<<endl;
          //imwrite(fileNameTemp, localFrame);
          //cout<<"Cols: "<<localFrame.cols<<", Rows: "<<localFrame.rows<<endl;
          if(nPos==0)
          {
            //out<<fileNameTemp<<", "<<myRot->thetaX<<", "<<myRot->thetaY<<", "<<myP1->phix<<", "<<myP1->phiy<<", "<<myS->Sx<<", "<<myS->Sy<<", "<<myS->Sz<<endl;
            out<<fileNameTemp<<", "<<myP1->Px<<", "<<myP1->Py<<", "<<myP1->Pz<<", "<<myS->Sx<<", "<<myS->Sy<<", "<<myS->Sz<<endl;
            posPiso++;
          }
          else
          {
            //out<<fileNameTemp<<", "<<myRot->thetaX<<", "<<myRot->thetaY<<", "<<myP2->phix<<", "<<myP2->phiy<<", "<<myS->Sx<<", "<<myS->Sy<<", "<<myS->Sz<<endl;
            out<<fileNameTemp<<", "<<myP2->Px<<", "<<myP2->Py<<", "<<myP2->Pz<<", "<<myS->Sx<<", "<<myS->Sy<<", "<<myS->Sz<<endl;
          }

          cout<<"Guardado!!"<<endl;
          //out
          contFrames++;
          flagNextImage=true;

        }//if(key=='c')
      }//while(flagNextImage)
    }//for(nPos<2)
  }//while(getline(line))


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
      imshow("nuevo", frame);
      pthread_cond_signal( &condition_var );
      pthread_mutex_unlock( &mutex1 );
      
      waitKey(10);
  }
  cout<<"finalizando hilo de la captura"<<endl;
}

void paintGrid(Mat &Grid, Point3f coordinate, int width, int height, float rotPT, Point2f& coordinate2d)
{
  rotPT=-rotPT;//La rotación ahora es en sentido contrario para devolver los puntos 
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

  //cout<<"rotX = "<<endl<<rotX<<endl;
  //Trasladamos el punto hacia el eje de rotación de la cámara
  pS.at<float>(1,0)-=DCU;
  //Rotamos el punto tantos grados que rote la unidad pt alrededor del eje X
  pS=rotX*pS;
  //Devolvemos el punto sobre el e 
  pS.at<float>(1,0)+=DCU;
  //cout<<"Puntos en la pantalla con la cámara como marco = "<<endl<<pS<<endl;
  coordinate2d=Point2f(pS.at<float>(0,0), pS.at<float>(1,0));
#if 1
  //Se cambia el marco de referencia y se pasa a pixeles
  //cout<<"***Marco de referencia esquina superior izquierda de la pantalla***"<<endl;
  //cout<<"x = "<<W/2.0-coordinate2d.x<<", y = "<<coordinate2d.y-(CY+DCU)<<endl;
  xGrid=(W/2.0-coordinate2d.x)/dimXPix;//Se pasa de m a pix dividiendo entre dimXPix 
  yGrid = (coordinate2d.y-(CY+DCU))/dimYPix;
  //cout<<"xGrid: "<<xGrid<<", yGrid: "<<yGrid<<endl;
  //Para el pintarrón no es necesario (y no se puede) visualizar 
  //Mat roi=Grid(Rect(xGrid, yGrid, width, height));



  //roi.setTo(255);
  //circle(Grid, /*Point(20, 20)*/Point(xGrid+CIRCLE_RAD, yGrid+CIRCLE_RAD), CIRCLE_RAD, Scalar(180, 100, 140) ,10);

  //Para el pintarrón no es necesario (y no se puede) visualizar 
  //circle(Grid, Point(xGrid+CIRCLE_RAD, yGrid+CIRCLE_RAD), CIRCLE_RAD, Scalar(rand()%256, rand()%256, rand()%256) ,10);
#endif

}

void getCoordinate(Point3f& coordinates, string line)
{
  int pos=0, pos_ant=0;
  string SxTemp, SyTemp, SzTemp;
  for(int i=0; i<9; i++)
  {
    pos=line.find(',', pos_ant);
    if(i==SX)
      SxTemp=line.substr(pos_ant, pos-pos_ant).c_str();
    if(i==SY)
      SyTemp=line.substr(pos_ant, pos-pos_ant).c_str();
    if(i==SZ)
      SzTemp=line.substr(pos_ant, pos-pos_ant).c_str();
    pos_ant=pos+1;
  }
  coordinates.x=atof(SxTemp.c_str());
  coordinates.y=atof(SyTemp.c_str());
  coordinates.z=atof(SzTemp.c_str());
  //coordinates.x=-W/2;
  //coordinates.y=CY;
  cout<<"Sx = "<<coordinates.x<<", Sy = "<<coordinates.y<<", Sz = "<<coordinates.z<<endl;
}

void getMarkF(P* myP1, P* myP2, Point3f& pHeight, string line)
{
  int pos=0, pos_ant=0;
  for(int i=0; i<8; i++)
  {
    pos=line.find(',', pos_ant);
    if(i==F1X)
      myP1->Fx=atof(line.substr(pos_ant, pos-pos_ant).c_str());
    if(i==F1Y)
      myP1->Fy=atof(line.substr(pos_ant, pos-pos_ant).c_str());
    if(i==F1Z)
      myP1->Fz=atof(line.substr(pos_ant, pos-pos_ant).c_str());
    pos=line.find(',', pos_ant);
    if(i==F2X)
      myP2->Fx=atof(line.substr(pos_ant, pos-pos_ant).c_str());
    if(i==F2Y)
      myP2->Fy=atof(line.substr(pos_ant, pos-pos_ant).c_str());
    if(i==F2Z)
      myP2->Fz=atof(line.substr(pos_ant, pos-pos_ant).c_str());
    pos_ant=pos+1;
  }//Ya tenemos las posiciones en el piso
  //Calculamos la posición 3d con respecto a la cabeza
  myP1->Px=myP1->Fx+pHeight.x;//Hay que tomar en cuenta la pos en el suelo
  myP1->Py=myP1->Fy+pHeight.y;
  myP1->Pz=myP1->Fz+pHeight.z;
  myP2->Px=myP2->Fx+pHeight.x;
  myP2->Py=myP2->Fy+pHeight.y;
  myP2->Pz=myP2->Fz+pHeight.z;
  //cout<<"myP1->Px = "<<myP1->Px<<", myP2->Px = "<<myP2->Px<<endl;

  
}

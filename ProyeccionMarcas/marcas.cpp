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

    //setCam(capture);//Se alinea la cámara con el tablero
    Mat K, scnPts, imgPts, RT, dummy, testPoints, I, Corners, Icol;
    vector<Point2d>P;
    /*capture>>Icol;//Se captura el tablero para encontrar las intersecciones
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
    }*/
    
    //Se crea la matriz con los puntos detectados del tablero automaticamente
    /*imgPts=(Mat_<double>(3,1)<<P[0].x,
                                P[0].y,
                                1);
    for(int i=1; i<Corners.rows; i++)
    {
      Mat temp=(Mat_<double>(3,1)<<P[i].x,
                                  P[i].y,
                                  1);
      hconcat(imgPts,temp,imgPts);
    }*/
    //Para versión offline estableciendo los ptos
    /*scnPts=(Mat_<double>(4,12)<< 0, 0, 0.4, 0.4, 0.4, 0.4, -0.4, -0.4, -0.4, -0.4, 0, 0,//Ahora se parte de la mitad
                                  0.8, 1.2, 0, 0.4, 0.8, 1.2, 0, 0.4, 0.8, 1.2, 0, 0.4,
                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);

    imgPts=(Mat_<double>(3,12)<< 957, 959, 1099, 1114, 1134, 1160, 815, 800, 780, 754, 957, 959,
                                 979, 1057, 881, 928, 982, 1058, 880, 924, 979, 1054, 878, 927,
                                1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);*/
    imgPts=(Mat_<double>(3,48)<< 815.56787109375, 886.8239135742188, 958.4537963867188, 1029.955322265625, 1101.938354492188, 1174.005615234375, 808.3628540039062, 883.7537841796875, 958.714111328125, 1034.350463867188, 1109.638305664062, 1185.86376953125, 800.7138061523438, 879.9265747070312, 959.5117797851562, 1038.83056640625, 1118.88916015625, 1198.915893554688, 791.6714477539062, 875.909912109375, 959.8040771484375, 1044.343017578125, 1128.69677734375, 1214.092529296875, 837.8741455078125, 897.2778930664062, 956.9844360351562, 1016.42333984375, 1076.3017578125, 1136.21044921875, 832.9345092773438, 895.29541015625, 957.1570434570312, 1019.464172363281, 1081.859375, 1144.757080078125, 827.8035278320312, 892.4791870117188, 957.6625366210938, 1022.542297363281, 1087.906127929688, 1153.239379882812, 821.8426513671875, 890.1197509765625, 957.7943115234375, 1026.245727539062, 1094.3779296875, 1163.309936523438, //Primera fila
        789.2471313476562, 786.6021728515625, 783.5311279296875, 781.093505859375, 778.31787109375, 776.1768798828125, 824.7357177734375, 821.5552368164062, 818.5955810546875, 816.066162109375, 813.6526489257812, 811.060791015625, 864.3524169921875, 861.1209106445312, 858.2491455078125, 855.4376220703125, 853.0947265625, 850.376708984375, 908.6774291992188, 905.4671020507812, 902.0062255859375, 899.7337036132812, 896.8604736328125, 894.7748413085938, 679.0939331054688, 676.92724609375, 674.5238647460938, 672.1847534179688, 670.0202026367188, 667.4046630859375, 703.5702514648438, 701.4966430664062, 699.173828125, 696.7796020507812, 694.4873657226562, 692.2327270507812, 729.8762817382812, 726.9873046875, 724.7837524414062, 722.2488403320312, 719.9083862304688, 717.5863647460938, 758.3207397460938, 754.8271484375, 752.7904052734375, 750.1862182617188, 747.7540283203125, 745.3524169921875,//segunda fila
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);


    scnPts=(Mat_<double>(4,48)<<  0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131, 0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131, 0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131, 0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131,   0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131, 0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131, 0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131, 0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131,//Primera fila
0.4799999892711639, 0.4799999892711639, 0.4799999892711639, 0.4799999892711639, 0.4799999892711639, 0.4799999892711639, 0.5999999642372131, 0.5999999642372131, 0.5999999642372131, 0.5999999642372131, 0.5999999642372131, 0.5999999642372131, 0.7199999690055847, 0.7199999690055847, 0.7199999690055847, 0.7199999690055847, 0.7199999690055847, 0.7199999690055847, 0.8399999737739563, 0.8399999737739563, 0.8399999737739563, 0.8399999737739563, 0.8399999737739563, 0.8399999737739563,  0, 0, 0, 0, 0, 0, 0.119999997317791, 0.119999997317791, 0.119999997317791, 0.119999997317791, 0.119999997317791, 0.119999997317791, 0.239999994635582, 0.239999994635582, 0.239999994635582, 0.239999994635582, 0.239999994635582, 0.239999994635582, 0.3599999845027924, 0.3599999845027924, 0.3599999845027924, 0.3599999845027924, 0.3599999845027924, 0.3599999845027924, //Segunda fila
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,//Tercera fila
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);

    cout<<"imgPts = "<<imgPts<<endl;
    /*float startX3d, startY3d;
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
    }*/
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
#if 0
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
#endif

  return 0;
}

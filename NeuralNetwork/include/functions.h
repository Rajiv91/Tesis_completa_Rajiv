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

using namespace std;
using namespace cv;

void setRes(VideoCapture& capture)
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

void findCorners(Mat& mChessboard, int filsGrid, int colsGrid, vector<Point2d>& P)
{
  Mat I, Icol, Corners; 
  int idxP=0;

    //capture>>Icol;//Se captura el tablero para encontrar las intersecciones
    //Icol=imread("chessboard.jpg",1);
    //Icol=imread("/home/rajiv/Documentos/seminario3/chessboard720/chessboard3_720.jpg", 1);
    Icol=mChessboard.clone();
    //Icol=imread("/home/rajiv/Documentos/seminario3/picturesCalib/L_Clbr_000004.jpg",1);

      //remap (Icol, Icol, Mx, My, INTER_LINEAR, BORDER_TRANSPARENT);

   cvtColor(Icol, I, CV_RGB2GRAY);
    cvNamedWindow("chessboard",WINDOW_NORMAL );
    if(findChessboardCorners(I, Size(colsGrid, filsGrid), Corners))
    {
            cornerSubPix(I, Corners, Size(10,10) , Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            for (int j = 0; j < Corners.rows; ++j)
            {
                Vec2f *apuv = Corners.ptr<Vec2f>(j);
                circle(Icol, Point((*apuv)[0], (*apuv)[1]) , 5,  Scalar_ <uchar> (255,64,255), 3);
                //P.push_back(Point2d((*apuv)[0], (*apuv)[1]));
                P[idxP]=Point2d((*apuv)[0], (*apuv)[1]);
                stringstream textTemp;
                textTemp<<j;
                putText(Icol,textTemp.str(),  Point((*apuv)[0], (*apuv)[1]),FONT_HERSHEY_COMPLEX_SMALL,1,CV_RGB(0, 255, 0),2 );
                idxP++;
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

}

void setImgP(vector<Point2d>& P, Mat& imgPts)
{
  imgPts=(Mat_<double>(3,1)<<P[0].x,
                                P[0].y,
                                1);
    for(int i=1; i<P.size(); i++)
    {
      Mat temp=(Mat_<double>(3,1)<<P[i].x,
                                  P[i].y,
                                  1);
      hconcat(imgPts,temp,imgPts);
    }
}

void buildScnP(int filsGrid, int colsGrid, Mat& scnPts)
{
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
    int origen= 0;
    Mat mPoint=(Mat_<double>(4,1)<<scnPts.at<double>(0,origen),
                                  scnPts.at<double>(1,origen),
                                  scnPts.at<double>(2,origen), 
                                  scnPts.at<double>(3,origen));
    //scnPts-=scnPts(Rect(23, 0, 1, 4));
    scnPts-=mPoint;
    //cout<<"subscnPts = "<<scnPts(Rect(23, 0, 1, 4))<<endl;
    //cout<<"scnPts movido = "<<scnPts<<endl;

}


//Para la versión offline estos puntos se establecen:
#if 0

    scnPts=(Mat_<double>(4,12)<< 0, 0, 0.4, 0.4, 0.4, 0.4, -0.4, -0.4, -0.4, -0.4, 0, 0,//Ahora se parte de la mitad
                                  0.8, 1.2, 0, 0.4, 0.8, 1.2, 0, 0.4, 0.8, 1.2, 0, 0.4,
                                  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);

    imgPts=(Mat_<double>(3,12)<< 957, 959, 1099, 1114, 1134, 1160, 815, 800, 780, 754, 957, 959,
                                 979, 1057, 881, 928, 982, 1058, 880, 924, 979, 1054, 878, 927,
                                1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);
    /*imgPts=(Mat_<double>(3,48)<< 815.56787109375, 886.8239135742188, 958.4537963867188, 1029.955322265625, 1101.938354492188, 1174.005615234375, 808.3628540039062, 883.7537841796875, 958.714111328125, 1034.350463867188, 1109.638305664062, 1185.86376953125, 800.7138061523438, 879.9265747070312, 959.5117797851562, 1038.83056640625, 1118.88916015625, 1198.915893554688, 791.6714477539062, 875.909912109375, 959.8040771484375, 1044.343017578125, 1128.69677734375, 1214.092529296875, 837.8741455078125, 897.2778930664062, 956.9844360351562, 1016.42333984375, 1076.3017578125, 1136.21044921875, 832.9345092773438, 895.29541015625, 957.1570434570312, 1019.464172363281, 1081.859375, 1144.757080078125, 827.8035278320312, 892.4791870117188, 957.6625366210938, 1022.542297363281, 1087.906127929688, 1153.239379882812, 821.8426513671875, 890.1197509765625, 957.7943115234375, 1026.245727539062, 1094.3779296875, 1163.309936523438, //Primera fila
        789.2471313476562, 786.6021728515625, 783.5311279296875, 781.093505859375, 778.31787109375, 776.1768798828125, 824.7357177734375, 821.5552368164062, 818.5955810546875, 816.066162109375, 813.6526489257812, 811.060791015625, 864.3524169921875, 861.1209106445312, 858.2491455078125, 855.4376220703125, 853.0947265625, 850.376708984375, 908.6774291992188, 905.4671020507812, 902.0062255859375, 899.7337036132812, 896.8604736328125, 894.7748413085938, 679.0939331054688, 676.92724609375, 674.5238647460938, 672.1847534179688, 670.0202026367188, 667.4046630859375, 703.5702514648438, 701.4966430664062, 699.173828125, 696.7796020507812, 694.4873657226562, 692.2327270507812, 729.8762817382812, 726.9873046875, 724.7837524414062, 722.2488403320312, 719.9083862304688, 717.5863647460938, 758.3207397460938, 754.8271484375, 752.7904052734375, 750.1862182617188, 747.7540283203125, 745.3524169921875,//segunda fila
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);


    scnPts=(Mat_<double>(4,48)<<  0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131, 0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131, 0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131, 0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131,   0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131, 0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131, 0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131, 0, 0.119999997317791, 0.239999994635582, 0.3599999845027924, 0.4799999892711639, 0.5999999642372131,//Primera fila
0.4799999892711639, 0.4799999892711639, 0.4799999892711639, 0.4799999892711639, 0.4799999892711639, 0.4799999892711639, 0.5999999642372131, 0.5999999642372131, 0.5999999642372131, 0.5999999642372131, 0.5999999642372131, 0.5999999642372131, 0.7199999690055847, 0.7199999690055847, 0.7199999690055847, 0.7199999690055847, 0.7199999690055847, 0.7199999690055847, 0.8399999737739563, 0.8399999737739563, 0.8399999737739563, 0.8399999737739563, 0.8399999737739563, 0.8399999737739563,  0, 0, 0, 0, 0, 0, 0.119999997317791, 0.119999997317791, 0.119999997317791, 0.119999997317791, 0.119999997317791, 0.119999997317791, 0.239999994635582, 0.239999994635582, 0.239999994635582, 0.239999994635582, 0.239999994635582, 0.239999994635582, 0.3599999845027924, 0.3599999845027924, 0.3599999845027924, 0.3599999845027924, 0.3599999845027924, 0.3599999845027924, //Segunda fila
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,//Tercera fila
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1);*/

#endif

#endif

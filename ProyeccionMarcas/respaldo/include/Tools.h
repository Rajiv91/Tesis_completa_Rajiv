#ifndef __TOOLS__
#define __TOOLS__

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <cstdlib>
#include <cstdio>
//#include <Tools2.h>
#define GRAM 0

using namespace std;
using namespace cv;

struct ParamsK
{
  float fx;
  float fy;
  float gamma;
  float cx;
  float cy;
  ParamsK(float fx2, float fy2, float gamma2, float cx2, float cy2)
  {
    fx=fx2;
    fy=fy2;
    gamma=gamma2;
    cx=cx2;
    cy=cy2;
  }
};

//http://people.rennes.inria.fr/Eric.Marchand/pose-estimation/tutorial-homography-opencv.html
Mat homography_dlt(vector< Point2d > &x1, vector< Point2d > &x2)
{
  int npoints = (int)x1.size();
  cv::Mat A(2*npoints, 9, CV_64F, cv::Scalar(0));
  // We need here to compute the SVD on a (n*2)*9 matrix (where n is
  // the number of points). if n == 4, the matrix has more columns
  // than rows. The solution is to add an extra line with zeros
  if (npoints == 4)
    A.resize(2*npoints+1, cv::Scalar(0));
  // Since the third line of matrix A is a linear combination of the first and second lines
  // (A is rank 2) we don't need to implement this third line
  for(int i = 0; i < npoints; i++) {              // Update matrix A using eq. 33
    A.at<double>(2*i,3) = -x1[i].x;               // -xi_1
    A.at<double>(2*i,4) = -x1[i].y;               // -yi_1
    A.at<double>(2*i,5) = -1;                     // -1
    A.at<double>(2*i,6) =  x2[i].y * x1[i].x;     //  yi_2 * xi_1
    A.at<double>(2*i,7) =  x2[i].y * x1[i].y;     //  yi_2 * yi_1
    A.at<double>(2*i,8) =  x2[i].y;               //  yi_2
    A.at<double>(2*i+1,0) =  x1[i].x;             //  xi_1
    A.at<double>(2*i+1,1) =  x1[i].y;             //  yi_1
    A.at<double>(2*i+1,2) =  1;                   //  1
    A.at<double>(2*i+1,6) = -x2[i].x * x1[i].x;   // -xi_2 * xi_1
    A.at<double>(2*i+1,7) = -x2[i].x * x1[i].y;   // -xi_2 * yi_1
    A.at<double>(2*i+1,8) = -x2[i].x;             // -xi_2
  }
  // Add an extra line with zero.
  if (npoints == 4) {
    for (int i=0; i < 9; i ++) {
      A.at<double>(2*npoints,i) = 0;
    }
  }
  cv::Mat w, u, vt;
  cv::SVD::compute(A, w, u, vt);
  double smallestSv = w.at<double>(0, 0);
  unsigned int indexSmallestSv = 0 ;
  for (int i = 1; i < w.rows; i++) {
    if ((w.at<double>(i, 0) < smallestSv) ) {
      smallestSv = w.at<double>(i, 0);
      indexSmallestSv = i;
    }
  }
  cv::Mat h = vt.row(indexSmallestSv);
  if (h.at<double>(0, 8) < 0) // tz < 0
    h *=-1;
  cv::Mat _2H1(3, 3, CV_64F);
  for (int i = 0 ; i < 3 ; i++)
    for (int j = 0 ; j < 3 ; j++)
      _2H1.at<double>(i,j) = h.at<double>(0, 3*i+j);
  return _2H1;
}


void getPlaneOrientation2 (Mat &scenePts, Mat &imagePts, Point3f &N, double &D, Mat& Rnew, Mat& Tnew)
{
    Mat H, R, R1, R2, R3, T, Nm, scPt;
    double n;
    //Point2f scP[9], imP[9];
    vector<Point2d>scP;
    vector<Point2d> imP;
    int i;

    assert (scenePts.rows == 4 && scenePts.cols >= 4 && imagePts.rows == 3 && imagePts.cols >= 4);
    //cout<<"cols y rows de scene: "<<scenePts.cols<<", "<<scenePts.rows<<endl
     // <<"cols y rows de imagePts: "<<imagePts.cols<<", "<<imagePts.rows<<endl;

    //Calculamos la homografia
    for (i=0;i<scenePts.cols;++i)
    {
        scP.push_back(Point2f(scenePts.at<double>(0,i), scenePts.at<double>(1,i)));
        imP.push_back(Point2f(imagePts.at<double>(0,i), imagePts.at<double>(1,i)));
    }
    H = findHomography(scP, imP, CV_LMEDS );
    //H=homography_dlt(scP, imP);
    cout<<"H="<<H<<endl;

    //Se entiende que la H se puede descomponer como H=[R1,R2, T)
    //Donde R1, y R2, son las dos primeras columnas de la matriz de rotacion
    //y T es el vector de Translación.
    //Primero normalizamos la H, a sabiendas que la norma de R1 debe ser 1.
    
#if GRAM
    R1=H(Rect(0,0,1,3));
    n = sqrt(R1.dot(R1));
    assert (n != 0.0);
    H /= n;
    R2=H(Rect(1,0,1,3));
    R3=H(Rect(2,0,1,3));
    R3.copyTo(T);

    //En dado caso que R1 y R2 no sean ortogonales, aplicamos la
    //ortogonalizacion de Grahm-Schmidt,
    R2 = R2 - R2.dot (R1) * R1;
    n = R2.dot(R2);
    assert (n != 0.0);
    R2 /= sqrt(n);
    R3 = R1.cross(R2);
    hconcat (R1, R2, R2);
    hconcat (R2, R3, R);
    R.copyTo(Rnew);
    T.copyTo(Tnew);
    cout<<"GRAM"<<endl;
#else 
  //G obtenida de: http://dsp.stackexchange.com/questions/1484/how-to-compute-camera-pose-from-homography-matrix
    Mat pose = Mat::eye(3, 4, CV_64FC1); //3x4 matrix
    float norm1 = (float)norm(H.col(0)); 
    float norm2 = (float)norm(H.col(1));
    float tnorm = (norm1 + norm2) / 2.0f;

    Mat v1 = H.col(0);
    Mat v2 = pose.col(0);

    cv::normalize(v1, v2); // Normalize the rotation

    v1 = H.col(1);
    v2 = pose.col(1);

    cv::normalize(v1, v2);

    v1 = pose.col(0);
    v2 = pose.col(1);

    Mat v3 = v1.cross(v2);  //Computes the cross-product of v1 and v2
    Mat c2 = pose.col(2);
    v3.copyTo(c2);      

    pose.col(3) = H.col(2) / tnorm; //vector t [R|t]
    Mat RTemp;
    R1=pose(Rect(0,0,1,3));
    R2=pose(Rect(1,0,1,3));
    R3=pose(Rect(2,0,1,3));
    T=pose(Rect(3,0,1,3));
    hconcat (R1, R2, R2);
    hconcat (R2, R3, R);

    R.copyTo(Rnew);
    T.copyTo(Tnew);
#endif
    //La normal al plano está dada por el eje Z (i.e. [0,0,1]^T) en
    //el marco de referencia del plano, transladado al marco de referencia
    //de la cámara. 
    Nm = R * (Mat_<double>(3,1) << 0., 0., 1.);

    //Transladamos una coordenada en el plano al marco de referencia de
    //la cámara y ...
    scPt = R * scenePts(Rect(0,0,1,3)) + T;
    //Calculamos el parametro D de la ecuacion del plano como:
    D = Nm.dot(scPt);

    N.x = Nm.at<double>(0,0);
    N.y = Nm.at<double>(1,0);
    N.z = Nm.at<double>(2,0);
}

void getPlaneOrientation (Mat &scenePts, Mat &imagePts, Point3f &N, double &D)
{
    Mat H, R, R1, R2, R3, T, Nm, scPt;
    double n;
    Point2f scP[4], imP[4];
    int i;

    assert (scenePts.rows == 4 && scenePts.cols == 4 && imagePts.rows == 3 && imagePts.cols >= 4);

    //Calculamos la homografia
    for (i=0;i<4;++i)
    {
        scP[i] = Point2f(scenePts.at<double>(0,i), scenePts.at<double>(1,i));
        imP[i] = Point2f(imagePts.at<double>(0,i), imagePts.at<double>(1,i));
    }
    H = getPerspectiveTransform (scP, imP);

    //Se entiende que la H se puede descomponer como H=[R1,R2, T)
    //Donde R1, y R2, son las dos primeras columnas de la matriz de rotacion
    //y T es el vector de Translación.
    //Primero normalizamos la H, a sabiendas que la norma de R1 debe ser 1.
    R1=H(Rect(0,0,1,3));
    n = sqrt(R1.dot(R1));
    assert (n != 0.0);
    H /= n;
    R2=H(Rect(1,0,1,3));
    R3=H(Rect(2,0,1,3));
    R3.copyTo(T);

    //En dado caso que R1 y R2 no sean ortogonales, aplicamos la
    //ortogonalizacion de Grahm-Schmidt,
    R2 = R2 - R2.dot (R1) * R1;
    n = R2.dot(R2);
    assert (n != 0.0);
    R2 /= sqrt(n);
    R3 = R1.cross(R2);
    hconcat (R1, R2, R2);
    hconcat (R2, R3, R);
    

    //La normal al plano está dada por el eje Z (i.e. [0,0,1]^T) en
    //el marco de referencia del plano, transladado al marco de referencia
    //de la cámara. 
    Nm = R * (Mat_<double>(3,1) << 0., 0., 1.);

    //Transladamos una coordenada en el plano al marco de referencia de
    //la cámara y ...
    scPt = R * scenePts(Rect(0,0,1,3)) + T;
    //Calculamos el parametro D de la ecuacion del plano como:
    D = Nm.dot(scPt);

    N.x = Nm.at<double>(0,0);
    N.y = Nm.at<double>(1,0);
    N.z = Nm.at<double>(2,0);
}


struct PosP//Las posiciones de las P en el área con respecto a un marco de referencia
{
  float startX;
  float skipX;
  float startZ;
  float skipZ;
  PosP(float startX2, float skipX2, float startZ2, float skipZ2) {
  startX=startX2;
  skipX=skipX2;
  startZ=startZ2;
  skipZ=skipZ2;
    
  }
};


struct mouseData
{
    /*!
    \var vector <Point> *p;
    \brief Una referencia a un vector (STL) de objetos tipo CV::Point en donde se almacenaran cada una de las coordenadas indicada por el usuario 
    */
    vector <Point> *p;

    /*!
    \var bool Out
    \brief Una variable de tipo booleanp que almacena el estado de captura del programa. Si Out es falso, entonces la captura debe continuar. Cuando Out es  verdadero, la captura termina.
     */
    bool Out;

    /*!
    \fn mouseData(vector <Point> *vP)
    \brief Constructor de la clase. inicializa los miembros de la case como sigue: asigna un valor al apuntador al vector de puntos, e inicializa el atributo Out como falso.
    \param vector <Point> *vP  Un apuntador al vector en donde se van a almacenar los datos 
    */
    mouseData(vector <Point> *vP)
    {
        p =vP;
        Out = false;
    }
};

/*!
  \fn void on_mouseEvent(int event, int x, int y, int flags, void *p)
  \brief esta funcion en invocada cuando ocurre un evento en le dipositivo apuntador. por ejemplo cuando el usuario mueve el raton, oprime uno de los botones o gira la rueda. Esta función es definida por el usuario de acuerdo a susu necesidades. En el caso que se ilustra en este programa, cada que se invoque esta función se van a imprimir los paramtros de la función en la pantalla; cuando se invoque por que el usuario oprimió el boton izquierdo del ratón, se almacenran las coordendas del mismo en el vector contenido en el objeto tipo mouseData referenciado por p, y i se oprime el boton derecho del ratón, la bandera Out del objeto tipo mouseData referenciado por p se se le asignará el valor verdadero.
 \param int event Un entero que indica que tipo de evento ocurrió.
 \param int x, y Estas son las coordenadas en la imagen que tenía el dispositivo apuntador cuando se invocó la interrupción.
 \param int flags Estas son las banderas asociadas el evento.
 \param void *p Un apuntador a un objeto que puede ser utilizado por la función. En éste caso p debe ser un apuntador a un objeto de tipo mouseData, en donde se van a almacenar los datos capturados.

*/
void on_mouseEvent(int event, int x, int y, int flags, void *p)
{
    mouseData *mD = (mouseData *)p;

        mD->Out = false;
    switch (event)
    {
        case CV_EVENT_MOUSEMOVE:
        cout << "CV_EVENT_MOUSEMOVE: [" << x << ", " << y << " | " << flags << "]" << endl;
        break;
        case CV_EVENT_LBUTTONDOWN:
        cout << "CV_EVENT_LBUTTONDOWN: [" << x << ", " << y << " | " << flags << "]" << endl;
        {
            Point tmp(x,y);
            (mD->p)->push_back(tmp);
        }    

        break;
        case CV_EVENT_RBUTTONDOWN:
        cout << "CV_EVENT_RBUTTONDOWN: [" << x << ", " << y << " | " << flags << "]" << endl;
        mD->Out = true;
        break;
        case CV_EVENT_MBUTTONDOWN:
        cout << "CV_EVENT_MBUTTONDOWN: [" << x << ", " << y << " | " << flags << "]" << endl;
        break;
        case CV_EVENT_LBUTTONUP:
        cout << "CV_EVENT_LBUTTONUP: [" << x << ", " << y << " | " << flags << "]" << endl;
        break;
        case CV_EVENT_RBUTTONUP:
        cout << "CV_EVENT_RBUTTONUP: [" << x << ", " << y << " | " << flags << "]" << endl;
        break;
        case CV_EVENT_MBUTTONUP:
        cout << "CV_EVENT_MBUTTONUP: [" << x << ", " << y << " | " << flags << "]" << endl;
        break;
        case CV_EVENT_LBUTTONDBLCLK:
        cout << "CV_EVENT_LBUTTONDBLCLK: [" << x << ", " << y << " | " << flags << "]" << endl;
        break;
        case CV_EVENT_RBUTTONDBLCLK:
        cout << "CV_EVENT_RBUTTONDBLCLK: [" << x << ", " << y << " | " << flags << "]" << endl;
        break;
        case CV_EVENT_MBUTTONDBLCLK:
        cout << "CV_EVENT_MBUTTONDBLCLK: [" << x << ", " << y << " | " << flags << "]" << endl;
        break;
    }
}

/*!
\fn void dibujaPoints(Mat &frame, vector <Point> &P, Scalar color=Scalar(0,0,255))
\brief Esta función dibuja en la imagen almacenada en la mtriz frame, circulos de radio 5 en las coordenadas de cada uno de los puntos almacenados en el vector P. el color del circulo es rojo por defecto, pero puede ser modificado por el usurio usando el parametro colo de la función.
\param Mat &frame  Es la matriz en donde se van a dibujar los círculos.
\param vector <Point> &* Este el el vector de puntos en donde estan almacenadas las coordenadas de los puntos a dibujar.
\param Scalar color Esté parámtero almacena el color con el que se van a dibujar los circulos; el valor por defecto corresponde al color rojo.
*/
void dibujaPoints(Mat &frame, vector <Point> &P, Scalar color=Scalar(0,0,255))
{
    vector <Point>::iterator ini, end;

    ini = P.begin();
    end = P.end();
    for (;ini != end; ++ini)
        circle(frame, *ini, 5, color);
}


/*!
\fn void capturaPuntos(const char *Name, Mat &frame, vector<Point> *P)
\brief Esta función muestra una imagen en una ventana, registra la funcion on_mouseEvent (definida previamente) como el manejador de eventos del raton de dicha ventana, e inicia un ciclo, controlado por el atributo Out del objeto md (tipo mouseData). En el interior del ciclo se dibujan los circulos capturado en la imagen, se muestra la imagen y se espera que termine el programa de manera natural, cuando la función on_mouseEvent cambia el atributo Out  a verdadero o cuando el usuario oprime una tecla durante la captura.
\param const char *Name El nombre con el que se va a hacer referencia a la ventana en donde vamos a desplegar la imagen.
\param Mat &frame La imagen sobre la cual vamos a pintar los puntos capturados.
\param vector <Point> *P El vector de puntos en donde se almacenrán las coordenadas capturadas.
*/
int capturaPuntos(const char *Name, Mat &frame, vector<Point> *P)
{
    mouseData mD(P);
    cv::imshow(Name, frame);
    cvSetMouseCallback(Name, on_mouseEvent, (void*) &mD);
    string pictureName;
    while (!mD.Out)
    {
        dibujaPoints(frame, *P);
        cv::imshow(Name, frame);
        if (waitKey( 30 ) >= 0 )
        {
            cout<<"La captura de puntos se terminó"<<endl;
            break;
        
        }
    }
    cvSetMouseCallback(Name, 0, 0);
    /*if ( !mD.Out )
    {  
      cerr << "La captura de puntos se aborto." << endl;
      (*P).clear();
      return 1;
    }*/
    return 0;
}

void getMatPointsMouse(Mat& frame, VideoCapture& capture, vector<Point2d>& pointsRect)
{
  Point coordinate;
  char key; 
    vector<Point>P;

    cvNamedWindow("frame2",WINDOW_NORMAL );//WINDOW_NORmaL);//WINDOW_AUTOSIZE );
    capture>>frame;
    //flip(frame, frame, 1);
	capturaPuntos("frame2", frame, &P);
		if(capturaPuntos("frame", frame, &P))//Si es igual a 1 deja de iterar
                  cout<<"Error en la captura!!"<<endl;
                cout<<"finalizó capturaPuntos"<<endl;
                
		//Imprimimos los puntos capturados:
		if (P.size())
		{
		  //cout << endl << "Se capturaron los siguientes " << P.size() << " puntos:" << endl << endl
		//	   << "P = [";
		  vector <Point>::iterator ini, end;
		  ini = P.begin();
		  end=P.end();		  
		  //for(int i=0; (ini+1) != end;++ini, i++)
		  for(int i=0; i< P.size(); i++)
		  {
                      pointsRect[i].x=P[i].x;
                      pointsRect[i].y=P[i].y;
                      //cout<<pointsRect[i].x<<" "<<pointsRect[i].y<<"; ";
			  //cout << "(" << ini->x << ", " << ini->y << "), ";
			  //myFile<<ini->x<<", "<<ini->y<< "| ";
		  }
                  //cout<<"]"<<endl;
		  //cout << "(" << ini->x << ", " << ini->y << ")]"<< endl;
                   //cout<<"(Mat_<double>(3, 4) <<"<<P[0].x<<", "<<P[1].x<<", "<<P[2].x<<", "<<P[3].x<<", "
                    //                          <<P[0].y<<", "<<P[1].y<<", "<<P[2].y<<", "<<P[3].y<<", "
                     //                         <<"1, 1, 1, 1);"<<endl;
                   /*pointsRect[0].x=P[0].x;
                   pointsRect[0].y=P[0].y;
                   pointsRect[1].x=P[1].x;
                   pointsRect[1].y=P[1].y;
                   pointsRect[2].x=P[2].x;
                   pointsRect[2].y=P[2].y;
                   pointsRect[3].x=P[3].x;
                   pointsRect[3].y=P[3].y;

                   pointsRect[4].x=P[4].x;
                   pointsRect[4].y=P[4].y;
                   pointsRect[5].x=P[5].x;
                   pointsRect[5].y=P[5].y;
                   pointsRect[6].x=P[6].x;
                   pointsRect[6].y=P[6].y;
                   pointsRect[7].x=P[7].x;
                   pointsRect[7].y=P[7].y;


                   pointsRect[8].x=P[8].x;
                   pointsRect[8].y=P[8].y;
                   pointsRect[9].x=P[9].x;
                   pointsRect[9].y=P[9].y;
                   pointsRect[10].x=P[10].x;
                   pointsRect[10].y=P[10].y;
                   pointsRect[11].x=P[11].x;
                   pointsRect[11].y=P[11].y;*/
//pointsRect[3].y=100;

		  //myFile<<ini->x<<", "<<ini->y<< "| ";
		}  
		//myFile<<pictureName<<endl;
       P.clear();
	 //Cierra ventanas que fueron abiertas.
    cvDestroyWindow ("frame2");
  
}
void getVanishingLine (Mat &K, Point3f &N, Point3f &l)
{
    Mat KT, iKT, L, iKTL;

    transpose(K, KT);
    invert(KT, iKT);
    L=(Mat_<double> (3,1) << N.x, N.y, N.z);
    iKTL = iKT*L;
    iKTL /= sqrt(iKTL.dot(iKTL));
    l.x = iKTL.at<double>(0,0);
    l.y = iKTL.at<double>(1,0);
    l.z = iKTL.at<double>(2,0);

}

void drawLine(Mat &I, Point3f &L, const Scalar &color)
{
    double A, B, C, x, y;
    A=L.x;
    B=L.y;
    C=L.z;

    if (B == 0. && A != 0.)
    {
        x = -C / A;
        if (x >= 0 && x < I.cols)
            line (I, Point(x, 0), Point(x,I.rows - 1), color);
        return;
    }
    if (A==0 && B != 0.)
    {
        y = -C / B;
        if (y >= 0 && y < I.rows)
            line (I, Point(0, y), Point(I.cols - 1, y), color);
        return;
    }
    double m, b, yl, yr, xt, xb;
    int cont = 0;

    m = -A/B;
    b = -C/B;

    yl = b;
    yr = m*(I.cols-1)+b;
    xt = -b/m;
    xb = (I.rows-1-b)/m;
    if (yl >= 0 && yl < I.rows)
        cont += 1;
    if (xt >= 0 && xt < I.cols)
        cont += 3;
    if (yr >= 0 && yr < I.rows)
        cont += 5;
    if (xb >= 0 && xb < I.cols)
        cont += 9;
    switch (cont)
    {
        case 4:   //Left & Top
            line (I, Point(0, yl), Point(xt,0), color);
            break;
        case 6:   //Left & Right
            line (I, Point(0, yl), Point(I.cols - 1, yr), color);
            break;
        case 10:  //Left & Bottom
            line (I, Point(0, yl), Point(xb, I.rows - 1), color);
            break;
        case 8:   //Top & Right
            line (I, Point(xt, 0), Point(I.cols -1, yr), color);
            break;
        case 12:  //Top & Bottom
            line (I, Point(xt, 0), Point(xb, I.rows - 1), color);
            break;
        case 14:  //Right & Botton
            line (I, Point(xb, I.rows - 1), Point(I.cols - 1, yr), color);
            break;
        default:
            break;
    }
}

void getIntersec(Mat& frame, vector<Point3f*>& vCorners, Point3f& vanishingLine, Mat& K, Point3f &N, double &D)
{
    double A, B, C;
    A=vanishingLine.x;
    B=vanishingLine.y;
    C=vanishingLine.z;
    //cerca del borde izq
    Point2f corUL;
    int  offset=20;
    corUL.x=offset;
    corUL.y=((-C-A*corUL.x)/B)+offset;
    //corUL.y=(-C-A*corUL.x)/B;
    //Abajo a la izq
    Point2f corBL;
    corBL.x=offset;
    corBL.y=frame.rows-offset;
  
    //Abajo a la der
    Point2f corBR;
    corBR.x=frame.cols-offset;
    corBR.y=frame.rows-offset;

    //Arriba a la der
    Point2f corUR;
    corUR.x=frame.cols-offset;
    corUR.y=((-C-A*corUR.x)/B)+offset;
    //corUR.y=((-C-A*corUR.x)/B);
    Mat temp1, temp2, temp3, temp4, Kinv;
    temp1=(Mat_<double>(3, 1)<<corUL.x, corUL.y, 1);//Pasar a matriz para poder multiplicar por la Kinv
    temp2=(Mat_<double>(3, 1)<<corBL.x, corBL.y, 1);
    temp3=(Mat_<double>(3, 1)<<corBR.x, corBR.y, 1);
    temp4=(Mat_<double>(3, 1)<<corUR.x, corUR.y, 1);
    Kinv=K.inv();
    //A metros
    temp1=Kinv*temp1;
    temp2=Kinv*temp2;
    temp3=Kinv*temp3;
    temp4=Kinv*temp4;
    
    //Normalización
    temp1/=norm(temp1);
    temp2/=norm(temp2);
    temp3/=norm(temp3);
    temp4/=norm(temp4);

    Point3f pUL, pBL, pBR, pUR;//Puntos ya en m y unitarios
    pUL=Point3f(temp1.at<double>(0,0), temp1.at<double>(1,0), temp1.at<double>(2,0));
    pBL=Point3f(temp2.at<double>(0,0), temp2.at<double>(1,0), temp2.at<double>(2,0));
    pBR=Point3f(temp3.at<double>(0,0), temp3.at<double>(1,0), temp3.at<double>(2,0));
    pUR=Point3f(temp4.at<double>(0,0), temp4.at<double>(1,0), temp4.at<double>(2,0));
    float r1Temp, r2Temp, r3Temp, r4Temp;
    r1Temp=(D/N.dot(pUL));
    vCorners[0]->x=pUL.x*r1Temp;
    vCorners[0]->y=pUL.y*r1Temp;
    vCorners[0]->z=pUL.z*r1Temp;

    r2Temp=(D/N.dot(pBL));
    vCorners[1]->x=pBL.x*r2Temp;
    vCorners[1]->y=pBL.y*r2Temp;
    vCorners[1]->z=pBL.z*r2Temp;

    r3Temp=(D/N.dot(pBR));
    vCorners[2]->x=pBR.x*r3Temp;
    vCorners[2]->y=pBR.y*r3Temp;
    vCorners[2]->z=pBR.z*r3Temp;


    r4Temp=(D/N.dot(pUR));
    vCorners[3]->x=pUR.x*r4Temp;
    vCorners[3]->y=pUR.y*r4Temp;
    vCorners[3]->z=pUR.z*r4Temp;

    cout<<"Posiciones 3d"<<endl;
    for(int i=0; i<vCorners.size(); i++)
      cout<<*vCorners[i]<<endl;

}

void points3dTo2d(vector<Point3f*>& vCorners, Mat& K, Mat& dummy, vector<Point2f>& vCorners2D)
{
    Mat P2DCorner, P3DCorner;
    P3DCorner=(Mat_<double>(4, 4)<<vCorners[0]->x,vCorners[1]->x, vCorners[2]->x, vCorners[3]->x, 
                                  vCorners[0]->y,vCorners[1]->y, vCorners[2]->y, vCorners[3]->y, 
                                  vCorners[0]->z,vCorners[1]->z, vCorners[2]->z, vCorners[3]->z, 
                                  1, 1, 1, 1);

    P2DCorner=K*dummy*P3DCorner;
    //Deshomoge...
    cout<<"Posiciones 3d en la imagen"<<endl;
    for(int i=0;i<P2DCorner.cols;++i)
    {
        P2DCorner(Rect(i, 0, 1, 3)) /= P2DCorner.at<double>(2, i);
        vCorners2D[i]=Point2f(P2DCorner.at<double>(0, i), P2DCorner.at<double>(1, i));
        cout<<vCorners2D[i]<<endl;
    }
  
}

void getIntersec2(Mat& frame, vector<Point3f*>& vCorners, Point3f& vanishingLine, Mat& K, Point3f &N, double &D, vector<Point2d>& P)
{
    double A, B, C;
    A=vanishingLine.x;
    B=vanishingLine.y;
    C=vanishingLine.z;
    //cerca del borde izq
    Point2f corUL;
    int  offset=20;
    corUL.x=P[0].x;
    corUL.y=P[0].y;
    //corUL.y=(-C-A*corUL.x)/B;
    //Abajo a la izq
    Point2f corBL;
    corBL.x=P[1].x;
    corBL.y=P[1].y;
  
    //Abajo a la der
    Point2f corBR;
    corBR.x=P[2].x;
    corBR.y=P[2].y;

    //Arriba a la der
    Point2f corUR;
    corUR.x=P[3].x;
    corUR.y=P[3].y;
    //corUR.y=((-C-A*corUR.x)/B);
    Mat temp1, temp2, temp3, temp4, Kinv;
    temp1=(Mat_<double>(3, 1)<<corUL.x, corUL.y, 1);//Pasar a matriz para poder multiplicar por la Kinv
    temp2=(Mat_<double>(3, 1)<<corBL.x, corBL.y, 1);
    temp3=(Mat_<double>(3, 1)<<corBR.x, corBR.y, 1);
    temp4=(Mat_<double>(3, 1)<<corUR.x, corUR.y, 1);
    Kinv=K.inv();
    //A metros
    temp1=Kinv*temp1;
    temp2=Kinv*temp2;
    temp3=Kinv*temp3;
    temp4=Kinv*temp4;
    
    //Normalización
    temp1/=norm(temp1);
    temp2/=norm(temp2);
    temp3/=norm(temp3);
    temp4/=norm(temp4);

    Point3f pUL, pBL, pBR, pUR;//Puntos ya en m y unitarios
    pUL=Point3f(temp1.at<double>(0,0), temp1.at<double>(1,0), temp1.at<double>(2,0));
    pBL=Point3f(temp2.at<double>(0,0), temp2.at<double>(1,0), temp2.at<double>(2,0));
    pBR=Point3f(temp3.at<double>(0,0), temp3.at<double>(1,0), temp3.at<double>(2,0));
    pUR=Point3f(temp4.at<double>(0,0), temp4.at<double>(1,0), temp4.at<double>(2,0));
    float r1Temp, r2Temp, r3Temp, r4Temp;
    r1Temp=(D/N.dot(pUL));
    vCorners[0]->x=pUL.x*r1Temp;
    vCorners[0]->y=pUL.y*r1Temp;
    vCorners[0]->z=pUL.z*r1Temp;

    r2Temp=(D/N.dot(pBL));
    vCorners[1]->x=pBL.x*r2Temp;
    vCorners[1]->y=pBL.y*r2Temp;
    vCorners[1]->z=pBL.z*r2Temp;

    r3Temp=(D/N.dot(pBR));
    vCorners[2]->x=pBR.x*r3Temp;
    vCorners[2]->y=pBR.y*r3Temp;
    vCorners[2]->z=pBR.z*r3Temp;


    r4Temp=(D/N.dot(pUR));
    vCorners[3]->x=pUR.x*r4Temp;
    vCorners[3]->y=pUR.y*r4Temp;
    vCorners[3]->z=pUR.z*r4Temp;

    /*cout<<"Posiciones 3d"<<endl;
    for(int i=0; i<vCorners.size(); i++)
      cout<<*vCorners[i]<<endl;*/

}
void pointsSelect3dTo2D(vector<Point2d>& P, vector<Point2f>& vps2d, Mat& K, Point3f& N, double& D, Mat& dummy)
{
  Mat temp, tempCol;
    temp=(Mat_<double>(3, 1)<<P[0].x,
                                P[0].y,
                                1);
    //Se construye una matriz de 3x12 con todos los puntos
    for(int i=1; i<P.size(); i++)
    {
      tempCol=(Mat_<double>(3, 1)<<P[i].x,
                                P[i].y,
                                1);
      hconcat(temp, tempCol, temp);
    }
    Mat Kinv=K.inv();
    //Pasamos a m
    temp=Kinv*temp;
    //Normalización
    for(int i=0; i<temp.cols; i++)
    {
      temp(Rect(i, 0, 1, 3))/=norm(temp(Rect(i, 0, 1, 3)));
    }
    
    vector<Point3f> vPoints3D;
    //Calculamos la intersección en 3D de los puntos con el plano
    for(int i=0; i<temp.cols; i++)
    {
      Point3f pTemp=Point3f(temp.at<double>(0,i), temp.at<double>(1,i), temp.at<double>(2,i));
      float rTemp=(D/N.dot(pTemp));
      vPoints3D.push_back(Point3f(pTemp.x*rTemp, pTemp.y*rTemp, pTemp.z*rTemp));
    }
    
    //Proyectamos los ptos. 3d en la imagen
    Mat P2DPoints, P3DPoints, tempCol2;
    P3DPoints=(Mat_<double>(4, 1)<<vPoints3D[0].x,
                                  vPoints3D[0].y,
                                  vPoints3D[0].z,
                                  1);

    for(int i=1; i<P.size(); i++)
    {
      tempCol2=(Mat_<double>(4, 1)<<vPoints3D[i].x,
                                vPoints3D[i].y,
                                vPoints3D[i].z,
                                1);
      hconcat(P3DPoints, tempCol2, P3DPoints);
    }
    //cout<<"intersec="<<P3DPoints<<endl;
    P2DPoints=K*dummy*P3DPoints;
    //Deshomoge...
    //cout<<"Posiciones 3d en la imagen"<<endl;
    for(int i=0;i<P2DPoints.cols;++i)
    {
        P2DPoints(Rect(i, 0, 1, 3)) /= P2DPoints.at<double>(2, i);
        vps2d[i]=Point2f(P2DPoints.at<double>(0, i), P2DPoints.at<double>(1, i));
        //cout<<vCorners2D[i]<<endl;
    }

}

/*void getIntersec3(Mat& frame, vector<Point3f*>& vCorners, Point3f& vanishingLine, Mat& K, Point3f &N, double &D, vector<Point2d>& P)
{

    Mat temp1, temp2, temp3, temp4, temp5, temp6, temp7, temp8, temp9, temp10, temp11, temp12, Kinv;
    temp1=(Mat_<double>(3, 1)<<P[0].x, P[0].y, 1);//Pasar a matriz para poder multiplicar por la Kinv
    temp2=(Mat_<double>(3, 1)<<P[1].x, P[1].y, 1);
    temp3=(Mat_<double>(3, 1)<<P[2].x, P[2].y, 1);
    temp4=(Mat_<double>(3, 1)<<P[3].x, P[3].y, 1);

    temp5=(Mat_<double>(3, 1)<<P[4].x, P[4].y, 1);
    temp6=(Mat_<double>(3, 1)<<P[5].x, P[5].y, 1);
    temp7=(Mat_<double>(3, 1)<<P[6].x, P[6].y, 1);
    temp8=(Mat_<double>(3, 1)<<P[7].x, P[7].y, 1);

    temp9=(Mat_<double>(3, 1)<<P[8].x, P[8].y, 1);
    temp10=(Mat_<double>(3, 1)<<P[9].x, P[9].y, 1);
    temp11=(Mat_<double>(3, 1)<<P[10].x, P[10].y, 1);
    temp12=(Mat_<double>(3, 1)<<P[11].x, P[11].y, 1);

    Kinv=K.inv();
    //A metros
    temp1=Kinv*temp1;
    temp2=Kinv*temp2;
    temp3=Kinv*temp3;
    temp4=Kinv*temp4;

    temp5=Kinv*temp5;
    temp6=Kinv*temp6;
    temp7=Kinv*temp7;
    temp8=Kinv*temp8;

    temp9=Kinv*temp1;
    temp10=Kinv*temp10;
    temp11=Kinv*temp11;
    temp12=Kinv*temp12;
    
    //Normalización
    temp1/=norm(temp1);
    temp2/=norm(temp2);
    temp3/=norm(temp3);
    temp4/=norm(temp4);

    temp5/=norm(temp5);
    temp6/=norm(temp6);
    temp7/=norm(temp7);
    temp8/=norm(temp8);

    temp9/=norm(temp9);
    temp10/=norm(temp10);
    temp11/=norm(temp11);
    temp12/=norm(temp12);

    Point3f p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12;//Puntos ya en m y unitarios
    p1=Point3f(temp1.at<double>(0,0), temp1.at<double>(1,0), temp1.at<double>(2,0));
    p2=Point3f(temp2.at<double>(0,0), temp2.at<double>(1,0), temp2.at<double>(2,0));
    p3=Point3f(temp3.at<double>(0,0), temp3.at<double>(1,0), temp3.at<double>(2,0));
    p4=Point3f(temp4.at<double>(0,0), temp4.at<double>(1,0), temp4.at<double>(2,0));
    p5=Point3f(temp5.at<double>(0,0), temp5.at<double>(1,0), temp5.at<double>(2,0));
    p6=Point3f(temp6.at<double>(0,0), temp6.at<double>(1,0), temp6.at<double>(2,0));
    p7=Point3f(temp7.at<double>(0,0), temp7.at<double>(1,0), temp7.at<double>(2,0));
    p8=Point3f(temp8.at<double>(0,0), temp8.at<double>(1,0), temp8.at<double>(2,0));
    p9=Point3f(temp9.at<double>(0,0), temp9.at<double>(1,0), temp9.at<double>(2,0));
    p10=Point3f(temp10.at<double>(0,0), temp10.at<double>(1,0), temp10.at<double>(2,0));
    p11=Point3f(temp11.at<double>(0,0), temp11.at<double>(1,0), temp11.at<double>(2,0));
    p12=Point3f(temp12.at<double>(0,0), temp12.at<double>(1,0), temp12.at<double>(2,0));

    float r1Temp, r2Temp, r3Temp, r4Temp, r5Temp, r6Temp, r7Temp, r8Temp, r9Temp, r10Temp, r11Temp, r12Temp;
    r1Temp=(D/N.dot(p1));
    vCorners[0]->x=p1.x*r1Temp;
    vCorners[0]->y=p2.y*r1Temp;
    vCorners[0]->z=p3.z*r1Temp;

    r2Temp=(D/N.dot(pBL));
    vCorners[1]->x=pBL.x*r2Temp;
    vCorners[1]->y=pBL.y*r2Temp;
    vCorners[1]->z=pBL.z*r2Temp;

    r3Temp=(D/N.dot(pBR));
    vCorners[2]->x=pBR.x*r3Temp;
    vCorners[2]->y=pBR.y*r3Temp;
    vCorners[2]->z=pBR.z*r3Temp;


    r4Temp=(D/N.dot(pUR));
    vCorners[3]->x=pUR.x*r4Temp;
    vCorners[3]->y=pUR.y*r4Temp;
    vCorners[3]->z=pUR.z*r4Temp;

    cout<<"Posiciones 3d"<<endl;
    for(int i=0; i<vCorners.size(); i++)
      cout<<*vCorners[i]<<endl;
}*/

#endif

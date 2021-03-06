#ifndef __LEVENBERG__
#define __LEVENBERG__

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>
#include <cstdlib>
#include <cstdio>
#include "tools.h"

using namespace std;
using namespace cv;

// funciones y parametros globales
unsigned numFunEval = 0;      // contador de llamadas a funcion

void evalFunc( void* param, double* X,  double *F, Mat& Xscn,  Mat& Ximg, Mat& R, Mat& Tr, Point3d& Nplane, double& D)
{
  cout<<endl<<endl<<"Entrando a evalFunc"<<endl;
  double *val = (double*) param; 
  int M, col, iters, nColN;
  double cosw, sinw, Dtemp, rtemp, fx, fy, fz;
  Mat scPt, Rtemp, Ttemp, Column;
  Point3d Ximgi, Xscni, Ximg1, Ntemp, firstC, secondC, thirdC, scPt2;
  M=int(val[1]);
  iters=M/3;//son 3 coordenadas por cada punto

  cosw=(1-cos(X[6]));
  cout<<"normw= "<<X[6]<<endl;
  sinw=sin(X[6]);
   col=0;
  firstC=Point3d((X[0]*X[0]-1)*cosw+1, 
                (X[2]*sinw+X[0]*X[1]*cosw),
                X[0]*X[2]*cosw-X[1]*sinw);

  secondC=Point3d((X[0]*X[1]*cosw)-(X[2]*sinw),
                  ((X[1]*X[1]-1)*cosw)+1,
                  X[0]*sinw+X[1]*X[2]*cosw);

  thirdC=Point3d(X[1]*sinw+X[0]*X[2]*cosw, 
           (-X[0]*sinw)+X[1]*X[2]*cosw,
           ((X[2]*X[2]-1)*cosw)+1);
  Ntemp=thirdC;//N es la última columna de la matriz de rotación
   Rtemp=(Mat_<double>(3,3)<<firstC.x, secondC.x, thirdC.x,
                                firstC.y, secondC.y, thirdC.y,
                                firstC.z, secondC.z, thirdC.z);//Creamos una matriz temporal de Rot para hallar la "d" de la ec. del plano 
   Ttemp=(Mat_<double>(3,1)<<X[3],
                              X[4],
                            X[5]);
   cout<<"R="<<Rtemp<<endl<<"T="<<Ttemp<<endl;
   R=Rtemp.clone();
   Tr=Ttemp.clone();
  nColN=0;

   Column=(Mat_<double>(3,1)<<Xscn.at<double>(0,nColN),
                              Xscn.at<double>(1,nColN),
                              Xscn.at<double>(2,nColN));
  //Column=Xscn(Rect(0,0,1,3));

   //Column = Xscn(Rect(0,0,1,3));
  scPt=Rtemp * Column;
  scPt+=Ttemp;
   //scPt=Rtemp*Xscn(Rect(11,0,1,3))+Ttemp;//
   scPt2=Point3d(scPt.at<double>(0,nColN),
                scPt.at<double>(1,nColN),
                scPt.at<double>(2,nColN));
   Dtemp=Ntemp.dot(scPt2);
   Nplane=Ntemp;
   D=Dtemp;
   rtemp=0;
  //Hasta este punto ya obtuvimos la nueva R, T, N y D que estamos optimizando

  // salida de la funcion
  for(int i=0; i<144; i+=3, col++)
  //for(int i=0; i<iters; i+=3, col++)
  {
    Xscni=Point3d(Xscn.at<double>(0, col),
                  Xscn.at<double>(1, col), 
                  Xscn.at<double>(2, col));

    Ximgi=Point3d(Ximg.at<double>(0, col),
                  Ximg.at<double>(1, col), 
                  Ximg.at<double>(2, col));
    rtemp=Dtemp/(Ntemp.dot(Ximgi));
    fx=(firstC.x*Xscni.x+secondC.x*Xscni.y+thirdC.x*Xscni.z+X[3])-(rtemp*Ximgi.x);
    fy=(firstC.y*Xscni.x+secondC.y*Xscni.y+thirdC.y*Xscni.z+X[4])-(rtemp*Ximgi.y);
    fz=(firstC.z*Xscni.x+secondC.z*Xscni.y+thirdC.z*Xscni.z+X[5])-(rtemp*Ximgi.z);
    F[col]=fx*fx+fy*fy+fz*fz;
    cout<<"f["<<col<<"]="<<F[col]<<", "<<endl;
    /*
    F[i]=pow((firstC.x*Xscni.x+secondC.x*Xscni.y+thirdC.x*Xscni.z+X[3])-(rtemp*Ximgi.x),2);
    F[i+1]=pow((firstC.y*Xscni.x+secondC.y*Xscni.y+thirdC.y*Xscni.z+X[4])-(rtemp*Ximgi.y),2);
    F[i+2]=pow((firstC.z*Xscni.x+secondC.z*Xscni.y+thirdC.z*Xscni.z+X[5])-(rtemp*Ximgi.z),2);
    cout<<"f["<<i<<"]="<<F[i]<<", "<<
    "f["<<i+1<<"]="<<F[i+1]<<", "<<
    "f["<<i+2<<"]="<<F[i+2]<<", "<<endl;*/

  }
  
  // llamadas a la evaluacion
  numFunEval++; 
}

void Func( void* param, double* x,  double *f, double* J, Mat& Xscn, Mat& Ximg, Mat& R, Mat& Tr , Point3d& Nplane, double& D)
{
  // obtiene parametros del problema
  double *val = (double*) param; 
  int N = int(val[0]);
  int M = int(val[1]);
  
  // salida de la funcion
  
  //cout << "A" << endl; cout.flush();
  evalFunc(param,x,f, Xscn, Ximg, R, Tr, Nplane, D);
  //cout << "B" << endl; cout.flush();
#if 1
  // calcula el gradiente
  const double EPS=sqrt(DBL_EPSILON);
  volatile double temp;
  for(int r=0; r<N; r++)
  {
	  // Encontrar el tamaño de h adecuado
	  temp=x[r];  // guarda valor original
	  double h=EPS*fabs(temp);
	  if(h<EPS)
		h=EPS;
	  x[r]=temp+h;			//Trick to reduce finite precision error
	  h=x[r]-temp;
	  evalFunc(param,x,&J[r*M], Xscn, Ximg, R, Tr, Nplane, D);
	  daxpy_(M,-1.0,f,1, &J[r*M],1);
	  dscal_(M,1.0/h,&J[r*M],1);		//Usar este o el siguiente for
	  //~ for (int c=0; c<M; c++)		//Usar este o el dscal anterior
		//~ J[c+r*M]/=h;
	  x[r]=temp;
          //break;//Debugeando..
  }
  
#endif
  // llamadas a la evaluacion
  //numFunEval++;
}

void setW(Point3d& W, double Wnorm, Mat& R)
{
 W=Point3d(R.at<double>(2,1)-R.at<double>(1, 2), 
          R.at<double>(0,2)-R.at<double>(2, 0), 
          R.at<double>(1,0)-R.at<double>(0, 1)); 
//cout<<"(2, 1): "<<R.at<double>(2,1)<<endl;
//W=W*(Wnorm/(2*sin(Wnorm)));
W=W*(1/(2*sin(Wnorm)));//Normalizado
}

void levenbergMain(Mat& R, Mat& Tr, Mat& K, Mat& frame, Mat& Ximg, Mat& scnPts, vector<Point2d>& P)
{
    Mat marcas3d, marcas2d, mDummy, Xscn, frameCopy;
    Size_<int> sizeG(4,4);
    Mat G(sizeG, R.type());
    Point3d Nplane;
    Point3f Nplanef;
    double D;
      Scalar_<double> traza2;
      traza2=trace(R);//Traza de R
      double traza=traza2[0];
      double Wnorm=acos((traza-1)/2);
      Point3d W;
      setW(W,Wnorm, R);
      cout<<"W="<<endl<<W<<endl; 
      int nP=Ximg.cols;//número de puntos con los que se trabajará!!!!!!!!!!!!!!!!!!
      Mat Id=Mat::eye(R.size(), R.type());
      Mat wg=(Mat_<double>(3,3)<<0, -W.z, W.y,
                                W.z, 0, -W.x,
                                -W.y, W.x, 0);

      Mat wg2=(Mat_<double>(3,3)<<W.x*W.x-1,W.x*W.y, W.x*W.z,
                                  W.x*W.y, W.y*W.y-1, W.y*W.z,
                                  W.x*W.z, W.y*W.z, W.z*W.z-1);
    Xscn=scnPts(Rect(0,0,nP,3));
    mDummy=(Mat_<double>(3, 4)<<1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
    vector<Point2f>vps2d;
    for(int i=0; i<nP; i++)
      vps2d.push_back(Point2f(0,0));

      //cout<<"wnorm="<<Wnorm<<endl<<"Id= "<<Id<<endl<<"wg = "<<wg<<endl<<"wg2="<<wg2<<endl;

      //Número de parámetros a optimizar
	const int N = 7;//Wx, Wy, Wz, Tx, Ty, Tz, Wnorm, Wnorm es necesario optimizarlo porque W depende de él
	//const int M = nP*Xscn.rows;
	const int M = nP;

	//  double parametros[] = {1.0, 100.0,double(N)};    // example 4.3
	double parametros[] = {double(N),double(M)};      // ecaucion (5.5)
	void *dummy = parametros;
	
	// declara variables de computo
	double* x = (double*) vectorr(N,sizeof(double));
	double* xw = (double*) vectorr(N,sizeof(double));
	double* g = (double*) vectorr(N,sizeof(double));
	//double* go = (double*) vectorr(N,sizeof(double));
	double* h = (double*) vectorr(N,sizeof(double));
	//double* v = (double*) vectorr(N,sizeof(double));
	double* A = (double*) vectorr(N*N,sizeof(double));
	//double* D = (double*) vectorr(N*N,sizeof(double));
	double* B = (double*) vectorr(N*M,sizeof(double));
	double* Jw = (double*) vectorr(N*M,sizeof(double));
	double* J = (double*) vectorr(N*M,sizeof(double));
	double* f = (double*) vectorr(M,sizeof(double));//función
	double* fw = (double*) vectorr(M,sizeof(double));
	double* I = (double*) vectorr(N*N,sizeof(double));
	double  mu, nu=2;
	double tao=1.0e-3;
	
	// valores iniciales
	double xo[N] = {W.x, W.y, W.z, Tr.at<double>(0,0), Tr.at<double>(1,0), Tr.at<double>(2,0), Wnorm}; 
        
	double epsilon1 = 1.0e-10, epsilon2 = 1.0e-10, normG;

	int pos,info,kmax=1000,k=0;
	
	//inicializa variables de cómputo
	dcopy_( N, xo, 1, x, 1 ); 	//posicion inicial
	Func(dummy,x,f,B, Xscn, Ximg, R, Tr, Nplane, D);			//valores iniciales
	cout<< " F(X)              : " << (0.5*ddot_(M,f,1,f,1)) << endl; cout.flush();

//Antes del for
    cout<<"Antes del for(;;)"<<endl;
    buildG(G, R, Tr);//Se actualiza la matriz G
    marcas3d=G*scnPts;//Rotamos y trasladamos los puntos 3d 
    marcas2d=K*mDummy*marcas3d;//Los proyectamos en la imagen
    //Deshomoge...
    for (int i=0;i<marcas2d.cols;++i)
        marcas2d(Rect(i, 0, 1, 3)) /= marcas2d.at<double>(2, i);

    pointsSelect3dTo2D(P, vps2d, K, Nplanef, D, mDummy);//Con los puntos seleccionados del tablero se halla la intersección en el plano y se proyectan en 2D

      frameCopy=frame.clone();
    //Se pintan los resultados de los dos métodos: intersección con el plano y rot+tr
      for(int j=0; j<marcas2d.cols; j++)
      {
        for(int i=0; i<vps2d.size(); i++)
          circle(frameCopy, vps2d[i],1, Scalar(0, 0, 200) ,2);//Se pintan los puntos de las intersecciones

        Point tempPoint(marcas2d.at<double>(0, j), marcas2d.at<double>(1, j));
        circle(frameCopy, tempPoint,1, Scalar(0, 200, 0) ,2);//Se pintan los puntos rotados y trasladados
      }
          imshow("frame", frameCopy);
          waitKey();
//**************************Despliegue*****************

        cout<<"Fin"<<endl;

	dscal_(N*N,0.0,I,1);		//matriz I
	for(int r=0;r<N;r++)
		I[r*N+r]=1.0;
	const char U='U';
	const char T='T';
	dsyrk_(U,T,N,M,1.0,B,M,0.0,A,N);	//matriz A=B'B=J'J
	dgemv_(&T,M,N,1.0,B,M,f,1.0,0.0,g,1);	//g=B'f
	pos=idamax_(N,g,1)-1;		//norma INF de g
	normG=fabs(g[pos]);
	pos=idamax_(N,A,N);			//amortiguamiento inicial
	mu=tao*fabs(A[pos*N+pos]);
	dcopy_( N*M, B, 1, J, 1 ); 	
	//inicia iteraciones
	bool found=(normG<=epsilon1);
        cvNamedWindow("frame",WINDOW_NORMAL  );
        Mat frameDummy=Mat::zeros(R.size(), CV_8UC1);

	while(!found && (k<kmax))
	{
		//obtiene paso Gauss-Newton
		dcopy_(N*N,I,1,A,1);//I->A
		//cout<<"A antes: "<<A[0]<<", "<<A[1]<<", "<<A[2]<<", "<<A[3]<<endl;
		dsyrk_(U,T,N,M,1.0,J,M,mu,A,N);	//matriz A=(J'J + muI). 
		dcopy_(N,g,1,h,1);
		dscal_(N,-1.0,h,1);
		
		dposv_(U,N,1,A,N,h,N,info);		//resuelve el sistema Ah=-g
		if(info > 0)
		{	
			cout << "\n\nError en la funcion de LAPACK...!!!\n";
		}
		//prueba criterios de paro
		if(dnrm2_(N,h,1)<=epsilon2*(dnrm2_(N,x,1)+epsilon2))
			found=true;
		else
		{
			//estima nueva posicion y valores
			dcopy_(N,x,1,xw,1);
			daxpy_(N,1.0,h,1,xw,1);
			Func(dummy,xw,fw,Jw, Xscn, Ximg, R, Tr, Nplane, D);
                        Nplanef.x=Nplane.x;
                        Nplanef.y=Nplane.y;
                        Nplanef.z=Nplane.z;
			
			double ro=(ddot_(M,f,1,f,1)-ddot_(M,fw,1,fw,1))/(mu*ddot_(N,h,1,h,1)-ddot_(N,h,1,g,1)); 
			//double ro=(ddot_(M,f,1,f,1)-ddot_(M,fw,1,fw,1))/(1.0/3.0)*(mu*ddot_(N,h,1,h,1)-ddot_(N,g,1,g,1)); 
			if(ro>0.0)
			{
				//actualiza valores
				dcopy_(N,xw,1,x,1);
				dcopy_(M,fw,1,f,1);
				dcopy_(M*N,Jw,1,J,1);
				//dsyrk_(U,T,N,M,1.0,B,M,0.0,A,N);
				dgemv_("T",M,N,1.0,J,M,f,1.0,0.0,g,1);	//vector g=J'f				
				//actualiza parametro de amortiguamiento
				double v1=1.0/3.0;
				double v2=1.0-pow(2.0*ro-1.0,3.0); 
				mu*=((v1>v2) ? v1:v2);
				nu=2.0;
			}
			else
			{			
					mu*=nu;
					nu*=2.0;
			}			
			pos=idamax_(N,g,1)-1; 	//norma INF de g
			normG=fabs(g[pos]);
			found=(normG<=epsilon1);

		}
			
		k++;			//contador


    buildG(G, R, Tr);//Se actualiza la matriz G
    marcas3d=G*scnPts;//Rotamos y trasladamos los puntos 3d 
    marcas2d=K*mDummy*marcas3d;//Los proyectamos en la imagen
    //Deshomoge...
    for (int i=0;i<marcas2d.cols;++i)
        marcas2d(Rect(i, 0, 1, 3)) /= marcas2d.at<double>(2, i);

    pointsSelect3dTo2D(P, vps2d, K, Nplanef, D, mDummy);//Con los puntos seleccionados del tablero se halla la intersección en el plano y se proyectan en 2D

      frameCopy=frame.clone();
    //Se pintan los resultados de los dos métodos: intersección con el plano y rot+tr
      for(int j=0; j<marcas2d.cols; j++)
      {
        for(int i=0; i<vps2d.size(); i++)
          circle(frameCopy, vps2d[i],1, Scalar(0, 0, 200) ,2);//Se pintan los puntos de las intersecciones

        Point tempPoint(marcas2d.at<double>(0, j), marcas2d.at<double>(1, j));
        circle(frameCopy, tempPoint,1, Scalar(0, 200, 0) ,2);//Se pintan los puntos rotados y trasladados
      }

          imshow("frame", frameCopy);
	 cout<< " F(X)              : " << (0.5*ddot_(M,f,1,f,1)) << endl; cout.flush();
          waitKey(10);

	}  

	// despliega valores
	cout << endl << "Valores finales" << endl 
		<< " num. iteraciones  : " << k << endl
		<< " num. eval. func.  : " << numFunEval << endl
		<< scientific 
		<< " F(X)              : " << (0.5*ddot_(M,f,1,f,1)) << endl
		<< " ||g||            : " << normG << endl
		<< endl;
	cout<<endl<<"posicion optima"<<endl<<"x=["<<endl;
	
	for(int r=0;r<N;r++)
		cout<<"	"<<x[r]<<endl;
	cout<<"]"<<endl<<endl;
        //cout<<"K="<<K<<endl;
        cout<<"R="<<R<<endl<<"T = "<<Tr<<endl;
        cout<<"N = "<<Nplanef<<endl<<"D = "<<D<<endl;
	
    //última pintada    
    buildG(G, R, Tr);//Se actualiza la matriz G
    marcas3d=G*scnPts;//Rotamos y trasladamos los puntos 3d 
    marcas2d=K*mDummy*marcas3d;//Los proyectamos en la imagen
    //Deshomoge...
    for (int i=0;i<marcas2d.cols;++i)
        marcas2d(Rect(i, 0, 1, 3)) /= marcas2d.at<double>(2, i);

    pointsSelect3dTo2D(P, vps2d, K, Nplanef, D, mDummy);//Con los puntos seleccionados del tablero se halla la intersección en el plano y se proyectan en 2D

      frameCopy=frame.clone();
    //Se pintan los resultados de los dos métodos: intersección con el plano y rot+tr
      for(int j=0; j<marcas2d.cols; j++)
      {
        for(int i=0; i<vps2d.size(); i++)
          circle(frameCopy, vps2d[i],1, Scalar(0, 0, 200) ,2);//Se pintan los puntos de las intersecciones

        Point tempPoint(marcas2d.at<double>(0, j), marcas2d.at<double>(1, j));
        circle(frameCopy, tempPoint,1, Scalar(0, 200, 0) ,2);//Se pintan los puntos rotados y trasladados
      }

          imshow("frame", frameCopy);
          waitKey();


	// free memory
	free_vectorr( (void*) x );
	free_vectorr( (void*) f );
	free_vectorr( (void*) B );
	free_vectorr( (void*) xw );
	free_vectorr( (void*) fw );
	free_vectorr( (void*) g );
	free_vectorr( (void*) h );
	free_vectorr( (void*) A );
	free_vectorr( (void*) I );
        
}


#endif

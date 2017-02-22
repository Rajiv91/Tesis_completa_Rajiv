//****************************************************************************
//
//  Function definition of memory management for vectors and matrixs, save graphics files
//  and others
//
// Author       : Ricardo Legarda Saenz
// Language     : C++
// Compiler     : Borland C++ v5.0
// Environment  : Win32
// Revisions
//   Initial    : 14.08.2000
//   Last       : 25.04.2001
//
//****************************************************************************


#ifndef _MEMORIA_HPP
#define _MEMORIA_HPP     // include just once

// declaration of function used here
#include <iostream>



// funtion declaration for data type
void** matrix( unsigned long, unsigned long, unsigned long );
void free_matrix( void** );
void* vectorr( unsigned long, unsigned long );
void free_vectorr( void* );

// Declara estructuras para manejo de arreglos de datos.
struct Matrix
{
  unsigned ren;      // numero de renglones.
  unsigned col;      // numero de columnas
  float** data;      // apuntador a los datos del arreglo
  Matrix( unsigned r = 0, unsigned c = 0, float** d = NULL )            // inicialización de los datos del arreglo
    {  ren = r;  col = c;  data = d; }
};

struct MatrixD
{
  unsigned ren;      // numero de renglones.
  unsigned col;       // numero de columnas
  double** data;    // apuntador a los datos del arreglo
  MatrixD( unsigned r = 0, unsigned c = 0, double** d = NULL )            // inicialización de los datos del arreglo
    {  ren = r;  col = c;  data = d; }
};

struct MatrixByte
{
  unsigned ren;                 // numero de renglones.
  unsigned col;                  // numero de columnas
  unsigned char** data;    // apuntador a los datos del arreglo
  MatrixByte( unsigned r = 0, unsigned c = 0, unsigned char** d = NULL )            // inicialización de los datos del arreglo
    {  ren = r;  col = c;  data = d; }
};

struct MatrixU
{
  unsigned ren;         // numero de renglones.
  unsigned col;          // numero de columnas
  unsigned** data;    // apuntador a los datos del arreglo
  MatrixU( unsigned r = 0, unsigned c = 0, unsigned** d = NULL )            // inicialización de los datos del arreglo
    {  ren = r;  col = c;  data = d; }
};


// declaration for BLAS/LAPACK functions, double precision                  
struct _dcomplex { double re, im; };
typedef struct _dcomplex dcomplex;
extern "C"
{
  // BLAS 1, real
  double dnrm2_(const int&,const double*,const int&);
  double ddot_(const int&,const double*,const int&,const double*,const int&);
  void scopy_(const int&,const float*,const int&,const float*,const int&);
  void dcopy_(const int&,const double*,const int&,const double*,const int&);
  void dscal_(const int&,const double&,const double*,const int&);
  void daxpy_(const int&,const double&,const double*,const int&,const double*,const int&);
  int idamax_(const int&,const double*,const int&);
  // BLAS 1, complex
  double dznrm2_(const int&, const dcomplex*, const int& );  
  void zcopy_(const int&, const dcomplex*, const int&, const dcomplex*, const int& );
  void zdscal_(const int&, const double&, const dcomplex*, const int& );

  // BLAS 2
  void dgemv_(const char*,const int&,const int&,const double&,const double*,const int&,const double*,const int&,const double&,const double*,const int&);
  void dsymv_(const char*,const int&,const double&,const double*,const int&,const double*,const int&, const double&,const double*,const int&);
  void dger_(const int&,const int&,const double&,const double*,const int&,const double*,const int&,const double*, int&);
  void dsyr2_(const char*,const int&,const double&,const double*,const int&,const double*,const int&,const double*,const int&);
  void dsyr_(const char*,const int&,const double&,const double*,const int&,const double*, int&);
  void zgemv_(const char&,const int&,const int&,const double&,const dcomplex*,const int&,const dcomplex*,const int&,const double&,const dcomplex*,const int&);
  
  // BLAS 3
  void dgemm_(const char&,const char&,const int&,const int&,const int&,const double&,const double*,const int&,const double*,const int&,const double&,const double*,const int&);
  void dtrsm_(const char&,const char&,const char&,const char&,const int&,const int&,const double&,const double*,const int&,const double*,const int&);
  void zgemm_(const char&,const char&,const int&,const int&,const int&,const double&,const dcomplex*,const int&,const dcomplex*,const int&,const double&,const dcomplex*,const int&);
	void dsyrk_(const char&,const char&,const int&,const int&,const double&,const double*,const int&,const double&,const double*,const int&);
 
  // LAPACK
  void dgesv_(const int&,const int&,const double*,const int&,const int*,const double*,const int&,const int&);
  void dpotrf_(const char&,const int&,const double*,const int&,const int&);
  void dpotrs_(const char&,const int&,const int&,const double*,const int&,const double*,const int&, const int&);
  void dgelss_(const int&,const int&,const int&,const double*,const int&,const double*,const int&,const double*,const double*,const int&,const double*,const int&,const int&);
  void dgetrf_(const int&,const int&,const double*,const int&,const int&,const int&);
  void dgetrs_(const char&,const int&,const int&,const double*,const int&,const int&,const double*,const int&,const int&);
  void dgels_(const char&,const int&,const int&,const int&,const double*,const int&,const double*,const int&,const double*,const int&,const int&);  
  void zgels_(const char&,const int&,const int&,const int&,const dcomplex*,const int&,const dcomplex*,const int&,const dcomplex*,const int&,const int&);
	void dposv_(const char&,const int&,const int&,const double*,const int&,const double*,const int&,const int&);
}

#endif

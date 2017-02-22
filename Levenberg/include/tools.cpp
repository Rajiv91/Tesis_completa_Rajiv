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
//   Last       : 27.04.2001
//
//****************************************************************************


// declaration of function used here
#include "tools.h"      // function declaration
#include <fstream>   // funciones de manejo de archivos
#include <cfloat>
#include <cmath>
#include <new>          // exception-handling functions
using namespace std;



//*************************************************************************
//
// For all the allocation memory functions
//
// Inputs :
//     rows [=] number of rows
//     cols [=] number of columns
// Output :
//     return a memory pointer if it is allocated
//     otherwise, finish the program with error
//
//*************************************************************************

//*************************************************************************
//  functions for data type VOID
//*************************************************************************
void** matrix( unsigned long rows, unsigned long cols, unsigned long num_bytes )
{
  // define variables
  char** data = NULL;
  char*  vect = NULL;
  unsigned long k, num;

  // allocate memory with exceptions test
  try
    {
      data = new char*[rows];         // allocate rows
      num = cols*num_bytes;
      vect = (char*) vectorr( (rows*cols), num_bytes );
      for ( k = 0; k < rows; k++ )
        data[k] = &vect[k*num];       // allocate columns
    }
  catch ( bad_alloc )                  // an error ??
    {
      delete [] (char*) vect;        // yes, exit with error signal
      delete [] data;
      throw;
    }
    
	// return pointer 
  return (void **) data;      
}

// desallocate memory used by a matrix
void free_matrix( void** data )
{
  if ( data != NULL )
    {
      free_vectorr( data[0] );    // desallocate columns
      if ( data != NULL )
        delete [] data;          // desallocate rows
      data = NULL;
    }
}

// allocate memory for vector
void* vectorr( unsigned long cols, unsigned long num_bytes )
{
  // define variables
  char* data = NULL;
  unsigned long k, num;

  // allocate memory with exceptions test
  try
    {  
    	num = cols*num_bytes/sizeof (char);
    	data = new char[num];			  // allocate columns
    }   
  catch ( bad_alloc )               // an error ??
    {
      delete [] (char *) data;               // yes, exit with error signal
      throw;
    }

  // clean memory, return pointer
  for ( k = 0; k < num; k++ )
    data[k] = 0;
  return (void*) data;
}

// desallocate memory used by a vector
void free_vectorr( void* data )
{
  if ( data != NULL )
    delete [] (char *) data;
  data = NULL;
}



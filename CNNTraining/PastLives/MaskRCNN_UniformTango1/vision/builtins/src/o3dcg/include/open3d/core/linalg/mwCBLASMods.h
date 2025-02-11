/*============================================================
* This file was added by MathWorks staff.
* This file contains declarations and macros for mwcblas_sgemm and 
* mwcblas_dgemm functions
*============================================================*/

//***********************************************************************
#include <blas.h>

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {            /* Assume C declarations for C++ */
#endif /* __cplusplus */

/*
 * Enumerated and derived types
 */
#ifdef WeirdNEC
   #define CBLAS_INDEX long
#else
    #define CBLAS_INDEX int
#endif

typedef enum {CblasRowMajor=101, CblasColMajor=102} CBLAS_LAYOUT;
typedef enum {CblasNoTrans=111, CblasTrans=112, CblasConjTrans=113} CBLAS_TRANSPOSE;
typedef enum {CblasUpper=121, CblasLower=122} CBLAS_UPLO;
typedef enum {CblasNonUnit=131, CblasUnit=132} CBLAS_DIAG;
typedef enum {CblasLeft=141, CblasRight=142} CBLAS_SIDE;

typedef CBLAS_LAYOUT CBLAS_ORDER; /* this for backward compatibility with CBLAS_ORDER */

//https://www.ibm.com/docs/en/zos/2.3.0?topic=mdd-operator
#if defined(_WIN32)
   #define F77_GLOBAL(lcname,UCNAME)  lcname
#else
   #define F77_GLOBAL(lcname,UCNAME)  lcname##_
#endif

#ifdef __cplusplus
}
#endif
// #endif //CBLAS_H

//***********************************************************************
// #include "cblas_f77.h"
//***********************************************************************
/*
 * cblas_f77.h
 * Written by Keita Teranishi
 *
 * Updated by Jeff Horner
 * Merged cblas_f77.h and cblas_fortran_header.h
 */

#ifndef CBLAS_F77_H
#define CBLAS_F77_H

#ifdef CRAY
   #include <fortran.h>
   #define F77_CHAR _fcd
   #define C2F_CHAR(a) ( _cptofcd( (a), 1 ) )
   #define C2F_STR(a, i) ( _cptofcd( (a), (i) ) )
   #define F77_STRLEN(a) (_fcdlen)
#endif

#ifdef WeirdNEC
   #define F77_INT long
#endif

#ifdef  F77_CHAR
   #define FCHAR F77_CHAR
#else
   #define FCHAR char *
#endif

#ifdef F77_INT
   #define FINT const F77_INT *
   #define FINT2 F77_INT *
#else
   #define FINT const int *
   #define FINT2 int *
#endif

/*
 * Level 1 BLAS
 */

#define F77_xerbla 		F77_GLOBAL(xerbla,XERBLA)
/*
 * Level 3 BLAS
 */
#define F77_sgemm 		F77_GLOBAL(sgemm,SGEMM)
#define F77_dgemm 		F77_GLOBAL(dgemm,DGEMM)

#ifdef __cplusplus
extern "C" {
#endif

//***********************************************************************
// Function declarations
// mwcblas_xerbla mimics cblas_xerbla function from CBLAS interface
void mwcblas_xerbla(ptrdiff_t info, const char *rout, const char *form, ...);
// mwcblas_sgemm mimics cblas_sgemm function from CBLAS interface
void mwcblas_sgemm(const CBLAS_LAYOUT layout, const CBLAS_TRANSPOSE TransA,
                 const CBLAS_TRANSPOSE TransB, const ptrdiff_t M, const ptrdiff_t N,
                 const ptrdiff_t K, const float alpha, const float  *A,
                 const ptrdiff_t lda, const float  *B, const ptrdiff_t ldb,
                 const float beta, float  *C, const ptrdiff_t ldc);
// mwcblas_dgemm mimics cblas_dgemm function from CBLAS interface
void mwcblas_dgemm(const CBLAS_LAYOUT layout, const CBLAS_TRANSPOSE TransA,
                 const CBLAS_TRANSPOSE TransB, const ptrdiff_t M, const ptrdiff_t N,
                 const ptrdiff_t K, const double alpha, const double  *A,
                 const ptrdiff_t lda, const double  *B, const ptrdiff_t ldb,
                 const double beta, double  *C, const ptrdiff_t ldc);                 
//***********************************************************************

#ifdef __cplusplus
}
#endif

#endif /*  CBLAS_F77_H */
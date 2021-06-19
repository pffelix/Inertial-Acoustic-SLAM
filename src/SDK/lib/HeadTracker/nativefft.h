// fft.h: Fast fourier calculation based on Cooley-Tukey algorithm
// Copyright 2020, Felix Pfreundtner, All rights reserved.
// adapted from https://www.physics.nus.edu.sg/~phywjs/CZ5101/fft.c

/********************************************************************
F A S T   F O U R I E R   T R A N S F O R M   P R O G R A M S

by Wang Jian-Sheng 4 Nov 1998, added fft2D(), 11 Apr 2003
---------------------------------------------------------------------

Reference: "Computational Frameworks for the Fast Fourier
Transform", Charles Van Loan, SIAM, 1992.

There are many FFT algorithms, the most important ones are
        COOLEY-TUKEY:  in place, bit reversal
        STOCKHAM AUTOSORT:  additional memory size of input data
MIXED RADIX:  20% less operations comparing to Cooley-Tukey
        PRIME FACTOR: arbitrary length n

        We use a combination of the Stockham autosort algorithm 1.7.2,
page 57, and multirfow Cooley-Tukey (3.1.7), page 124, of the
reference above.

The discrete Fourier transform is defined by
        y[k] = sum_(j=0,n-1) x[j] exp(-2 Pi sqrt[-1] j k/n),
k=0,1,...,n-1.  The factor (1/n) is not included.
If y[]<-x[]; fft(x,n,1); fft(x,n,-1); then y[]==x[]/n is true.
Three dimensional transform is generalized straightforwardly.

Interface and usage:
1D Fourier transform
Use: fft(x, n, flag)
x    : an array of structure type cpx;
n    : the size of data, must be a power of 2;
flag : 1 for forward transform, -1 for inverse transform.

3D Fourier transform
Use :  fft3D(x, n1, n2, n3, flag)
x    : 1D array of type cpx representing 3D array;
mapping through C convention, i.e.,
(i,j,k) -> k + n3*j + n2*n3*i;
n1, n2, n3 : dimensions in three dirfections;
flag : same as in 1D.

2D FFT is similar but with n1 and n2 only.

**********************************************************************/

/* Data type and new names for flexibility:

    real:    Basic data type for floating point computations
             (typedef double  real;)
    cpx: Structure for complex numbers, real and imaginary parts
             are referred as c.real, c.imag.
             (typedef struct { real Re; real Im; }  cpx;)
*/

// Copyright 2020, Felix Pfreundtner, All rights reserved.
// nativefft.h: Native float or double Radix 2 implementation of cooley-tukey fft.

/* Inclusion of standard C libraries */
#pragma once
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include "params.h"
#include "debugging.h"


/* Mathematical functions and constants */
//#undef M_PI
#define REALSIZE 4
#if (REALSIZE == 16)
#define sinnn  sinl
#define cosss  cosl
#define fabsss  fabsl
#define NATIVEFFT_PI 3.1415926535897932384626433832795L
#else
#if (REALSIZE == 4)
#define NATIVEFFT_PI 3.14159265358979323846F
#define sinnn  sinf
#define cosss  cosf
#define fabsss  fabsf
#else
#define NATIVEFFT_PI 3.1415926535897932385E0
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

	static void stockham(struct cpx *x, long n, int flag, long n2, struct cpx *y);
	void cooley_tukey(struct cpx *x, long n, int flag, long n2);
	void nativefft(struct cpx *x, long n, int flag);
	void nativefft_stockham(struct cpx *x, long n, int flag);
	void nativefft2D(struct cpx *x, long n1, long n2, int flag);
	void nativefft3D(struct cpx *x, long n1, long n2, long n3, int flag);

#ifdef __cplusplus
};
#endif

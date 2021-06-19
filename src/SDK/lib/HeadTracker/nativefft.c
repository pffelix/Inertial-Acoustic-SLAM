// Copyright 2020, Felix Pfreundtner, All rights reserved.
#include "nativefft.h"


/*----------------------------------------------------------------------*/
/* Truncated Stockham algorithm for multi-column vector,
       X(n1,n2) <- F_n1 X(n1,n2)
   x[] input data of size n, viewed as n1 = n/n2 by n2 dimensional
   array, flag =+1 forward transform, -1 for backward transform, y[] is
   working space, which must be n in size.  This function is supposed
   to be internal (static), not used by application.  Note that the
   terminology of column or row respect to algorithms in the Loan's
   book is reversed, because we use row major convention of C.
*/
static void stockham(struct cpx *x, long n, int flag, long n2, struct cpx *y)
{
    struct cpx  *y_orig, *tmp;
    long  i, j, k, k2, Ls, r, jrs;
    long  half, m, m2;
    float wr, wi, tr, ti;							/* change to double if needed*/

    y_orig = y;
    r = half = n >> 1;
    Ls = 1;                                         /* Ls=L/2 is the L star */

    while(r >= n2) {                              /* loops log2(n/n2) times */
        tmp = x;                           /* swap pointers, y is always old */
        x = y;                                   /* x is always for new data */
        y = tmp;
        m = 0;                        /* m runs over first half of the array */
        m2 = half;                             /* m2 for second half, n2=n/2 */
        for(j = 0; j < Ls; ++j) {
            wr = cosss(NATIVEFFT_PI*j/Ls);                   /* real and imaginary part */
            wi = -flag * sinnn(NATIVEFFT_PI*j/Ls);                      /* of the omega */
            jrs = j*(r+r);
            for(k = jrs; k < jrs+r; ++k) {           /* "butterfly" operation */
                k2 = k + r;
                tr =  wr*y[k2].real - wi*y[k2].imag;      /* cpx multiply, w*y */
                ti =  wr*y[k2].imag + wi*y[k2].real;
                x[m].real = y[k].real + tr;
                x[m].imag = y[k].imag + ti;
                x[m2].real = y[k].real - tr;
                x[m2].imag = y[k].imag - ti;
                ++m;
                ++m2;
            }
        }
        r  >>= 1;
        Ls <<= 1;
    };

    if (y != y_orig) {                     /* copy back to permanent memory */
        for(i = 0; i < n; ++i) {               /* if it is not already there */
            y[i] = x[i];               /* performed only if log2(n/n2) is odd */
        }
    }

    assert(Ls == n/n2);                        /* ensure n is a power of 2  */
    assert(1 == n || m2 == n);           /* check array index within bound  */
}


/* The Cooley-Tukey multiple column algorithm, see page 124 of Loan.
   x[] is input data, overwritten by output, viewed as n/n2 by n2
   array. flag = 1 for forward and -1 for backward transform.
*/
void cooley_tukey(struct cpx *x, long n, int flag, long n2)
{
    struct cpx c;
    long i, j, k, m, p, n1;
    long Ls, ks, ms, jm, dk;
    float wr, wi, tr, ti;				/* change to double if needed*/


    n1 = n/n2;                               /* do bit reversal permutation */
    for(k = 0; k < n1; ++k) {        /* This is algorithms 1.5.1 and 1.5.2. */
        j = 0;
        m = k;
        p = 1;                               /* p = 2^q,  q used in the book */
        while(p < n1) {
            j = 2*j + (m&1);
            m >>= 1;
            p <<= 1;
        }
        assert(p == n1);                   /* make sure n1 is a power of two */
        if(j > k) {
            for(i = 0; i < n2; ++i) {                     /* swap k <-> j row */
                c = x[k*n2+i];                              /* for all columns */
                x[k*n2+i] = x[j*n2+i];
                x[j*n2+i] = c;
            }
        }
    }

    /* This is (3.1.7), page 124 */
    p = 1;
    while(p < n1) {
        Ls = p;
        p <<= 1;
        jm = 0;                                                /* jm is j*n2 */
        dk = p*n2;
        for(j = 0; j < Ls; ++j) {
            wr = cosss(NATIVEFFT_PI*j/Ls);                   /* real and imaginary part */
            wi = -flag * sinnn(NATIVEFFT_PI*j/Ls);                      /* of the omega */
            for(k = jm; k < n; k += dk) {                      /* "butterfly" */
                ks = k + Ls*n2;
                for(i = 0; i < n2; ++i) {                      /* for each row */
                    m = k + i;
                    ms = ks + i;
                    tr =  wr*x[ms].real - wi*x[ms].imag;
                    ti =  wr*x[ms].imag + wi*x[ms].real;
                    x[ms].real = x[m].real - tr;
                    x[ms].imag = x[m].imag - ti;
                    x[m].real += tr;
                    x[m].imag += ti;
                }
            }
            jm += n2;
        }
    }
}



/* 1D Fourier transform:
   Simply call cooley_tukey with proper arguments.
   Allocated working space of size n dynamically.
*/
void nativefft(struct cpx *x, long n, int flag)
{
    //cpx *y;

    assert(1 == flag || -1 == flag);

    // y = (cpx *) malloc( n*sizeof(cpx) );
    // assert(NULL != y);
    cooley_tukey(x, n, flag, 1);
    //free(y);
}

/* 1D Fourier transform: 
   Simply call stockham with proper arguments.  
   Allocated working space of size n dynamically.
*/
void nativefft_stockham(struct cpx *x, long n, int flag)
{
   struct cpx *y;
   
   assert(1 == flag || -1 == flag);
   y = (struct cpx *) malloc( n*sizeof(struct cpx) );
   assert(NULL != y);
   stockham(x, n, flag, 1, y);
   free(y);
}


/* 3D Fourier transform:
   The index for x[m] is mapped to (i,j,k) by
   m = k + n3*j + n3*n2*i, i.e. the row major convention of C.
   All indices start from 0.
   This algorithm requires working space of n2*n3.
   Stockham is efficient, good stride feature, but takes extra
   memory same size as input data; Cooley-Tukey is in place,
   so we take a compromise of the two.
*/

void nativefft2D(struct cpx *x, long n1, long n2, int flag)
{
    struct cpx *y;
    long i, n;

    assert(1 == flag || -1 == flag);
    n = n1*n2;
    y = (struct cpx *) malloc( n2*sizeof(struct cpx) );
    assert(NULL != y);

    for(i=0; i < n; i += n2) {                                  /* FFT in y */
        stockham(x+i, n2, flag, 1, y);
    }
    free(y);
    cooley_tukey(x, n, flag, n2);                               /* FFT in x */
}

void nativefft3D(struct cpx *x, long n1, long n2, long n3, int flag)
{
    struct cpx *y;
    long i, n, n23;

    assert(1 == flag || -1 == flag);
    n23 = n2*n3;
    n = n1*n23;
    y = (struct cpx *) malloc( n23*sizeof(struct cpx) );
    assert(NULL != y);

    for(i=0; i < n; i += n3) {                                  /* FFT in z */
        stockham(x+i, n3, flag, 1, y);
    }
    for(i=0; i < n; i += n23) {                                 /* FFT in y */
        stockham(x+i, n23, flag, n3, y);
    }
    free(y);
    cooley_tukey(x, n, flag, n23);                              /* FFT in x */
}
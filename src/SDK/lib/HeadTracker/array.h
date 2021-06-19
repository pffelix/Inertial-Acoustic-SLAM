// array.h: Matrix operations
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include <math.h>
#include "debugging.h"

#ifdef __cplusplus
extern "C" {
#endif

	// Matrix declarations (free() needed)
	//////////////////////////////////////////////////

	// one dimensional float array declaration
	float * ar_declare(long M);

	// one dimensional double array declaration
	double * ar_declare_double(long M);

	// reset one dimensional float array by filling up with value
	void ar_set(float *x, long N, float value);

	// one dimensional float array filled up with zeros
	float * ar_zeros(long N);

	// one dimensional double array filled up with zeros
	double * ar_zeros_double(long N);

	// one dimensional float array filled up with value
	float * ar_value(long N, float value);

	// linearly increased array from start value with increment incr for N times
	float * ar_linear(float start, double incr, long N);

	// squared increased array from start value with linear increment incr for N times
	float* ar_linearsquare(float start, double incr, long N);

	// square root increased array from start value with linear increment incr for N times
	float* ar_linearsquareroot(float start, double incr, long N);

	// linearly increased array from start value with increment incr for N times (faster but less precise)
	float * ar_linear_fast(float start, double incr, long N);

	// linearly increased array from start value with integer increment incr for N times
	float * ar_linear_int(float start, int incr, long N);

	// squared increased array from start value with linear increment incr for N times
	float* ar_linearsquare_int(float start, int incr, long N);

	// square root increased array from start value with linear increment incr for N times
	float* ar_linearsquareroot_int(float start, int incr, long N);

	// Creates a linearly spaced double array
	double* ar_linear_double(double start, double incr, long N);

	// two dimensional float array declaration
	float ** ar_declare2d(long M, long N);

	// two dimensional float array filled up with zeros
	float ** ar_zeros2d(long M, long N);

	// two dimensional quadratic float array filled up with value
	float ** ar_value2d(long M, long N, float value);

	// two dimensional quadratic float array filled up with value on diagonal
	float ** ar_valuediag2d(long M, float value);

	// reset two dimensional float array by filling up with value
	void ar_set2d(float** x, long M, long N, float value);

	// free two dimensional float array
	void ar_free2d(float **x, long M, long N);

	// free two dimensional long array
	void ar_free2dl(long** x, long M, long N);

	// free two dimensional char array
	void ar_free2d_char(const char **x, long M, long N);

	// three dimensional float array declaration
	float *** ar_declare3d(long M, long N, long O);

	// three dimensional float array filled up with zeros
	float *** ar_zeros3d(long M, long N, long O);

	// three dimensional float array filled up with value
	float*** ar_value3d(long M, long N, long O, float value);

	// free three dimensional float array
	void ar_free3d(float ***x, long M, long N, long O);

	// free three dimensional float array
	void ar_free3d_char(const char ***x, long M, long N, long O);

	// one dimensional long array declaration
	long * ar_declarel(long M);

	// two dimensional long array declaration
	long ** ar_declare2dl(long M, long N);

	// three dimensional long array declaration
	long *** ar_declare3dl(long M, long N, long O);

	// four dimensional long array declaration
	long **** ar_declare4dl(long M, long N, long O, long P);

	// one dimensional long array filled up with zeros
	long * ar_zerosl (long N);

	// two dimensional long aMray filled up with zeros
	long ** ar_zeros2dl (long M, long N);

	// three dimensional long array filled up with zeros
	long *** ar_zeros3dl (long M, long N, long O);

	// four dimensional long array filled up with zeros
	long **** ar_zeros4dl (long M, long N, long O, long P);

	// reset one dimensional long array by filling up with value
	void ar_setl(long *x, long N, long value);

	// reset one dimensional long array by filling up with value
	void ar_set2dl(long **x, long M, long N, long value);

	// one dimensional integer array declaration
	int * ar_declarei(int N);

	// one dimensional integer array filled up with value
	int * ar_valuei(int N, int value);

	// linearly increased integer array from start value with integer increment incr for N times
	int* ar_lineari(int start, int incr, int N);

	// two dimensional integer array declaration
	int ** ar_declare2di(int M, int N);

	// reset one dimensional integer array by filling up with value
	void ar_seti(int* x, int N, int value);

	// reset two dimensional integer array by filling up with value
	void ar_set2di(int** x, int M, int N, int value);

	// one dimensional bool array declaration
	bool * ar_declareb(int N);

	// one dimensional bool array filled up with value
	bool * ar_valueb(int N, bool value);

	// reset one dimensional boolean array by filling up with value
	void ar_setb(bool* x, int N, bool value);

	// two dimensional bool array declaration
	bool ** ar_declare2db(int M, int N);

	// two dimensional bool array filled up with value
	bool ** ar_value2db(int M, int N, bool value);

	// reset two dimensional boolean array by filling up with value
	void ar_set2db(bool** x, int M, int N, bool value);

	// Matrix operations wiht return (no free() needed)
	//////////////////////////////////////////////////
	
	// index of maximum of 1d float array
	long ar_maxn(float *x, long N);

	// index of maximum of 2d float array
	float ar_max2d(float** x, long M, long N);

	// index of maximum of 3d float array
	float ar_max3d(float*** x, long M, long N, long O);

	// index of absolute maximum of 1d float array
	long ar_absmaxn(float* x, long N);

	// maximum of 1d int array
	int ar_maxi(int* x, int N);

	// sum of all 1d array values
	float ar_sum(float *x, long N);

	// squared sum of all 1d array values
	float ar_sumsquared(float *x, long N);

	// absolute sum of all 1d array values
	float ar_sumabs(float* x, long N);



	// Matrix operations (no free() needed)
	//////////////////////////////////////////////////
	// index and amplitude of absolute maximum of 1d float array
	void ar_absmax(float* x, long N, long* max_n, float* max_amp);

	// mean absolute sum of all 1d array values
	void ar_sumabsaverage(float* x, long N, float *y);

	// mean of all positive 1d array values
	void sample_mean_positive(float* x, long* N, float* y, long* Npositive);

	// mean of all 1d array values with negative values flipped sign
	void sample_mean_flipped(float* x, long* N, float* y);

	// index and amplitude of maximum of 1d array
	void ar_max(float* x, long N, long* max_n, float* max_amp);

	// index and amplitude of Nmax maxima of 1d array
	void ar_Nmaxi(float* x, int N, int* max_n, float* max_amp, int Nmax);

	// index and amplitude of Nmax maxima of 2d array
	void ar_Nmax2di(float** x, int M, int N, int* max_m, int* max_n, float* max_amp, int Nmax);

	// index and amplitude of minimum of 1d array
	void ar_min(float* x, long N, long* min_N, float* min_amp);

	// square array values
	void ar_square(float* x, long N);

	// multiply array with constant value
	void ar_multiply(float* x, long N, float value);

	// multiply 2d array with constant value
	void ar_multiply2d(float** x, long M, long N, float value);

	// divide array by constant value
	void ar_divide(float* x, long N, float value);

	// divide array by constant value
	void ar_divide2d(float** x, long M, long N, float value);

	// normalize array
	void ar_norm(float* x, long N, float * y);

	// mean sum 1d array with 1d array
	void ar_smean(float * x1, float * x2, long N, float * y);

	// multiply 1d array with value
	void ar_mvalue(float* x, long N, float value, float* y);

	// composed matrix operations: (no free() needed)
	//////////////////////////////////////////////////
	// transpose 1d array
	extern inline void ar_tp(float *x, long N, float * y);

	// negate 1d array
	extern inline void ar_neg(float *x, long N, float * y);

	// sum 1d array with 1d array
	extern inline void ar_s(float * x1, float * x2, long N, float * y);

	// transpose 2d array
	extern inline void ar_tp2d(float **x, long M, long N, float ** y);

	// negate 2d array (3 entries)
	extern inline void ar_neg2d(float **x, long M, long N, float ** y);

	// multiply 2d with 2d array
	extern inline void ar_m2d(float** x1, float** x2, long M1, long N1M2, long N2, float** y);

	// sum 2d with 2d array
	extern inline void ar_s2d(float ** x1, float ** x2, long M, long N, float ** y);

	// multiply 2d with 1d array
	extern inline void ar_m2d1d(float ** x1, float * x2, long M, long N, float * y);

	// gauss jordan inverse of 2d square matrix (changes input too and is slow around 40ms for 20x20 matrix)
	extern inline void ar_inv2d(float** x, long MN, float** y);

	// composed matrix operations: 3 entries (no free() needed)
	//////////////////////////////////////////////////

	// transpose one dimensional array (3 entries) 
	extern inline void ar_tpM3(float *x, float * y);

	// negated 1d array (3 entries)
	extern inline void ar_negM3(float *x, float * y);

	// sum 1d array with 1d array (3 entries)
	extern inline void ar_sM3(float * x1, float * x2, float * y);

	// transpose two dimensional array (3 entries)
	extern inline void ar_tp2dM3(float **x, float ** y);

	// negated 2d array (3 entries)
	extern inline void ar_neg2dM3(float **x, float ** y);

	// multiply 2d with 2d array (3 entries)
	extern inline void ar_m2dM3(float ** x1, float ** x2, float ** y);

	// sum 2d with 2d array (3 entries)
	extern inline void ar_s2dM3(float ** x1, float ** x2, float ** y);

	// multiply 2d with 1d array (3 entries)
	extern inline void ar_m2d1dM3(float ** x1, float * x2, float * y);

	// analytical inverse 2d quadratic array (3 entries)
	extern inline void ar_inv2dM3(float ** x, float ** y);


#ifdef __cplusplus
};
#endif
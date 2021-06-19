// array.h: Matrix operations
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#define active 0
#pragma once
#include <math.h>
#include "debugging.h"
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#if (active == 1)
	// Matrix declarations (free() needed)
	//////////////////////////////////////////////////
	// one dimensional float array declaration
	float * ar_declare(long M);

	// one dimensional float array filled up with zeros
	float * ar_zeros(long N);

	// one dimensional float array filled up with value
	float * ar_value(long N, float value);

	// linearly increased array from start value with increment incr for N times
	float * ar_linear(float start, double incr, long N);

	// linearly increased array from start value with increment incr for N times (faster but less precise)
	float * ar_linear_fast(float start, double incr, long N);

	// free one dimensional float array
	void ar_free(float *x, long M);

	// two dimensional float array declaration
	float ** ar_declare2d(long M, long N);

	// two dimensional float array filled up with zeros
	float ** ar_zeros2d(long M, long N);

	// two dimensional quadratic float array filled up with value on diagonal
	float ** ar_valuediag2d(long M, float value);

	// free two dimensional float array
	void ar_free2d(float **x, long M);




	// Matrix operations (no free() needed)
	//////////////////////////////////////////////////
	
	// index at maximum of 1d array
	long ar_maxn(float *x, long N);

	// sum of all 1d array values
	float ar_sum(float *x, long N);

	// squared sum of all 1d array values
	float ar_sumsquared(float *x, long N);

	// absolute sum of all 1d array values
	float ar_sumabs(float* x, long N);


	// Matrix operations (free() needed)
	//////////////////////////////////////////////////
	// normalize array
	void ar_norm(float* x, long N, float *y);

	// transpose 1d array
	float * ar_tp(float *x, long N);

	// negated 1d array
	float * ar_neg(float *x, long N);

	// sum 1d array with 1d array (3 entries)
	float * ar_s(float * x1, float * x2, long N);

	// transpose 2d array
	float ** ar_tp2d(float **x, long M, long N);

	// negated 2d array (3 entries)
	float ** ar_neg2d(float **x, long M, long N);

	// multiply 2d with 2d array
	float ** ar_m2d(float ** x1, float ** x2, long M, long N);

	// sum 2d with 2d array
	float ** ar_s2d(float ** x1, float ** x2, long M, long N);

	// multiply 2d with 1d array
	float * ar_m2d1d(float ** x1, float * x2, long M, long N);


	// composed matrix operations: 3 entries (free() needed)
	//////////////////////////////////////////////////
	// transpose one dimensional array (3 entries) 
	float * ar_tpM3(float *x);

	// negated 1d array (3 entries)
	float * ar_negM3(float *x);

	// sum 1d array (3 entries)
	float * ar_sM3(float * x1, float * x2);

	// transpose two dimensional array (3 entries)
	float ** ar_tp2dM3(float **x);

	// negated 2d array (3 entries)
	float ** ar_neg2dM3(float **x);

	// multiply 2d with 2d array (3 entries)
	float ** ar_m2dM3(float ** x1, float ** x2);

	// sum 2d with 2d array (3 entries)
	float ** ar_s2dM3(float ** x1, float ** x2);

	// multiply 2d with 1d array (3 entries)
	float * ar_m2d1dM3(float ** x1, float * x2);

	// inverse 2d quadratic array (3 entries)
	float ** ar_inv2dM3(float ** x);

#endif

#ifdef __cplusplus
};
#endif